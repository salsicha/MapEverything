//
//  DepthPointCloudMetalProcessor.swift
//  MapEverything
//

import Foundation
import CoreVideo
import Metal
import simd

/// GPU port of the per-pixel Depth Anything work in
/// `PointCloudProcessor.processDepthAnythingPointCloud`: inverse-depth
/// calibration, unprojection through the scaled intrinsics, YCbCr color
/// sampling, and the map-frame transform. The kernel mirrors the CPU math
/// operation-for-operation so results match within float tolerance, and the
/// caller falls back to the CPU path whenever this returns nil.
///
/// All Metal state is guarded by `lock`; the camera planes are bound
/// zero-copy through a CVMetalTextureCache.
nonisolated final class DepthPointCloudMetalProcessor: @unchecked Sendable {
    // Must mirror the MSL DepthPointUniforms layout exactly: the float4x4
    // first (16-byte aligned), then twelve 4-byte scalars.
    private struct Uniforms {
        var transform: simd_float4x4
        var fx: Float
        var fy: Float
        var cx: Float
        var cy: Float
        var scale: Float
        var offset: Float
        var minDepth: Float
        var maxDepth: Float
        var depthWidth: UInt32
        var depthHeight: UInt32
        var imageWidth: UInt32
        var imageHeight: UInt32
    }

    // packed_float3 position (12 bytes) + uchar4 color (4 bytes); color.w is
    // the validity flag so invalid pixels compact away in output order.
    private static let outputStride = 16

    private static let kernelSource = """
    #include <metal_stdlib>
    using namespace metal;

    struct DepthPointUniforms {
        float4x4 transform;
        float fx;
        float fy;
        float cx;
        float cy;
        float scale;
        float offset;
        float minDepth;
        float maxDepth;
        uint depthWidth;
        uint depthHeight;
        uint imageWidth;
        uint imageHeight;
    };

    struct GPUColoredPoint {
        packed_float3 position;
        uchar4 color;
    };

    kernel void unprojectDepthAnything(
        device const float *relativeDepth [[buffer(0)]],
        device GPUColoredPoint *outPoints [[buffer(1)]],
        constant DepthPointUniforms &u [[buffer(2)]],
        texture2d<float, access::read> yTexture [[texture(0)]],
        texture2d<float, access::read> cbcrTexture [[texture(1)]],
        uint2 gid [[thread_position_in_grid]]
    ) {
        if (gid.x >= u.depthWidth || gid.y >= u.depthHeight) {
            return;
        }
        uint index = gid.y * u.depthWidth + gid.x;
        device GPUColoredPoint &out = outPoints[index];
        out.color = uchar4(0, 0, 0, 0);

        // Mirrors DepthAnythingProcessor.calibratedMetricDepth exactly.
        float r = relativeDepth[index];
        if (!isfinite(r) || !(r > 0.0f)) {
            return;
        }
        float inverseDepth = u.scale * r + u.offset;
        if (!isfinite(inverseDepth) || !(inverseDepth > 0.0f)) {
            return;
        }
        float depth = 1.0f / inverseDepth;
        if (!(depth > u.minDepth) || !(depth < u.maxDepth)) {
            return;
        }

        // Mirrors the projection-table factors and camera-space convention.
        float xf = (float(gid.x) - u.cx) / u.fx;
        float yf = (u.cy - float(gid.y)) / u.fy;
        float4 cameraPoint = float4(xf * depth, yf * depth, -depth, 1.0f);
        float4 world = u.transform * cameraPoint;

        // Mirrors the CPU image index math: truncate, then clamp.
        int imageX = int((float(gid.x) / float(u.depthWidth)) * float(u.imageWidth));
        int imageY = int((float(gid.y) / float(u.depthHeight)) * float(u.imageHeight));
        imageX = min(int(u.imageWidth) - 1, max(0, imageX));
        imageY = min(int(u.imageHeight) - 1, max(0, imageY));

        float yv = yTexture.read(uint2(imageX, imageY)).r * 255.0f;
        float2 cbcr = cbcrTexture.read(uint2(imageX >> 1, imageY >> 1)).rg * 255.0f;
        float cb = cbcr.x - 128.0f;
        float cr = cbcr.y - 128.0f;
        float red = yv + 1.402f * cr;
        float green = yv - 0.344136f * cb - 0.714136f * cr;
        float blue = yv + 1.772f * cb;

        out.position = packed_float3(world.x, world.y, world.z);
        out.color = uchar4(
            uchar(clamp(red, 0.0f, 255.0f)),
            uchar(clamp(green, 0.0f, 255.0f)),
            uchar(clamp(blue, 0.0f, 255.0f)),
            1
        );
    }
    """

    private let lock = NSLock()
    private let device: MTLDevice
    private let commandQueue: MTLCommandQueue
    private let pipeline: MTLComputePipelineState
    private let textureCache: CVMetalTextureCache
    private var depthBuffer: MTLBuffer?
    private var outputBuffer: MTLBuffer?

    init?() {
        guard let device = MTLCreateSystemDefaultDevice(),
              let commandQueue = device.makeCommandQueue() else {
            return nil
        }

        let options = MTLCompileOptions()
        // Fast math folds away the NaN checks the validity gates rely on.
        options.mathMode = .safe

        guard let library = try? device.makeLibrary(source: Self.kernelSource, options: options),
              let function = library.makeFunction(name: "unprojectDepthAnything"),
              let pipeline = try? device.makeComputePipelineState(function: function) else {
            return nil
        }

        var cache: CVMetalTextureCache?
        guard CVMetalTextureCacheCreate(kCFAllocatorDefault, nil, device, nil, &cache) == kCVReturnSuccess,
              let cache else {
            return nil
        }

        self.device = device
        self.commandQueue = commandQueue
        self.pipeline = pipeline
        self.textureCache = cache
    }

    /// Returns nil when the camera buffer cannot be bound or GPU execution
    /// fails; the caller then uses the CPU path.
    func processDepthAnythingPointCloud(
        cameraImage pixelBuffer: CVPixelBuffer,
        intrinsics: simd_float3x3,
        imageResolution resolution: CGSize,
        transform: simd_float4x4,
        relativeDepthMap: RelativeDepthMap,
        calibration: DepthAnythingProcessor.MaximumLikelihoodCalibration
    ) -> [ColoredPoint]? {
        let depthWidth = relativeDepthMap.width
        let depthHeight = relativeDepthMap.height
        guard depthWidth > 0, depthHeight > 0,
              resolution.width > 0, resolution.height > 0 else {
            return nil
        }

        let imageWidth = CVPixelBufferGetWidth(pixelBuffer)
        let imageHeight = CVPixelBufferGetHeight(pixelBuffer)
        guard imageWidth > 0, imageHeight > 0,
              CVPixelBufferGetPlaneCount(pixelBuffer) >= 2 else {
            return nil
        }

        // Same scaled-intrinsics expressions as the CPU projection table.
        let scaleX = Float(depthWidth) / Float(resolution.width)
        let scaleY = Float(depthHeight) / Float(resolution.height)
        let fx = intrinsics[0][0] * scaleX
        let fy = intrinsics[1][1] * scaleY
        let cx = intrinsics[2][0] * scaleX
        let cy = intrinsics[2][1] * scaleY
        guard fx.isFinite, fy.isFinite, cx.isFinite, cy.isFinite,
              abs(fx) > 1e-5, abs(fy) > 1e-5 else {
            return nil
        }

        lock.lock()
        defer { lock.unlock() }

        let pixelCount = depthWidth * depthHeight
        guard let depthBuffer = reusableBuffer(&self.depthBuffer, length: pixelCount * MemoryLayout<Float>.stride),
              let outputBuffer = reusableBuffer(&self.outputBuffer, length: pixelCount * Self.outputStride) else {
            return nil
        }

        let depthPointer = depthBuffer.contents().assumingMemoryBound(to: Float.self)
        relativeDepthMap.withReadAccess { reader in
            for y in 0..<depthHeight {
                for x in 0..<depthWidth {
                    depthPointer[y * depthWidth + x] = reader.value(atX: x, y: y)
                }
            }
        }

        // The CVMetalTexture wrappers must outlive GPU execution.
        guard let yTextureRef = planeTexture(pixelBuffer, plane: 0, format: .r8Unorm),
              let cbcrTextureRef = planeTexture(pixelBuffer, plane: 1, format: .rg8Unorm),
              let yTexture = CVMetalTextureGetTexture(yTextureRef),
              let cbcrTexture = CVMetalTextureGetTexture(cbcrTextureRef) else {
            return nil
        }

        var uniforms = Uniforms(
            transform: transform,
            fx: fx,
            fy: fy,
            cx: cx,
            cy: cy,
            scale: calibration.scale,
            offset: calibration.offset,
            minDepth: DepthAnythingProcessor.minimumCalibratedDepth,
            maxDepth: DepthAnythingProcessor.maximumCalibratedDepth,
            depthWidth: UInt32(depthWidth),
            depthHeight: UInt32(depthHeight),
            imageWidth: UInt32(imageWidth),
            imageHeight: UInt32(imageHeight)
        )

        guard let commandBuffer = commandQueue.makeCommandBuffer(),
              let encoder = commandBuffer.makeComputeCommandEncoder() else {
            return nil
        }

        encoder.setComputePipelineState(pipeline)
        encoder.setBuffer(depthBuffer, offset: 0, index: 0)
        encoder.setBuffer(outputBuffer, offset: 0, index: 1)
        encoder.setBytes(&uniforms, length: MemoryLayout<Uniforms>.stride, index: 2)
        encoder.setTexture(yTexture, index: 0)
        encoder.setTexture(cbcrTexture, index: 1)

        let threadWidth = pipeline.threadExecutionWidth
        let threadHeight = max(1, pipeline.maxTotalThreadsPerThreadgroup / threadWidth)
        let threadsPerGroup = MTLSize(width: threadWidth, height: threadHeight, depth: 1)
        let groups = MTLSize(
            width: (depthWidth + threadWidth - 1) / threadWidth,
            height: (depthHeight + threadHeight - 1) / threadHeight,
            depth: 1
        )
        encoder.dispatchThreadgroups(groups, threadsPerThreadgroup: threadsPerGroup)
        encoder.endEncoding()

        commandBuffer.commit()
        commandBuffer.waitUntilCompleted()
        guard commandBuffer.status == .completed else {
            return nil
        }

        withExtendedLifetime((yTextureRef, cbcrTextureRef)) {}

        // Compact valid slots in row-major order, matching CPU output order.
        var points: [ColoredPoint] = []
        points.reserveCapacity(pixelCount)
        let raw = outputBuffer.contents()
        for index in 0..<pixelCount {
            let base = raw.advanced(by: index * Self.outputStride)
            let flag = base.load(fromByteOffset: 15, as: UInt8.self)
            guard flag == 1 else { continue }
            let position = SIMD3<Float>(
                base.load(fromByteOffset: 0, as: Float.self),
                base.load(fromByteOffset: 4, as: Float.self),
                base.load(fromByteOffset: 8, as: Float.self)
            )
            let color = SIMD3<UInt8>(
                base.load(fromByteOffset: 12, as: UInt8.self),
                base.load(fromByteOffset: 13, as: UInt8.self),
                base.load(fromByteOffset: 14, as: UInt8.self)
            )
            points.append(ColoredPoint(position: position, color: color))
        }
        return points
    }

    private func reusableBuffer(_ slot: inout MTLBuffer?, length: Int) -> MTLBuffer? {
        if let existing = slot, existing.length >= length {
            return existing
        }
        let buffer = device.makeBuffer(length: length, options: .storageModeShared)
        slot = buffer
        return buffer
    }

    private func planeTexture(
        _ pixelBuffer: CVPixelBuffer,
        plane: Int,
        format: MTLPixelFormat
    ) -> CVMetalTexture? {
        let width = CVPixelBufferGetWidthOfPlane(pixelBuffer, plane)
        let height = CVPixelBufferGetHeightOfPlane(pixelBuffer, plane)
        var texture: CVMetalTexture?
        let status = CVMetalTextureCacheCreateTextureFromImage(
            kCFAllocatorDefault,
            textureCache,
            pixelBuffer,
            nil,
            format,
            width,
            height,
            plane,
            &texture
        )
        guard status == kCVReturnSuccess else { return nil }
        return texture
    }
}
