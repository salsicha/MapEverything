//
//  VideoExporter.swift
//  Reconstructor
//

import Foundation
import SceneKit
import AVFoundation
import SceneKit.ModelIO
import UIKit

class VideoExporter {
    /// Generates a 10-second MP4 cinematic flythrough of a 3D model using an orbiting virtual camera.
    static func exportFlythrough(objURL: URL, filename: String) async -> URL? {
        guard let docDir = FileManager.default.cloudDocumentsURL else { return nil }
        let outputURL = docDir.appendingPathComponent("\(filename).mp4")
        try? FileManager.default.removeItem(at: outputURL)

        guard let asset = MDLAsset(url: objURL) as MDLAsset? else { return nil }
        let scene = SCNScene(mdlAsset: asset)
        
        // Apply a clean gray material to highlight the LiDAR geometry
        scene.rootNode.enumerateChildNodes { node, _ in
            if let geometry = node.geometry {
                let material = SCNMaterial()
                material.diffuse.contents = UIColor.lightGray
                material.lightingModel = .physicallyBased
                geometry.materials = [material]
            }
        }

        let cameraNode = SCNNode()
        let camera = SCNCamera()
        cameraNode.camera = camera
        scene.rootNode.addChildNode(cameraNode)

        let ambientLight = SCNNode()
        ambientLight.light = SCNLight()
        ambientLight.light?.type = .ambient
        ambientLight.light?.intensity = 800
        scene.rootNode.addChildNode(ambientLight)
        
        let directionalLight = SCNNode()
        directionalLight.light = SCNLight()
        directionalLight.light?.type = .directional
        directionalLight.light?.intensity = 1000
        scene.rootNode.addChildNode(directionalLight)

        let (min, max) = scene.rootNode.boundingBox
        
        let center = SCNVector3((min.x + max.x)/2, (min.y + max.y)/2, (min.z + max.z)/2)
        let radius = Swift.max(max.x - min.x, max.y - min.y, max.z - min.z)
        let distance = radius * 1.8

        let renderer = SCNRenderer(device: MTLCreateSystemDefaultDevice(), options: nil)
        renderer.scene = scene
        renderer.autoenablesDefaultLighting = true

        guard let assetWriter = try? AVAssetWriter(outputURL: outputURL, fileType: .mp4) else { return nil }
        let settings: [String: Any] = [
            AVVideoCodecKey: AVVideoCodecType.h264,
            AVVideoWidthKey: 1080,
            AVVideoHeightKey: 1080
        ]
        let writerInput = AVAssetWriterInput(mediaType: .video, outputSettings: settings)
        let attributes: [String: Any] = [
            kCVPixelBufferPixelFormatTypeKey as String: Int(kCVPixelFormatType_32ARGB),
            kCVPixelBufferWidthKey as String: 1080,
            kCVPixelBufferHeightKey as String: 1080,
            kCVPixelBufferCGImageCompatibilityKey as String: true,
            kCVPixelBufferCGBitmapContextCompatibilityKey as String: true
        ]
        let adaptor = AVAssetWriterInputPixelBufferAdaptor(assetWriterInput: writerInput, sourcePixelBufferAttributes: attributes)

        assetWriter.add(writerInput)
        assetWriter.startWriting()
        assetWriter.startSession(atSourceTime: .zero)

        let targetNode = SCNNode()
        targetNode.position = center
        scene.rootNode.addChildNode(targetNode)
        cameraNode.constraints = [SCNLookAtConstraint(target: targetNode)]

        let fps: Int32 = 30
        let duration: Double = 10.0
        let totalFrames = Int(duration * Double(fps))

        let colorSpace = CGColorSpaceCreateDeviceRGB() // Cached outside loop for massive performance boost
        
        // Render loop for the 360-degree orbit
        for frame in 0..<totalFrames {
            autoreleasepool {
                let angle = (Double(frame) / Double(totalFrames)) * .pi * 2.0
                cameraNode.position = SCNVector3(
                    center.x + Float(cos(angle) * Double(distance)),
                    center.y + Float(radius * 0.8), // Elevated viewing angle
                    center.z + Float(sin(angle) * Double(distance))
                )
                directionalLight.position = cameraNode.position
                directionalLight.look(at: center)

                while !writerInput.isReadyForMoreMediaData {
                    Thread.sleep(forTimeInterval: 0.01)
                }

                let renderTime = CFTimeInterval(frame) / Double(fps)
                let image = renderer.snapshot(atTime: renderTime, with: CGSize(width: 1080, height: 1080), antialiasingMode: .multisampling4X)

                if let pixelBuffer = pixelBufferFromImage(image: image, size: CGSize(width: 1080, height: 1080), pool: adaptor.pixelBufferPool, colorSpace: colorSpace) {
                    let time = CMTimeMake(value: Int64(frame), timescale: fps)
                    adaptor.append(pixelBuffer, withPresentationTime: time)
                }
            }
        }

        writerInput.markAsFinished()
        await assetWriter.finishWriting()
        return outputURL
    }

    private static func pixelBufferFromImage(image: UIImage, size: CGSize, pool: CVPixelBufferPool?, colorSpace: CGColorSpace) -> CVPixelBuffer? {
        guard let cgImage = image.cgImage else { return nil }
        var pixelBufferOut: CVPixelBuffer?
        
        let status: CVReturn
        if let pool = pool {
            status = CVPixelBufferPoolCreatePixelBuffer(kCFAllocatorDefault, pool, &pixelBufferOut)
        } else {
            let options: [String: Any] = [
                kCVPixelBufferCGImageCompatibilityKey as String: true,
                kCVPixelBufferCGBitmapContextCompatibilityKey as String: true
            ]
            status = CVPixelBufferCreate(kCFAllocatorDefault, Int(size.width), Int(size.height), kCVPixelFormatType_32ARGB, options as CFDictionary, &pixelBufferOut)
        }

        guard status == kCVReturnSuccess, let pixelBuffer = pixelBufferOut else { return nil }

        CVPixelBufferLockBaseAddress(pixelBuffer, [])
        let data = CVPixelBufferGetBaseAddress(pixelBuffer)
        let context = CGContext(data: data, width: Int(size.width), height: Int(size.height), bitsPerComponent: 8, bytesPerRow: CVPixelBufferGetBytesPerRow(pixelBuffer), space: colorSpace, bitmapInfo: CGImageAlphaInfo.noneSkipFirst.rawValue)

        guard let context = context else {
            CVPixelBufferUnlockBaseAddress(pixelBuffer, [])
            return nil
        }
        context.draw(cgImage, in: CGRect(origin: .zero, size: size))
        CVPixelBufferUnlockBaseAddress(pixelBuffer, [])

        return pixelBuffer
    }
}
