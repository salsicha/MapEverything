//
//  BlueprintExporter.swift
//  MapEverything
//

import Foundation
import ARKit
import UIKit
import RoomPlan

class BlueprintExporter {
    /// Projects SafeARMesh models into a 2D top-down floor plan and saves it as a PDF.
    static func exportToPDF(safeMeshes: [SafeARMesh], filename: String) -> URL? {
        guard !safeMeshes.isEmpty else { return nil }
        
        var all2DPoints: [CGPoint] = []
        var paths: [CGPath] = []
        
        for mesh in safeMeshes {
            
            var transformedVertices: [simd_float3] = []
            transformedVertices.reserveCapacity(mesh.vertices.count)
            
            for vertex in mesh.vertices {
                let worldVertex = simd_mul(mesh.transform, simd_float4(vertex.x, vertex.y, vertex.z, 1.0))
                transformedVertices.append(simd_float3(worldVertex.x, worldVertex.y, worldVertex.z))
            }
            
            let path = CGMutablePath()
            
            for i in stride(from: 0, to: mesh.indices.count, by: 3) {
                let v1 = Int(mesh.indices[i])
                let v2 = Int(mesh.indices[i+1])
                let v3 = Int(mesh.indices[i+2])
                
                let p1 = transformedVertices[v1]
                let p2 = transformedVertices[v2]
                let p3 = transformedVertices[v3]
                
                // Map X and Z (horizontal/depth coordinates) to a 2D plane
                let cgP1 = CGPoint(x: CGFloat(p1.x), y: CGFloat(p1.z))
                let cgP2 = CGPoint(x: CGFloat(p2.x), y: CGFloat(p2.z))
                let cgP3 = CGPoint(x: CGFloat(p3.x), y: CGFloat(p3.z))
                
                path.move(to: cgP1)
                path.addLine(to: cgP2)
                path.addLine(to: cgP3)
                path.closeSubpath()
                
                all2DPoints.append(contentsOf: [cgP1, cgP2, cgP3])
            }
            paths.append(path)
        }
        
        guard !all2DPoints.isEmpty,
              let minX = all2DPoints.map({ $0.x }).min(),
              let maxX = all2DPoints.map({ $0.x }).max(),
              let minY = all2DPoints.map({ $0.y }).min(),
              let maxY = all2DPoints.map({ $0.y }).max() else { return nil }

        let width = maxX - minX
        let height = maxY - minY
        guard width > 0 && height > 0 else { return nil }

        // Standard letter size PDF: 8.5 x 11 inches (612 x 792 points)
        let pdfPageSize = CGSize(width: 612, height: 792)
        let margin: CGFloat = 40
        let availableWidth = pdfPageSize.width - margin * 2
        let availableHeight = pdfPageSize.height - margin * 2
        
        let scale = min(availableWidth / width, availableHeight / height)
        
        // Center the blueprint drawing onto the page
        let xOffset = margin + (availableWidth - width * scale) / 2 - minX * scale
        let yOffset = margin + (availableHeight - height * scale) / 2 - minY * scale
        
        guard let docDir = FileManager.default.cloudDocumentsURL else { return nil }
        let fileURL = docDir.appendingPathComponent("\(filename).pdf")
        
        UIGraphicsBeginPDFContextToFile(fileURL.path, CGRect(origin: .zero, size: pdfPageSize), nil)
        guard let context = UIGraphicsGetCurrentContext() else { return nil }
        UIGraphicsBeginPDFPage()
        
        context.setFillColor(UIColor.white.cgColor)
        context.fill(CGRect(origin: .zero, size: pdfPageSize))
        
        var transform = CGAffineTransform(translationX: xOffset, y: yOffset).scaledBy(x: scale, y: scale)
        
        // Apply Architectural Blueprint styling
        context.setStrokeColor(UIColor(red: 0.1, green: 0.3, blue: 0.8, alpha: 0.7).cgColor)
        context.setFillColor(UIColor(red: 0.2, green: 0.4, blue: 0.9, alpha: 0.1).cgColor)
        context.setLineWidth(0.5 / scale) // Keep lines consistently thin across scaling variations
        
        for path in paths {
            if let transformedPath = path.copy(using: &transform) {
                context.addPath(transformedPath)
                context.drawPath(using: .fillStroke)
            }
        }
        
        UIGraphicsEndPDFContext()
        
        return fileURL
    }
    
    /// Projects RoomPlan CapturedRoom data into a 2D top-down floor plan and saves it as a PDF.
    static func exportToPDF(capturedRoom: CapturedRoom, filename: String) -> URL? {
        var all2DPoints: [CGPoint] = []
        
        enum ElementType { case wall, door, window, object }
        struct ElementPath {
            let path: CGPath
            let type: ElementType
        }
        var elementPaths: [ElementPath] = []
        
        func process(dimensions: simd_float3, transform: simd_float4x4, type: ElementType) {
            let halfX = dimensions.x / 2
            let halfZ = dimensions.z / 2
            
            let corners = [
                simd_float4( halfX, 0,  halfZ, 1),
                simd_float4(-halfX, 0,  halfZ, 1),
                simd_float4(-halfX, 0, -halfZ, 1),
                simd_float4( halfX, 0, -halfZ, 1)
            ]
            
            let path = CGMutablePath()
            for (i, corner) in corners.enumerated() {
                let worldCorner = simd_mul(transform, corner)
                let cgPoint = CGPoint(x: CGFloat(worldCorner.x), y: CGFloat(worldCorner.z))
                all2DPoints.append(cgPoint)
                
                if i == 0 {
                    path.move(to: cgPoint)
                } else {
                    path.addLine(to: cgPoint)
                }
            }
            path.closeSubpath()
            elementPaths.append(ElementPath(path: path, type: type))
        }
        
        for wall in capturedRoom.walls { process(dimensions: wall.dimensions, transform: wall.transform, type: .wall) }
        for door in capturedRoom.doors { process(dimensions: door.dimensions, transform: door.transform, type: .door) }
        for window in capturedRoom.windows { process(dimensions: window.dimensions, transform: window.transform, type: .window) }
        for object in capturedRoom.objects { process(dimensions: object.dimensions, transform: object.transform, type: .object) }
        
        guard !all2DPoints.isEmpty,
              let minX = all2DPoints.map({ $0.x }).min(),
              let maxX = all2DPoints.map({ $0.x }).max(),
              let minY = all2DPoints.map({ $0.y }).min(),
              let maxY = all2DPoints.map({ $0.y }).max() else { return nil }

        let width = maxX - minX
        let height = maxY - minY
        guard width > 0 && height > 0 else { return nil }

        let pdfPageSize = CGSize(width: 612, height: 792)
        let margin: CGFloat = 40
        let availableWidth = pdfPageSize.width - margin * 2
        let availableHeight = pdfPageSize.height - margin * 2
        
        let scale = min(availableWidth / width, availableHeight / height)
        
        let xOffset = margin + (availableWidth - width * scale) / 2 - minX * scale
        let yOffset = margin + (availableHeight - height * scale) / 2 - minY * scale
        
        guard let docDir = FileManager.default.cloudDocumentsURL else { return nil }
        let fileURL = docDir.appendingPathComponent("\(filename).pdf")
        
        UIGraphicsBeginPDFContextToFile(fileURL.path, CGRect(origin: .zero, size: pdfPageSize), nil)
        guard let context = UIGraphicsGetCurrentContext() else { return nil }
        UIGraphicsBeginPDFPage()
        
        context.setFillColor(UIColor.white.cgColor)
        context.fill(CGRect(origin: .zero, size: pdfPageSize))
        
        var transform2D = CGAffineTransform(translationX: xOffset, y: yOffset).scaledBy(x: scale, y: scale)
        context.setLineWidth(1.0 / scale)
        
        // Render hierarchically: Objects bottom, then walls, with doors & windows stacked on top.
        let order: [ElementType] = [.object, .wall, .window, .door]
        for currentType in order {
            for element in elementPaths where element.type == currentType {
                if let transformedPath = element.path.copy(using: &transform2D) {
                    context.addPath(transformedPath)
                    
                    switch element.type {
                    case .wall:
                        context.setFillColor(UIColor(red: 0.1, green: 0.3, blue: 0.8, alpha: 0.8).cgColor)
                        context.setStrokeColor(UIColor(red: 0.1, green: 0.2, blue: 0.6, alpha: 1.0).cgColor)
                    case .door:
                        context.setFillColor(UIColor.white.cgColor)
                        context.setStrokeColor(UIColor(red: 0.1, green: 0.3, blue: 0.8, alpha: 1.0).cgColor)
                    case .window:
                        context.setFillColor(UIColor(red: 0.8, green: 0.9, blue: 1.0, alpha: 0.8).cgColor)
                        context.setStrokeColor(UIColor(red: 0.1, green: 0.3, blue: 0.8, alpha: 1.0).cgColor)
                    case .object:
                        context.setFillColor(UIColor(white: 0.9, alpha: 0.5).cgColor)
                        context.setStrokeColor(UIColor(white: 0.6, alpha: 1.0).cgColor)
                    }
                    
                    context.drawPath(using: .fillStroke)
                }
            }
        }
        
        UIGraphicsEndPDFContext()
        return fileURL
    }

    // MARK: - Dimensioned Floorplan

    struct WallSegment {
        let startPoint: CGPoint
        let endPoint: CGPoint
        let lengthMeters: Float
    }

    static func extractWallSegments(from capturedRoom: CapturedRoom) -> [WallSegment] {
        capturedRoom.walls.map { wall in
            let halfX = wall.dimensions.x / 2
            let startLocal = simd_float4(-halfX, 0, 0, 1)
            let endLocal = simd_float4(halfX, 0, 0, 1)
            let startWorld = simd_mul(wall.transform, startLocal)
            let endWorld = simd_mul(wall.transform, endLocal)
            return WallSegment(
                startPoint: CGPoint(x: CGFloat(startWorld.x), y: CGFloat(startWorld.z)),
                endPoint: CGPoint(x: CGFloat(endWorld.x), y: CGFloat(endWorld.z)),
                lengthMeters: wall.dimensions.x
            )
        }
    }

    static func extractWallSegments(from anchors: [ARAnchor], minLength: Float = 0.3) -> [WallSegment] {
        var walls: [WallSegment] = []
        for anchor in anchors {
            guard let plane = anchor as? ARPlaneAnchor,
                  plane.alignment == .vertical,
                  plane.planeExtent.width >= minLength else { continue }

            let center = plane.center
            let halfW = plane.planeExtent.width / 2
            let startLocal = simd_float4(center.x - halfW, center.y, center.z, 1)
            let endLocal = simd_float4(center.x + halfW, center.y, center.z, 1)
            let startWorld = simd_mul(anchor.transform, startLocal)
            let endWorld = simd_mul(anchor.transform, endLocal)

            walls.append(WallSegment(
                startPoint: CGPoint(x: CGFloat(startWorld.x), y: CGFloat(startWorld.z)),
                endPoint: CGPoint(x: CGFloat(endWorld.x), y: CGFloat(endWorld.z)),
                lengthMeters: plane.planeExtent.width
            ))
        }
        return walls
    }

    static func exportDimensionedFloorplan(walls: [WallSegment], filename: String, useImperialUnits: Bool) -> URL? {
        guard !walls.isEmpty else { return nil }

        var allPoints: [CGPoint] = []
        for wall in walls {
            allPoints.append(wall.startPoint)
            allPoints.append(wall.endPoint)
        }

        guard let minX = allPoints.map({ $0.x }).min(),
              let maxX = allPoints.map({ $0.x }).max(),
              let minY = allPoints.map({ $0.y }).min(),
              let maxY = allPoints.map({ $0.y }).max() else { return nil }

        let dataWidth = max(maxX - minX, 0.01)
        let dataHeight = max(maxY - minY, 0.01)

        let pdfPageSize = CGSize(width: 612, height: 792)
        let margin: CGFloat = 60
        let headerHeight: CGFloat = 50
        let availableWidth = pdfPageSize.width - margin * 2
        let availableHeight = pdfPageSize.height - margin * 2 - headerHeight

        let scale = min(availableWidth / dataWidth, availableHeight / dataHeight)
        let xOffset = margin + (availableWidth - dataWidth * scale) / 2 - minX * scale
        let yOffset = margin + headerHeight + (availableHeight - dataHeight * scale) / 2 - minY * scale

        guard let docDir = FileManager.default.cloudDocumentsURL else { return nil }
        let fileURL = docDir.appendingPathComponent("\(filename).pdf")

        UIGraphicsBeginPDFContextToFile(fileURL.path, CGRect(origin: .zero, size: pdfPageSize), nil)
        guard let context = UIGraphicsGetCurrentContext() else { return nil }
        UIGraphicsBeginPDFPage()

        context.setFillColor(UIColor.white.cgColor)
        context.fill(CGRect(origin: .zero, size: pdfPageSize))

        // Header
        let titleAttr: [NSAttributedString.Key: Any] = [
            .font: UIFont.systemFont(ofSize: 20, weight: .bold),
            .foregroundColor: UIColor.black
        ]
        ("Dimensioned Floor Plan" as NSString).draw(at: CGPoint(x: margin, y: 20), withAttributes: titleAttr)

        let unitLabel = useImperialUnits ? "Units: feet" : "Units: meters"
        let subtitleAttr: [NSAttributedString.Key: Any] = [
            .font: UIFont.systemFont(ofSize: 10, weight: .regular),
            .foregroundColor: UIColor.darkGray
        ]
        (unitLabel as NSString).draw(at: CGPoint(x: margin, y: 44), withAttributes: subtitleAttr)

        func toPage(_ pt: CGPoint) -> CGPoint {
            CGPoint(x: pt.x * scale + xOffset, y: pt.y * scale + yOffset)
        }

        // Draw walls as thick lines
        context.setStrokeColor(UIColor(red: 0.12, green: 0.12, blue: 0.18, alpha: 1.0).cgColor)
        context.setLineWidth(4.0)
        context.setLineCap(.round)
        context.setLineJoin(.round)

        for wall in walls {
            context.move(to: toPage(wall.startPoint))
            context.addLine(to: toPage(wall.endPoint))
        }
        context.strokePath()

        // Draw dimension annotations
        let dimTextAttr: [NSAttributedString.Key: Any] = [
            .font: UIFont.monospacedDigitSystemFont(ofSize: 8, weight: .semibold),
            .foregroundColor: UIColor(red: 0.85, green: 0.15, blue: 0.1, alpha: 1.0)
        ]
        let dimLineColor = UIColor(red: 0.85, green: 0.15, blue: 0.1, alpha: 0.6).cgColor
        let dimOffset: CGFloat = 14

        for wall in walls {
            let pStart = toPage(wall.startPoint)
            let pEnd = toPage(wall.endPoint)

            let dx = pEnd.x - pStart.x
            let dy = pEnd.y - pStart.y
            let len = hypot(dx, dy)
            guard len > 1 else { continue }

            // Normal perpendicular to wall (points outward)
            let nx = -dy / len
            let ny = dx / len

            // Dimension line endpoints, offset from wall
            let dStart = CGPoint(x: pStart.x + nx * dimOffset, y: pStart.y + ny * dimOffset)
            let dEnd = CGPoint(x: pEnd.x + nx * dimOffset, y: pEnd.y + ny * dimOffset)

            // Extension lines from wall to dimension line
            context.setStrokeColor(dimLineColor)
            context.setLineWidth(0.5)
            let extLen = dimOffset + 3
            for p in [pStart, pEnd] {
                context.move(to: CGPoint(x: p.x + nx * 4, y: p.y + ny * 4))
                context.addLine(to: CGPoint(x: p.x + nx * extLen, y: p.y + ny * extLen))
            }
            context.strokePath()

            // Dimension line with tick marks
            context.setLineWidth(0.75)
            context.move(to: dStart)
            context.addLine(to: dEnd)
            context.strokePath()

            let tickSize: CGFloat = 3
            let tdx = dx / len, tdy = dy / len
            for pt in [dStart, dEnd] {
                context.move(to: CGPoint(x: pt.x - tdx * tickSize - nx * tickSize, y: pt.y - tdy * tickSize - ny * tickSize))
                context.addLine(to: CGPoint(x: pt.x + tdx * tickSize + nx * tickSize, y: pt.y + tdy * tickSize + ny * tickSize))
            }
            context.strokePath()

            // Measurement label
            let value: Float = useImperialUnits ? wall.lengthMeters * 3.28084 : wall.lengthMeters
            let unit = useImperialUnits ? "ft" : "m"
            let label = String(format: "%.2f %@", value, unit) as NSString
            let textSize = label.size(withAttributes: dimTextAttr)

            let mid = CGPoint(x: (dStart.x + dEnd.x) / 2, y: (dStart.y + dEnd.y) / 2)
            let textOrigin = CGPoint(x: mid.x - textSize.width / 2, y: mid.y - textSize.height / 2)

            // White backing for readability
            let bgRect = CGRect(x: textOrigin.x - 3, y: textOrigin.y - 1, width: textSize.width + 6, height: textSize.height + 2)
            context.setFillColor(UIColor.white.cgColor)
            context.fill(bgRect)

            label.draw(at: textOrigin, withAttributes: dimTextAttr)
        }

        // Scale bar at bottom
        let scaleBarMeters: Float = useImperialUnits ? (1.0 / 3.28084) : 1.0
        let scaleBarLabel = useImperialUnits ? "1 ft" : "1 m"
        let scaleBarWidth = CGFloat(scaleBarMeters) * scale
        let barY = pdfPageSize.height - margin / 2
        let barX = pdfPageSize.width - margin - scaleBarWidth

        context.setStrokeColor(UIColor.black.cgColor)
        context.setLineWidth(1.5)
        context.move(to: CGPoint(x: barX, y: barY))
        context.addLine(to: CGPoint(x: barX + scaleBarWidth, y: barY))
        // End ticks
        for x in [barX, barX + scaleBarWidth] {
            context.move(to: CGPoint(x: x, y: barY - 4))
            context.addLine(to: CGPoint(x: x, y: barY + 4))
        }
        context.strokePath()

        let scaleAttr: [NSAttributedString.Key: Any] = [
            .font: UIFont.systemFont(ofSize: 8, weight: .medium),
            .foregroundColor: UIColor.black
        ]
        let scaleLabelSize = (scaleBarLabel as NSString).size(withAttributes: scaleAttr)
        (scaleBarLabel as NSString).draw(
            at: CGPoint(x: barX + (scaleBarWidth - scaleLabelSize.width) / 2, y: barY - scaleLabelSize.height - 5),
            withAttributes: scaleAttr
        )

        UIGraphicsEndPDFContext()
        return fileURL
    }
}