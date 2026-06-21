//
//  EnvironmentModel.swift
//  Reconstructor
//
//

import Foundation
import SwiftData

@Model
final class EnvironmentModel {
    @Attribute(.unique) var id: UUID
    var name: String
    var creationDate: Date
    var filePathToPointCloudData: String?
    var arWorldMapPath: String?
    var meshPath: String?
    var objPath: String?
    var blueprintPath: String?
    var videoPath: String?
    var thumbnailPath: String?
    
    init(id: UUID = UUID(), name: String, creationDate: Date = Date(), filePathToPointCloudData: String? = nil, arWorldMapPath: String? = nil, meshPath: String? = nil, objPath: String? = nil, blueprintPath: String? = nil, videoPath: String? = nil, thumbnailPath: String? = nil) {
        self.id = id
        self.name = name
        self.creationDate = creationDate
        self.filePathToPointCloudData = filePathToPointCloudData
        self.arWorldMapPath = arWorldMapPath
        self.meshPath = meshPath
        self.objPath = objPath
        self.blueprintPath = blueprintPath
        self.videoPath = videoPath
        self.thumbnailPath = thumbnailPath
    }
}