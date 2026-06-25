//
//  MapEverythingApp.swift
//  MapEverything
//
//  Created by Alex Moran on 5/2/26.
//

import SwiftUI
import SwiftData
import UIKit

final class MapEverythingAppDelegate: NSObject, UIApplicationDelegate {
    func application(
        _ application: UIApplication,
        supportedInterfaceOrientationsFor window: UIWindow?
    ) -> UIInterfaceOrientationMask {
        .portrait
    }
}

@main
struct MapEverythingApp: App {
    @UIApplicationDelegateAdaptor(MapEverythingAppDelegate.self) private var appDelegate

    var sharedModelContainer: ModelContainer = {
        let schema = MapEverythingModelSchema.schema
        let modelConfiguration = ModelConfiguration(schema: schema, isStoredInMemoryOnly: false)

        do {
            return try ModelContainer(for: schema, configurations: [modelConfiguration])
        } catch {
            fatalError("Could not create ModelContainer: \(error)")
        }
    }()

    var body: some Scene {
        WindowGroup {
            ContentView()
        }
        .modelContainer(sharedModelContainer)
    }
}
