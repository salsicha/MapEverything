//
//  MapEverythingApp.swift
//  MapEverything
//
//  Created by Alex Moran on 5/2/26.
//

import SwiftUI
import SwiftData

@main
struct MapEverythingApp: App {
    var sharedModelContainer: ModelContainer = {
        let schema = Schema([
            EnvironmentModel.self
        ])
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
