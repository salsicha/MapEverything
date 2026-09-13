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

/// Keeps the process alive while the recorder's final flush drains after the
/// app enters the background. Ends its assertion exactly once, from either
/// the flush completing or iOS expiring the allowance.
@MainActor
private final class BackgroundFlushToken {
    private var identifier: UIBackgroundTaskIdentifier = .invalid

    func begin() {
        identifier = UIApplication.shared.beginBackgroundTask(withName: "FlushLocalROS2Bag") { [weak self] in
            self?.end()
        }
    }

    func end() {
        guard identifier != .invalid else { return }
        UIApplication.shared.endBackgroundTask(identifier)
        identifier = .invalid
    }
}

@main
struct MapEverythingApp: App {
    @UIApplicationDelegateAdaptor(MapEverythingAppDelegate.self) private var appDelegate
    @Environment(\.scenePhase) private var scenePhase

    /// True when this process hosts unit tests. The full AR UI must not run
    /// then: the live camera, RealityKit rendering, and CoreML warm-up
    /// saturate the main queue on a physical device and contend with the
    /// sensors, starving main-queue work the tests schedule. UI tests drive
    /// the app in a separate process, where this stays false and the real
    /// interface runs.
    static var isHostingUnitTests: Bool {
        NSClassFromString("XCTestCase") != nil
            || ProcessInfo.processInfo.environment["XCTestConfigurationFilePath"] != nil
            || ProcessInfo.processInfo.environment["XCTestBundlePath"] != nil
    }

    init() {
        MetricKitDiagnostics.shared.start()
    }

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
            if Self.isHostingUnitTests {
                Text("Running unit tests")
            } else {
                ContentView()
            }
        }
        .modelContainer(sharedModelContainer)
        .onChange(of: scenePhase) { _, phase in
            // A background termination discards the in-memory write batch;
            // flush it while iOS still allows work — but never on the main
            // thread: synchronously waiting on a busy writer queue during the
            // background transition trips the 10-second scene-update watchdog
            // (0x8BADF00D, MapEverything-2026-09-11-171357.ips). A background
            // task assertion buys the flush its own time allowance instead.
            if phase == .background {
                let token = BackgroundFlushToken()
                token.begin()
                DispatchQueue.global(qos: .userInitiated).async {
                    LocalROS2BagRecorder.shared.flushAndWait()
                    Task { @MainActor in token.end() }
                }
            }
        }
    }
}
