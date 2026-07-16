//
//  MapEverythingUITests.swift
//  MapEverythingUITests
//
//  Created by Alex Moran on 5/2/26.
//

import XCTest

final class MapEverythingUITests: XCTestCase {

    override func setUpWithError() throws {
        // Put setup code here. This method is called before the invocation of each test method in the class.

        // In UI tests it is usually best to stop immediately when a failure occurs.
        continueAfterFailure = false

        // In UI tests it’s important to set the initial state - such as interface orientation - required for your tests before they run. The setUp method is a good place to do this.
    }

    override func tearDownWithError() throws {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
    }

    @MainActor
    func testAppLaunches() throws {
        let app = XCUIApplication()
        app.launch()

        XCTAssertTrue(app.wait(for: .runningForeground, timeout: 5))
    }

    @MainActor
    func testSessionHistoryOpensFromActionRail() throws {
        let app = XCUIApplication()
        app.launch()

        let springboard = XCUIApplication(bundleIdentifier: "com.apple.springboard")
        let allowCamera = springboard.buttons["Allow"].firstMatch
        let okCamera = springboard.buttons["OK"].firstMatch
        if allowCamera.waitForExistence(timeout: 3) {
            allowCamera.tap()
        } else if okCamera.exists {
            okCamera.tap()
        }

        // The simulator cannot run AR world tracking; clear the resulting alert.
        let arAlert = app.alerts["AR Session Error"]
        if arAlert.waitForExistence(timeout: 5) {
            arAlert.buttons["OK"].tap()
        }

        let historyButton = app.buttons["Session History"]
        XCTAssertTrue(historyButton.waitForExistence(timeout: 10))
        historyButton.tap()

        XCTAssertTrue(app.navigationBars["Session History"].waitForExistence(timeout: 5))
        let emptyState = app.staticTexts["No Mapping Sessions"]
        XCTAssertTrue(emptyState.exists || app.cells.count > 0)

        app.buttons["Done"].tap()
        XCTAssertTrue(historyButton.waitForExistence(timeout: 5))
    }

    @MainActor
    func testLaunchPerformance() throws {
        // This measures how long it takes to launch your application.
        measure(metrics: [XCTApplicationLaunchMetric()]) {
            XCUIApplication().launch()
        }
    }
}
