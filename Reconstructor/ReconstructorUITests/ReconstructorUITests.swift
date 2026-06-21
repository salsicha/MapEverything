//
//  ReconstructorUITests.swift
//  ReconstructorUITests
//
//  Created by Alex Moran on 5/2/26.
//

import XCTest

final class ReconstructorUITests: XCTestCase {

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
    func testMainInterfaceElementsExist() throws {
        let app = XCUIApplication()
        app.launch()

        // Check for Heads Up Display Status (Initial State)
        XCTAssertTrue(app.staticTexts["Paused"].waitForExistence(timeout: 2))
        XCTAssertTrue(app.staticTexts["Points: 0"].waitForExistence(timeout: 2))
        
        // Test App Mode Segments
        let measureButton = app.segmentedControls.buttons["Measure"]
        XCTAssertTrue(measureButton.waitForExistence(timeout: 2))
        measureButton.tap()
        
        let remodelButton = app.segmentedControls.buttons["Remodel"]
        XCTAssertTrue(remodelButton.waitForExistence(timeout: 2))
        remodelButton.tap()
        
        let landscapeButton = app.segmentedControls.buttons["Landscape"]
        XCTAssertTrue(landscapeButton.waitForExistence(timeout: 2))
        landscapeButton.tap()
        
        // Test Visualization Mode Segments
        let wireframeButton = app.segmentedControls.buttons["Wireframe"]
        XCTAssertTrue(wireframeButton.waitForExistence(timeout: 2))
        wireframeButton.tap()
        
        let solidMeshButton = app.segmentedControls.buttons["Solid Mesh"]
        XCTAssertTrue(solidMeshButton.waitForExistence(timeout: 2))
        solidMeshButton.tap()
    }

    @MainActor
    func testLaunchPerformance() throws {
        // This measures how long it takes to launch your application.
        measure(metrics: [XCTApplicationLaunchMetric()]) {
            XCUIApplication().launch()
        }
    }
}
