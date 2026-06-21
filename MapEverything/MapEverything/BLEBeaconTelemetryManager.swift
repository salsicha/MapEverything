//
//  BLEBeaconTelemetryManager.swift
//  MapEverything
//

import Combine
import CoreBluetooth
import Foundation

struct BLEBeaconTelemetrySample {
    let peripheralID: UUID
    let localName: String
    let serviceUUIDs: [String]
    let rssiDBM: Int
    let advertisementData: [String: Any]
    let timestamp: Date

    var rosMessage: [String: Any] {
        [
            "peripheral_id": peripheralID.uuidString,
            "local_name": localName,
            "service_uuids": serviceUUIDs,
            "rssi_dbm": rssiDBM,
            "advertisement_data": advertisementData,
            "timestamp": ISO8601DateFormatter().string(from: timestamp)
        ]
    }
}

final class BLEBeaconTelemetryManager: NSObject, ObservableObject, CBCentralManagerDelegate {
    struct Configuration {
        let serviceUUIDs: [CBUUID]
        let peripheralIDs: Set<UUID>
        let localNamePrefixes: [String]
        let allowDuplicateAdvertisements: Bool

        static let serviceUUIDsKey = "bleBeaconServiceUUIDs"
        static let peripheralIDsKey = "bleBeaconPeripheralIDs"
        static let localNamePrefixesKey = "bleBeaconLocalNamePrefixes"
        static let allowDuplicateAdvertisementsKey = "bleBeaconAllowDuplicateAdvertisements"

        static func load(from userDefaults: UserDefaults = .standard) -> Configuration {
            let serviceUUIDs = configuredStrings(forKey: serviceUUIDsKey, userDefaults: userDefaults)
                .map { CBUUID(string: $0) }
            let peripheralIDs = Set(
                configuredStrings(forKey: peripheralIDsKey, userDefaults: userDefaults)
                    .compactMap { UUID(uuidString: $0) }
            )
            let localNamePrefixes = configuredStrings(forKey: localNamePrefixesKey, userDefaults: userDefaults)
            let hasExplicitDuplicateSetting = userDefaults.object(forKey: allowDuplicateAdvertisementsKey) != nil
            let allowDuplicateAdvertisements = hasExplicitDuplicateSetting
                ? userDefaults.bool(forKey: allowDuplicateAdvertisementsKey)
                : true

            return Configuration(
                serviceUUIDs: serviceUUIDs,
                peripheralIDs: peripheralIDs,
                localNamePrefixes: localNamePrefixes,
                allowDuplicateAdvertisements: allowDuplicateAdvertisements
            )
        }

        var hasFilters: Bool {
            !serviceUUIDs.isEmpty || !peripheralIDs.isEmpty || !localNamePrefixes.isEmpty
        }

        var serviceUUIDStrings: [String] {
            serviceUUIDs.map(\.uuidString)
        }

        var peripheralIDStrings: [String] {
            peripheralIDs.map(\.uuidString).sorted()
        }

        private static func configuredStrings(forKey key: String, userDefaults: UserDefaults) -> [String] {
            var rawValues: [String] = []

            if let arrayValue = userDefaults.stringArray(forKey: key) {
                rawValues.append(contentsOf: arrayValue)
            }
            if let stringValue = userDefaults.string(forKey: key) {
                rawValues.append(contentsOf: splitConfiguredString(stringValue))
            }

            var seenValues = Set<String>()
            return rawValues
                .map { $0.trimmingCharacters(in: .whitespacesAndNewlines) }
                .filter { !$0.isEmpty }
                .filter { seenValues.insert($0).inserted }
        }

        private static func splitConfiguredString(_ value: String) -> [String] {
            value
                .split { character in
                    character == "," || character == ";" || character.isNewline
                }
                .map(String.init)
        }
    }

    static let shared = BLEBeaconTelemetryManager()

    @Published private(set) var isRunning = false
    @Published private(set) var authorizationStatusLabel = "unknown"
    @Published private(set) var centralStateLabel = "unknown"
    @Published private(set) var configuredServiceUUIDs: [String] = []
    @Published private(set) var configuredPeripheralIDs: [String] = []
    @Published private(set) var configuredLocalNamePrefixes: [String] = []
    @Published private(set) var allowDuplicateAdvertisements = true
    @Published private(set) var lastSamplesByPeripheral: [UUID: BLEBeaconTelemetrySample] = [:]
    @Published private(set) var lastScanStartedAt: Date?
    @Published private(set) var lastError: String?

    private let maxStoredSamples = 32
    private var centralManager: CBCentralManager?
    private var configuration: Configuration

    override init() {
        self.configuration = Configuration.load()
        super.init()

        refreshConfiguration()
        updateAuthorizationState()
    }

    var sessionMetadata: [String: Any] {
        var metadata: [String: Any] = [
            "source_api": "CoreBluetooth.CBCentralManager",
            "requires_bluetooth_permission": true,
            "authorization": authorizationStatusLabel,
            "central_state": centralStateLabel,
            "running": isRunning,
            "configured_service_uuids": configuredServiceUUIDs,
            "configured_peripheral_ids": configuredPeripheralIDs,
            "configured_local_name_prefixes": configuredLocalNamePrefixes,
            "allow_duplicate_advertisements": allowDuplicateAdvertisements,
            "last_scan_started_at": lastScanStartedAt.map { ISO8601DateFormatter().string(from: $0) } ?? "",
            "last_error": lastError ?? "",
            "limitations": [
                "scoped_to_configured_beacons_or_peripherals",
                "background_scanning_limited_by_ios_policy",
                "reports_ble_advertisement_rssi_not_wifi_or_cellular_power"
            ]
        ]

        metadata["last_samples"] = recentSamples.map(\.rosMessage)

        return metadata
    }

    var diagnosticLevel: Int {
        if !isRunning { return 1 }
        if authorizationStatusLabel == "denied" || authorizationStatusLabel == "restricted" { return 2 }
        if centralStateLabel == "unsupported" || centralStateLabel == "unauthorized" { return 2 }
        if !configuration.hasFilters { return 1 }
        if centralStateLabel == "powered_off" { return 1 }
        if lastSamplesByPeripheral.isEmpty { return 1 }
        return 0
    }

    var diagnosticMessage: String {
        if !isRunning {
            return "BLE beacon telemetry is not running"
        }
        if authorizationStatusLabel == "denied" || authorizationStatusLabel == "restricted" {
            return "Bluetooth permission is required for BLE beacon RSSI"
        }
        if centralStateLabel == "unsupported" {
            return "Bluetooth LE scanning is unsupported on this device"
        }
        if centralStateLabel == "unauthorized" {
            return "Bluetooth authorization is unavailable for BLE beacon RSSI"
        }
        if !configuration.hasFilters {
            return "Configure a BLE service UUID, peripheral UUID, or local-name prefix before scanning"
        }
        if centralStateLabel == "powered_off" {
            return "Bluetooth is powered off"
        }
        if let latestSample = recentSamples.first {
            return "BLE beacon RSSI available: \(latestSample.rssiDBM) dBm"
        }
        return lastError ?? "Waiting for configured BLE beacon advertisements"
    }

    var diagnosticValues: [String: String] {
        var values: [String: String] = [
            "running": String(isRunning),
            "source_api": "CoreBluetooth.CBCentralManager",
            "authorization": authorizationStatusLabel,
            "central_state": centralStateLabel,
            "configured_service_uuids": configuredServiceUUIDs.joined(separator: ","),
            "configured_peripheral_ids": configuredPeripheralIDs.joined(separator: ","),
            "configured_local_name_prefixes": configuredLocalNamePrefixes.joined(separator: ","),
            "allow_duplicate_advertisements": String(allowDuplicateAdvertisements),
            "known_peripherals": String(lastSamplesByPeripheral.count),
            "last_scan_started_at": lastScanStartedAt.map { ISO8601DateFormatter().string(from: $0) } ?? "",
            "last_error": lastError ?? "",
            "limitations": "scoped_to_configured_beacons_or_peripherals,background_scanning_limited_by_ios_policy"
        ]

        if let latestSample = recentSamples.first {
            values["peripheral_id"] = latestSample.peripheralID.uuidString
            values["local_name"] = latestSample.localName
            values["service_uuids"] = latestSample.serviceUUIDs.joined(separator: ",")
            values["rssi_dbm"] = String(latestSample.rssiDBM)
            values["sample_timestamp"] = ISO8601DateFormatter().string(from: latestSample.timestamp)
        }

        return values
    }

    func start() {
        refreshConfiguration()
        updateAuthorizationState()

        guard !isRunning else {
            startScanIfReady()
            return
        }

        isRunning = true
        lastError = nil

        if centralManager == nil {
            centralManager = CBCentralManager(delegate: self, queue: nil)
        } else {
            startScanIfReady()
        }
    }

    func stop() {
        isRunning = false
        centralManager?.stopScan()
    }

    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        centralStateLabel = label(for: central.state)
        updateAuthorizationState()

        guard isRunning else { return }
        startScanIfReady()
    }

    func centralManager(
        _ central: CBCentralManager,
        didDiscover peripheral: CBPeripheral,
        advertisementData: [String: Any],
        rssi RSSI: NSNumber
    ) {
        guard isRunning, isConfiguredTarget(peripheral: peripheral, advertisementData: advertisementData) else {
            return
        }

        let timestamp = Date()
        let sample = BLEBeaconTelemetrySample(
            peripheralID: peripheral.identifier,
            localName: localName(for: peripheral, advertisementData: advertisementData),
            serviceUUIDs: advertisedServiceUUIDStrings(from: advertisementData),
            rssiDBM: RSSI.intValue,
            advertisementData: advertisementSummary(from: advertisementData),
            timestamp: timestamp
        )

        lastSamplesByPeripheral[peripheral.identifier] = sample
        trimStoredSamplesIfNeeded()
        lastError = nil
    }

    private var recentSamples: [BLEBeaconTelemetrySample] {
        lastSamplesByPeripheral.values.sorted { $0.timestamp > $1.timestamp }
    }

    private func refreshConfiguration() {
        configuration = Configuration.load()
        configuredServiceUUIDs = configuration.serviceUUIDStrings
        configuredPeripheralIDs = configuration.peripheralIDStrings
        configuredLocalNamePrefixes = configuration.localNamePrefixes
        allowDuplicateAdvertisements = configuration.allowDuplicateAdvertisements
    }

    private func startScanIfReady() {
        guard isRunning else { return }

        refreshConfiguration()

        guard configuration.hasFilters else {
            centralManager?.stopScan()
            lastError = "BLE scanning is disabled until beacon filters are configured."
            return
        }

        guard let centralManager else {
            lastError = "BLE central manager is not available."
            return
        }

        centralStateLabel = label(for: centralManager.state)
        updateAuthorizationState()

        guard centralManager.state == .poweredOn else {
            centralManager.stopScan()
            lastError = "BLE central manager state is \(centralStateLabel)."
            return
        }

        let serviceFilter = configuration.serviceUUIDs.isEmpty ? nil : configuration.serviceUUIDs
        centralManager.scanForPeripherals(
            withServices: serviceFilter,
            options: [
                CBCentralManagerScanOptionAllowDuplicatesKey: configuration.allowDuplicateAdvertisements
            ]
        )
        lastScanStartedAt = Date()
        lastError = nil
    }

    private func isConfiguredTarget(peripheral: CBPeripheral, advertisementData: [String: Any]) -> Bool {
        if configuration.peripheralIDs.contains(peripheral.identifier) {
            return true
        }

        let advertisedUUIDs = Set(advertisedServiceUUIDStrings(from: advertisementData))
        let configuredUUIDs = Set(configuration.serviceUUIDStrings)
        if !configuredUUIDs.isDisjoint(with: advertisedUUIDs) {
            return true
        }

        let candidateName = localName(for: peripheral, advertisementData: advertisementData)
        guard !candidateName.isEmpty else { return false }

        let lowercasedName = candidateName.lowercased()
        return configuration.localNamePrefixes.contains { prefix in
            lowercasedName.hasPrefix(prefix.lowercased())
        }
    }

    private func localName(for peripheral: CBPeripheral, advertisementData: [String: Any]) -> String {
        if let advertisementName = advertisementData[CBAdvertisementDataLocalNameKey] as? String,
           !advertisementName.isEmpty {
            return advertisementName
        }
        return peripheral.name ?? ""
    }

    private func advertisedServiceUUIDStrings(from advertisementData: [String: Any]) -> [String] {
        var serviceUUIDs: [CBUUID] = []

        if let primaryServiceUUIDs = advertisementData[CBAdvertisementDataServiceUUIDsKey] as? [CBUUID] {
            serviceUUIDs.append(contentsOf: primaryServiceUUIDs)
        }
        if let overflowServiceUUIDs = advertisementData[CBAdvertisementDataOverflowServiceUUIDsKey] as? [CBUUID] {
            serviceUUIDs.append(contentsOf: overflowServiceUUIDs)
        }
        if let solicitedServiceUUIDs = advertisementData[CBAdvertisementDataSolicitedServiceUUIDsKey] as? [CBUUID] {
            serviceUUIDs.append(contentsOf: solicitedServiceUUIDs)
        }
        if let serviceData = advertisementData[CBAdvertisementDataServiceDataKey] as? [CBUUID: Data] {
            serviceUUIDs.append(contentsOf: serviceData.keys)
        }

        var seenUUIDs = Set<String>()
        return serviceUUIDs
            .map(\.uuidString)
            .filter { seenUUIDs.insert($0).inserted }
    }

    private func advertisementSummary(from advertisementData: [String: Any]) -> [String: Any] {
        var summary: [String: Any] = [:]

        if let localName = advertisementData[CBAdvertisementDataLocalNameKey] as? String {
            summary["local_name"] = localName
        }
        if let txPowerLevel = advertisementData[CBAdvertisementDataTxPowerLevelKey] as? NSNumber {
            summary["tx_power_level_dbm"] = txPowerLevel.intValue
        }
        if let isConnectable = advertisementData[CBAdvertisementDataIsConnectable] as? NSNumber {
            summary["is_connectable"] = isConnectable.boolValue
        }
        if let manufacturerData = advertisementData[CBAdvertisementDataManufacturerDataKey] as? Data {
            summary["manufacturer_data_bytes"] = manufacturerData.count
        }
        if let serviceData = advertisementData[CBAdvertisementDataServiceDataKey] as? [CBUUID: Data] {
            summary["service_data_uuids"] = serviceData.keys.map(\.uuidString).sorted()
            let serviceDataBytes: [[String: Any]] = serviceData
                .map { ["uuid": $0.key.uuidString, "bytes": $0.value.count] as [String: Any] }
                .sorted {
                    ($0["uuid"] as? String ?? "") < ($1["uuid"] as? String ?? "")
                }
            summary["service_data_bytes"] = serviceDataBytes
        }
        if let overflowServiceUUIDs = advertisementData[CBAdvertisementDataOverflowServiceUUIDsKey] as? [CBUUID] {
            summary["overflow_service_uuids"] = overflowServiceUUIDs.map(\.uuidString)
        }
        if let solicitedServiceUUIDs = advertisementData[CBAdvertisementDataSolicitedServiceUUIDsKey] as? [CBUUID] {
            summary["solicited_service_uuids"] = solicitedServiceUUIDs.map(\.uuidString)
        }

        return summary
    }

    private func trimStoredSamplesIfNeeded() {
        let overflowCount = lastSamplesByPeripheral.count - maxStoredSamples
        guard overflowCount > 0 else { return }

        let oldestPeripheralIDs = lastSamplesByPeripheral
            .sorted { $0.value.timestamp < $1.value.timestamp }
            .prefix(overflowCount)
            .map(\.key)
        oldestPeripheralIDs.forEach {
            lastSamplesByPeripheral.removeValue(forKey: $0)
        }
    }

    private func updateAuthorizationState() {
        authorizationStatusLabel = label(for: CBCentralManager.authorization)
    }

    private func label(for authorization: CBManagerAuthorization) -> String {
        switch authorization {
        case .notDetermined:
            return "not_determined"
        case .restricted:
            return "restricted"
        case .denied:
            return "denied"
        case .allowedAlways:
            return "allowed_always"
        @unknown default:
            return "unknown"
        }
    }

    private func label(for state: CBManagerState) -> String {
        switch state {
        case .unknown:
            return "unknown"
        case .resetting:
            return "resetting"
        case .unsupported:
            return "unsupported"
        case .unauthorized:
            return "unauthorized"
        case .poweredOff:
            return "powered_off"
        case .poweredOn:
            return "powered_on"
        @unknown default:
            return "unknown"
        }
    }
}
