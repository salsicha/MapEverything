//
//  GeoTileProviderConfiguration.swift
//  MapEverything
//

import Foundation

enum GeoTileOptionalProviderID: String, CaseIterable, Codable, Hashable {
    case copernicusDataSpace = "copernicus_data_space"
    case openTopography = "open_topography"
    case usgsEROS = "usgs_eros_earthdata"
    case commercialImagery = "commercial_imagery"
    case commercialTerrain = "commercial_terrain"

    var displayName: String {
        switch self {
        case .copernicusDataSpace:
            return "Copernicus Data Space"
        case .openTopography:
            return "OpenTopography"
        case .usgsEROS:
            return "USGS EROS / Earthdata"
        case .commercialImagery:
            return "Commercial Imagery"
        case .commercialTerrain:
            return "Commercial Terrain"
        }
    }

    var userDefaultsPrefix: String {
        "geoProvider.\(rawValue)"
    }

    var providerKind: GeoTileLayerKind {
        switch self {
        case .copernicusDataSpace, .commercialImagery:
            return .satelliteImagery
        case .openTopography, .usgsEROS, .commercialTerrain:
            return .dem
        }
    }

    var credentialRequirement: GeoTileCredentialRequirement {
        switch self {
        case .copernicusDataSpace, .usgsEROS:
            return .userLogin
        case .openTopography:
            return .userAPIKey
        case .commercialImagery, .commercialTerrain:
            return .commercialAccount
        }
    }

    var defaultEndpointURL: String {
        switch self {
        case .copernicusDataSpace:
            return "https://catalogue.dataspace.copernicus.eu/odata/v1"
        case .openTopography:
            return "https://portal.opentopography.org/API"
        case .usgsEROS:
            return "https://m2m.cr.usgs.gov/api/api/json/stable"
        case .commercialImagery, .commercialTerrain:
            return ""
        }
    }

    var credentialLabel: String {
        switch credentialRequirement {
        case .none:
            return "None"
        case .userAPIKey:
            return "API key"
        case .userLogin:
            return "Login or access token"
        case .commercialAccount:
            return "Commercial account"
        }
    }
}

struct GeoTileOptionalProviderConfiguration: Codable, Hashable, Identifiable {
    let id: GeoTileOptionalProviderID
    var isEnabled: Bool
    var endpointURL: String
    var credentialReference: String
    var hasCredentialMaterial: Bool
    var recordingAllowed: Bool
    var attributionOverride: String

    var isConfigured: Bool {
        isEnabled && hasCredentialMaterial && recordingAllowed
    }

    var statusLabel: String {
        if !isEnabled { return "disabled" }
        if !hasCredentialMaterial { return "missing_credentials" }
        if !recordingAllowed { return "recording_not_allowed" }
        return "configured"
    }

    var rosMessage: [String: Any] {
        [
            "id": id.rawValue,
            "display_name": id.displayName,
            "kind": id.providerKind.rawValue,
            "enabled": isEnabled,
            "configured": isConfigured,
            "status": statusLabel,
            "endpoint_url": endpointURL,
            "credential_requirement": id.credentialRequirement.rawValue,
            "credential_reference": credentialReference,
            "has_credential_material": hasCredentialMaterial,
            "recording_allowed": recordingAllowed,
            "attribution_override": attributionOverride
        ]
    }

    var diagnosticValues: [String: String] {
        [
            "\(id.rawValue).enabled": String(isEnabled),
            "\(id.rawValue).configured": String(isConfigured),
            "\(id.rawValue).status": statusLabel,
            "\(id.rawValue).kind": id.providerKind.rawValue,
            "\(id.rawValue).credential_requirement": id.credentialRequirement.rawValue,
            "\(id.rawValue).credential_reference": credentialReference,
            "\(id.rawValue).has_credential_material": String(hasCredentialMaterial),
            "\(id.rawValue).recording_allowed": String(recordingAllowed),
            "\(id.rawValue).endpoint_configured": String(!endpointURL.isEmpty),
            "\(id.rawValue).attribution_override_configured": String(!attributionOverride.isEmpty)
        ]
    }

    static func disabled(_ id: GeoTileOptionalProviderID) -> GeoTileOptionalProviderConfiguration {
        GeoTileOptionalProviderConfiguration(
            id: id,
            isEnabled: false,
            endpointURL: id.defaultEndpointURL,
            credentialReference: "",
            hasCredentialMaterial: false,
            recordingAllowed: false,
            attributionOverride: ""
        )
    }
}

enum GeoTileProviderConfigurationStore {
    static func load(from userDefaults: UserDefaults = .standard) -> [GeoTileOptionalProviderConfiguration] {
        GeoTileOptionalProviderID.allCases.map { providerID in
            GeoTileOptionalProviderConfiguration(
                id: providerID,
                isEnabled: userDefaults.bool(forKey: key("enabled", providerID)),
                endpointURL: userDefaults.string(forKey: key("endpointURL", providerID)) ?? providerID.defaultEndpointURL,
                credentialReference: userDefaults.string(forKey: key("credentialReference", providerID)) ?? "",
                hasCredentialMaterial: userDefaults.bool(forKey: key("hasCredentialMaterial", providerID)),
                recordingAllowed: userDefaults.bool(forKey: key("recordingAllowed", providerID)),
                attributionOverride: userDefaults.string(forKey: key("attributionOverride", providerID)) ?? ""
            )
        }
    }

    static var rosMessage: [[String: Any]] {
        load().map(\.rosMessage)
    }

    static var diagnosticValues: [String: String] {
        load().reduce(into: [:]) { values, configuration in
            configuration.diagnosticValues.forEach { key, value in
                values[key] = value
            }
        }
    }

    static func key(_ field: String, _ providerID: GeoTileOptionalProviderID) -> String {
        "\(providerID.userDefaultsPrefix).\(field)"
    }
}
