//
//  CBOREncoder.swift
//  MapEverything
//

import Foundation

/// Minimal RFC 8949 CBOR encoder for rosbridge payload dictionaries.
/// Supports the value types the message builders produce: dictionaries with
/// string keys, arrays, strings, integers, doubles, booleans, Data, and NSNull.
/// Returns nil for any unsupported value so callers can fall back to JSON.
nonisolated enum CBOREncoder {
    static func encode(_ payload: [String: Any]) -> Data? {
        var output = Data()
        guard appendValue(payload, to: &output) else { return nil }
        return output
    }

    private static func appendValue(_ value: Any, to output: inout Data) -> Bool {
        // CFBoolean bridges to every numeric type, so it must be tested before
        // the numeric casts.
        if let number = value as? NSNumber, CFGetTypeID(number) == CFBooleanGetTypeID() {
            output.append(number.boolValue ? 0xF5 : 0xF4)
            return true
        }

        switch value {
        case let dictionary as [String: Any]:
            appendHeader(major: 5, length: UInt64(dictionary.count), to: &output)
            // Deterministic key order keeps encodings reproducible for tests.
            for key in dictionary.keys.sorted() {
                guard appendString(key, to: &output),
                      let entry = dictionary[key],
                      appendValue(entry, to: &output) else { return false }
            }
            return true
        case let array as [Any]:
            appendHeader(major: 4, length: UInt64(array.count), to: &output)
            for element in array {
                guard appendValue(element, to: &output) else { return false }
            }
            return true
        case let string as String:
            return appendString(string, to: &output)
        case let data as Data:
            appendHeader(major: 2, length: UInt64(data.count), to: &output)
            output.append(data)
            return true
        case is NSNull:
            output.append(0xF6)
            return true
        case let number as NSNumber:
            let objCType = String(cString: number.objCType)
            if objCType == "f" || objCType == "d" {
                appendDouble(number.doubleValue, to: &output)
            } else {
                appendInteger(number.int64Value, to: &output)
            }
            return true
        default:
            return false
        }
    }

    private static func appendString(_ string: String, to output: inout Data) -> Bool {
        let utf8 = Data(string.utf8)
        appendHeader(major: 3, length: UInt64(utf8.count), to: &output)
        output.append(utf8)
        return true
    }

    private static func appendInteger(_ value: Int64, to output: inout Data) {
        if value >= 0 {
            appendHeader(major: 0, length: UInt64(value), to: &output)
        } else {
            appendHeader(major: 1, length: UInt64(-1 - value), to: &output)
        }
    }

    private static func appendDouble(_ value: Double, to output: inout Data) {
        output.append(0xFB)
        var bigEndian = value.bitPattern.bigEndian
        withUnsafeBytes(of: &bigEndian) { output.append(contentsOf: $0) }
    }

    private static func appendHeader(major: UInt8, length: UInt64, to output: inout Data) {
        let majorBits = major << 5
        switch length {
        case 0..<24:
            output.append(majorBits | UInt8(length))
        case 24...UInt64(UInt8.max):
            output.append(majorBits | 24)
            output.append(UInt8(length))
        case (UInt64(UInt8.max) + 1)...UInt64(UInt16.max):
            output.append(majorBits | 25)
            var value = UInt16(length).bigEndian
            withUnsafeBytes(of: &value) { output.append(contentsOf: $0) }
        case (UInt64(UInt16.max) + 1)...UInt64(UInt32.max):
            output.append(majorBits | 26)
            var value = UInt32(length).bigEndian
            withUnsafeBytes(of: &value) { output.append(contentsOf: $0) }
        default:
            output.append(majorBits | 27)
            var value = length.bigEndian
            withUnsafeBytes(of: &value) { output.append(contentsOf: $0) }
        }
    }
}
