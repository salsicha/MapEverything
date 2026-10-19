import Foundation

/// MCAP v0, uncompressed data-section records (no optional chunk indexes).
/// https://mcap.dev/spec. Each committed batch has a footer, so a shared file
/// is independently readable without a database journal or a clean app stop.
nonisolated final class LocalMCAPWriter {
    static let magic = Data([0x89, 0x4d, 0x43, 0x41, 0x50, 0x30, 0x0d, 0x0a])
    private let file: FileHandle
    private var dataEnd: UInt64 = 0
    private var transactionStart: UInt64?

    init(url: URL) throws {
        guard FileManager.default.createFile(atPath: url.path, contents: nil) else { throw MCAPError.invalidFile }
        file = try FileHandle(forWritingTo: url)
        try file.write(contentsOf: Self.magic)
        try record(0x01, mcapString("") + mcapString("MapEverything"))
        dataEnd = try file.offset()
        try seal()
    }

    deinit { try? file.close() }

    func begin() throws {
        guard transactionStart == nil else { throw MCAPError.invalidFile }
        transactionStart = dataEnd
        try file.truncate(atOffset: dataEnd)
        try file.seek(toOffset: dataEnd)
    }

    func commit() throws {
        let end = try file.offset()
        try seal()
        try file.synchronize()
        dataEnd = end
        transactionStart = nil
    }

    func rollback() throws {
        guard let start = transactionStart else { return }
        try file.truncate(atOffset: start)
        try file.seek(toOffset: start)
        try seal()
        try file.synchronize()
        dataEnd = start
        transactionStart = nil
    }

    func channel(id: Int64, topic: String, type: String) throws {
        guard let id = UInt16(exactly: id), id > 0 else { throw MCAPError.invalidFile }
        // The stored object is the unchanged rosbridge publish envelope.
        let schema = Data(#"{"$schema":"https://json-schema.org/draft/2020-12/schema","type":"object"}"#.utf8)
        try record(0x03, mcapLE(id) + mcapString(type) + mcapString("jsonschema") + mcapLE(UInt32(schema.count)) + schema)
        let metadata = mcapString("ros_type") + mcapString(type)
            + mcapString("serialization_format") + mcapString("rosbridge_json")
        try record(0x04, mcapLE(id) + mcapLE(id) + mcapString(topic) + mcapString("json")
                   + mcapLE(UInt32(metadata.count)) + metadata)
    }

    func message(id: Int64, channel: Int64, timestamp: Int64, data: Data) throws {
        guard let channel = UInt16(exactly: channel), let timestamp = UInt64(exactly: timestamp) else {
            throw MCAPError.invalidFile
        }
        // Separate writes avoid copying a potentially large image/cloud payload.
        let prefix = mcapLE(channel) + mcapLE(UInt32(truncatingIfNeeded: id)) + mcapLE(timestamp) + mcapLE(timestamp)
        try file.write(contentsOf: Data([0x05]) + mcapLE(UInt64(prefix.count + data.count)) + prefix)
        try file.write(contentsOf: data)
    }

    private func record(_ opcode: UInt8, _ data: Data) throws {
        try file.write(contentsOf: Data([opcode]) + mcapLE(UInt64(data.count)) + data)
    }

    private func seal() throws {
        try record(0x0f, mcapLE(UInt32(0))) // DataEnd, optional CRC omitted
        try record(0x02, mcapLE(UInt64(0)) + mcapLE(UInt64(0)) + mcapLE(UInt32(0)))
        try file.write(contentsOf: Self.magic)
    }
}

nonisolated enum MCAPError: Error { case invalidFile, unsupportedEncoding }

private nonisolated func mcapLE<T: FixedWidthInteger>(_ value: T) -> Data {
    var value = value.littleEndian
    return withUnsafeBytes(of: &value) { Data($0) }
}
private nonisolated func mcapString(_ value: String) -> Data {
    let bytes = Data(value.utf8)
    return mcapLE(UInt32(bytes.count)) + bytes
}

/// Streaming scanner for the app's own uncompressed files, including complete
/// records preceding an interrupted final batch. Large point-cloud payloads can
/// be skipped while computing previews. Native/compressed MCAP is handled by
/// the companion Python tools; this is not a general-purpose MCAP importer.
nonisolated enum LocalMCAPReader {
    struct Message {
        let topic: String
        let type: String
        let timestamp: Int64
        let data: Data
    }

    static func scan(_ url: URL, includePayload: (String) -> Bool = { _ in true },
                     visit: (Message) throws -> Void) throws {
        let file = try FileHandle(forReadingFrom: url)
        defer { try? file.close() }
        let size = try file.seekToEnd()
        try file.seek(toOffset: 0)
        guard try file.read(upToCount: 8) == LocalMCAPWriter.magic else { throw MCAPError.invalidFile }
        var schemas: [UInt16: String] = [:]
        var channels: [UInt16: (String, String)] = [:]
        while let header = try file.read(upToCount: 9), !header.isEmpty {
            // In-progress/truncated last record: retain the complete prefix.
            guard header.count == 9 else { return }
            var h = Cursor(data: header)
            let opcode: UInt8 = try h.integer()
            let length: UInt64 = try h.integer()
            let start = try file.offset()
            guard start <= size, length <= size - start else { return }
            if opcode == 0x0f || opcode == 0x02 { return }
            if opcode == 0x05 {
                guard length >= 22, let prefix = try file.read(upToCount: 22), prefix.count == 22 else {
                    throw MCAPError.invalidFile
                }
                var p = Cursor(data: prefix)
                let channel: UInt16 = try p.integer()
                let _: UInt32 = try p.integer()
                let time: UInt64 = try p.integer()
                guard let (topic, type) = channels[channel], let timestamp = Int64(exactly: time) else {
                    throw MCAPError.invalidFile
                }
                let payload: Data
                if includePayload(topic) {
                    guard length - 22 <= 512 * 1_048_576,
                          let bytes = try file.read(upToCount: Int(length - 22)), bytes.count == Int(length - 22) else {
                        throw MCAPError.invalidFile
                    }
                    payload = bytes
                } else { payload = Data() }
                try visit(Message(topic: topic, type: type, timestamp: timestamp, data: payload))
            } else if opcode == 0x03 || opcode == 0x04 {
                guard length <= 1_048_576, let data = try file.read(upToCount: Int(length)), data.count == Int(length) else {
                    throw MCAPError.invalidFile
                }
                var p = Cursor(data: data)
                let id: UInt16 = try p.integer()
                if opcode == 0x03 {
                    schemas[id] = try p.string()
                } else {
                    let schema: UInt16 = try p.integer()
                    let topic = try p.string(), encoding = try p.string()
                    guard encoding == "json", let type = schemas[schema] else { throw MCAPError.unsupportedEncoding }
                    channels[id] = (topic, type)
                }
            } else if opcode == 0x06 {
                throw MCAPError.unsupportedEncoding
            }
            try file.seek(toOffset: start + length)
        }
    }

    private struct Cursor {
        let data: Data
        var offset = 0
        mutating func integer<T: FixedWidthInteger>() throws -> T {
            let size = MemoryLayout<T>.size
            guard offset <= data.count, size <= data.count - offset else { throw MCAPError.invalidFile }
            let value = data.withUnsafeBytes { $0.loadUnaligned(fromByteOffset: offset, as: T.self) }
            offset += size
            return T(littleEndian: value)
        }
        mutating func string() throws -> String {
            let length: UInt32 = try integer()
            guard Int(length) <= data.count - offset,
                  let value = String(data: data.subdata(in: offset..<(offset + Int(length))), encoding: .utf8) else {
                throw MCAPError.invalidFile
            }
            offset += Int(length)
            return value
        }
    }
}
