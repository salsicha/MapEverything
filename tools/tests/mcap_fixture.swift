// Compile with the production LocalMCAP.swift. The Python test validates this
// output with the independent official MCAP reader and the bag converter.
import Foundation
@main enum MCAPFixture {
    static func main() throws {
        let writer = try LocalMCAPWriter(url: URL(fileURLWithPath: CommandLine.arguments[1]))
        try writer.begin()
        try writer.channel(id: 1, topic: "/mapping/test", type: "std_msgs/msg/String")
        try writer.message(id: 1, channel: 1, timestamp: 100,
                           data: Data(#"{"op":"publish","topic":"/mapping/test","msg":{"data":"first"}}"#.utf8))
        try writer.commit()
        try writer.begin()
        try writer.message(id: 2, channel: 1, timestamp: 150, data: Data(#"{"msg":{"data":"rolled back"}}"#.utf8))
        try writer.rollback()
        try writer.begin()
        try writer.message(id: 2, channel: 1, timestamp: 200,
                           data: Data(#"{"op":"publish","topic":"/mapping/test","msg":{"data":"second"}}"#.utf8))
        try writer.commit()
    }
}
