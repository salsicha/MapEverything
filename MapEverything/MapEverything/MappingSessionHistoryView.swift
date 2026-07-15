//
//  MappingSessionHistoryView.swift
//  MapEverything
//

import SwiftUI
import SwiftData

struct MappingSessionHistoryView: View {
    @Environment(\.dismiss) private var dismiss
    @Environment(\.modelContext) private var modelContext
    @Query(sort: \MappingSessionModel.lastUpdatedAt, order: .reverse)
    private var sessions: [MappingSessionModel]

    var body: some View {
        NavigationStack {
            List {
                if sessions.isEmpty {
                    ContentUnavailableView(
                        "No Mapping Sessions",
                        systemImage: "clock.arrow.circlepath",
                        description: Text("Past mapping sessions appear here.")
                    )
                } else {
                    ForEach(sessions) { session in
                        NavigationLink {
                            MappingSessionHistoryDetailView(session: session)
                        } label: {
                            sessionRow(session)
                        }
                    }
                    .onDelete(perform: deleteSessions)
                }
            }
            .navigationTitle("Session History")
            .toolbar {
                ToolbarItem(placement: .navigationBarLeading) {
                    Button("Done") {
                        dismiss()
                    }
                }
            }
        }
    }

    private func sessionRow(_ session: MappingSessionModel) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            HStack(spacing: 8) {
                Circle()
                    .fill(session.stateColor)
                    .frame(width: 8, height: 8)
                Text(session.startLabel)
                    .font(.headline)
                    .lineLimit(1)
            }
            HStack(spacing: 10) {
                Label(session.state, systemImage: "circle.dashed")
                Label(session.durationLabel, systemImage: "clock")
                Label("\(session.enabledStreams.count)", systemImage: "antenna.radiowaves.left.and.right")
            }
            .font(.caption)
            .foregroundColor(.secondary)

            if let lastError = session.lastError, !lastError.isEmpty {
                Text(lastError)
                    .font(.caption2)
                    .foregroundColor(.orange)
                    .lineLimit(1)
            }
        }
    }

    private func deleteSessions(at offsets: IndexSet) {
        for index in offsets {
            modelContext.delete(sessions[index])
        }
        do {
            try modelContext.save()
        } catch {
            print("MappingSessionHistoryView: failed to delete session record: \(error)")
        }
    }
}

struct MappingSessionHistoryDetailView: View {
    let session: MappingSessionModel

    var body: some View {
        List {
            Section("Session") {
                LabeledContent("State", value: session.state)
                if let startedAt = session.startedAt {
                    LabeledContent("Started") {
                        Text(startedAt, format: .dateTime)
                    }
                }
                if let endedAt = session.endedAt {
                    LabeledContent("Ended") {
                        Text(endedAt, format: .dateTime)
                    }
                }
                LabeledContent("Duration", value: session.durationLabel)
                if let sessionID = session.sessionID {
                    LabeledContent("ID", value: sessionID.uuidString)
                        .font(.caption)
                }
            }

            Section("Recorder") {
                LabeledContent("URL", value: session.recorderURL.isEmpty ? "—" : session.recorderURL)
                LabeledContent("Transport", value: session.bridgeTransport)
                if let sessionDirectoryPath = session.sessionDirectoryPath {
                    LabeledContent("Local Bag", value: (sessionDirectoryPath as NSString).lastPathComponent)
                }
            }

            Section("Streams") {
                if session.enabledStreams.isEmpty {
                    Text("No streams enabled")
                        .foregroundColor(.secondary)
                } else {
                    ForEach(session.enabledStreams, id: \.self) { stream in
                        Text(MappingSensorStream(rawValue: stream)?.displayName ?? stream)
                    }
                }
            }

            if let lastError = session.lastError, !lastError.isEmpty {
                Section("Last Error") {
                    Text(lastError)
                        .foregroundColor(.orange)
                }
            }
        }
        .navigationTitle(session.startLabel)
        .navigationBarTitleDisplayMode(.inline)
    }
}

private extension MappingSessionModel {
    var startLabel: String {
        guard let startedAt else { return "Unstarted session" }
        return startedAt.formatted(date: .abbreviated, time: .shortened)
    }

    var durationLabel: String {
        guard let startedAt else { return "—" }
        guard let endedAt else { return "In progress" }
        let duration = endedAt.timeIntervalSince(startedAt)
        guard duration >= 0 else { return "—" }
        return Duration.seconds(duration).formatted(.time(pattern: .hourMinuteSecond))
    }

    var stateColor: Color {
        switch state {
        case "Active": return .green
        case "Failed": return .red
        default: return .secondary
        }
    }
}
