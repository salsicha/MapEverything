#!/usr/bin/env python3
"""Convert MapEverything local rosbridge_json SQLite bags to native ROS 2 bags.

Run this inside a sourced ROS 2 workspace that can import rosbag2_py, rclpy,
and any message packages present in the bag, including reconstructor_msgs.
Dry-run inspection only needs the Python standard library.
"""

from __future__ import annotations

import argparse
import array
import base64
import heapq
import json
import shutil
import sqlite3
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


MAPEVERYTHING_FORMAT = "rosbridge_json"
NATIVE_FORMAT = "cdr"


@dataclass(frozen=True)
class TopicDefinition:
    name: str
    message_type: str
    offered_qos_profiles: str = ""


@dataclass(frozen=True)
class BagRecord:
    timestamp: int
    topic: str
    message_type: str
    data: bytes
    source: Path
    row_id: int


class BagConversionError(RuntimeError):
    pass


class ChunkReader:
    def __init__(self, path: Path, index: int) -> None:
        self.path = path
        self.index = index
        self.connection = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
        self.cursor = self.connection.execute(
            """
            SELECT messages.timestamp, messages.id, topics.name, topics.type, messages.data
            FROM messages
            JOIN topics ON topics.id = messages.topic_id
            ORDER BY messages.timestamp ASC, messages.id ASC
            """
        )

    def next_record(self) -> BagRecord | None:
        row = self.cursor.fetchone()
        if row is None:
            return None

        timestamp, row_id, topic, message_type, data = row
        return BagRecord(
            timestamp=int(timestamp),
            topic=str(topic),
            message_type=str(message_type),
            data=bytes(data),
            source=self.path,
            row_id=int(row_id),
        )

    def close(self) -> None:
        self.cursor.close()
        self.connection.close()


class MessageHydrator:
    def __init__(self, verbose: bool = False) -> None:
        try:
            from rosidl_runtime_py.utilities import get_message
        except ImportError as exc:
            raise BagConversionError(
                "Unable to import rosidl_runtime_py. Source your ROS 2 setup.bash "
                "and the workspace containing reconstructor_msgs before converting."
            ) from exc

        self.get_message = get_message
        self.verbose = verbose
        self.message_classes: dict[str, Any] = {}

    def build(self, message_type: str, value: dict[str, Any]) -> Any:
        message_class = self.message_class(message_type)
        message = message_class()
        self.populate(message, value)
        return message

    def message_class(self, message_type: str) -> Any:
        normalized = normalize_message_type(message_type)
        if normalized not in self.message_classes:
            self.message_classes[normalized] = self.get_message(normalized)
        return self.message_classes[normalized]

    def populate(self, message: Any, values: dict[str, Any]) -> None:
        fields = message.get_fields_and_field_types()
        for field_name, field_type in fields.items():
            if field_name not in values:
                continue
            coerced = self.coerce_value(field_type, values[field_name])
            setattr(message, field_name, coerced)

    def coerce_value(self, field_type: str, value: Any) -> Any:
        element_type = sequence_element_type(field_type)
        if element_type is not None:
            return self.coerce_sequence(element_type, value)

        nested_type = nested_message_type(field_type)
        if nested_type is not None:
            if not isinstance(value, dict):
                raise BagConversionError(
                    f"Expected object for nested field {field_type}, got {type(value).__name__}"
                )
            message = self.message_class(nested_type)()
            self.populate(message, value)
            return message

        return coerce_primitive(field_type, value)

    def coerce_sequence(self, element_type: str, value: Any) -> Any:
        if is_byte_type(element_type):
            return coerce_byte_sequence(value)

        nested_type = nested_message_type(element_type)
        if nested_type is not None:
            if not isinstance(value, list):
                raise BagConversionError(
                    f"Expected list for sequence<{element_type}>, got {type(value).__name__}"
                )
            return [self.coerce_value(element_type, item) for item in value]

        if not isinstance(value, list):
            raise BagConversionError(
                f"Expected list for sequence<{element_type}>, got {type(value).__name__}"
            )
        return [coerce_primitive(element_type, item) for item in value]


def normalize_message_type(message_type: str) -> str:
    parts = message_type.split("/")
    if len(parts) == 2:
        return f"{parts[0]}/msg/{parts[1]}"
    return message_type


def nested_message_type(field_type: str) -> str | None:
    clean = field_type.strip()
    if "/" not in clean:
        return None
    return normalize_message_type(clean)


def sequence_element_type(field_type: str) -> str | None:
    clean = field_type.strip()
    if clean.startswith("sequence<") and clean.endswith(">"):
        return clean.removeprefix("sequence<").removesuffix(">").split(",", 1)[0].strip()
    if clean.startswith("bounded_sequence<") and clean.endswith(">"):
        return clean.removeprefix("bounded_sequence<").removesuffix(">").split(",", 1)[0].strip()
    if "[" in clean and clean.endswith("]"):
        return clean.split("[", 1)[0].strip()
    return None


def is_byte_type(field_type: str) -> bool:
    return field_type.strip() in {"byte", "octet", "uint8", "char"}


def coerce_byte_sequence(value: Any) -> array.array:
    if isinstance(value, str):
        try:
            decoded = base64.b64decode(value, validate=True)
        except Exception as exc:
            raise BagConversionError("Expected base64 text for uint8[] field") from exc
        return array.array("B", decoded)

    if isinstance(value, (bytes, bytearray)):
        return array.array("B", value)

    if isinstance(value, list):
        return array.array("B", (int(item) for item in value))

    raise BagConversionError(f"Cannot convert {type(value).__name__} to uint8[]")


def coerce_primitive(field_type: str, value: Any) -> Any:
    clean = field_type.strip()
    if clean == "bool":
        if isinstance(value, str):
            return value.lower() in {"1", "true", "yes", "on"}
        return bool(value)
    if clean == "string":
        return "" if value is None else str(value)
    if clean in {"float", "double", "float32", "float64"}:
        return float(value)
    if clean in {
        "byte",
        "char",
        "octet",
        "int8",
        "uint8",
        "int16",
        "uint16",
        "int32",
        "uint32",
        "int64",
        "uint64",
    }:
        return int(value)
    return value


def parse_type_overrides(values: list[str]) -> dict[str, str]:
    overrides: dict[str, str] = {}
    for item in values:
        if "=" not in item:
            raise BagConversionError(f"Invalid --type override {item!r}; expected TOPIC=TYPE")
        topic, message_type = item.split("=", 1)
        overrides[topic.strip()] = message_type.strip()
    return overrides


def discover_db_files(inputs: Iterable[Path]) -> list[Path]:
    db_files: list[Path] = []
    for path in inputs:
        if path.is_dir():
            metadata_files = list(db_files_from_metadata(path))
            if metadata_files:
                db_files.extend(metadata_files)
            else:
                db_files.extend(sorted(path.glob("*.db3")))
        elif path.suffix == ".db3":
            db_files.append(path)
        else:
            raise BagConversionError(f"Input is not a bag directory or .db3 file: {path}")

    unique_files: list[Path] = []
    seen: set[Path] = set()
    for db_file in db_files:
        resolved = db_file.resolve()
        if resolved not in seen:
            unique_files.append(db_file)
            seen.add(resolved)

    if not unique_files:
        raise BagConversionError("No .db3 chunks found")
    return unique_files


def db_files_from_metadata(directory: Path) -> Iterable[Path]:
    metadata = directory / "metadata.yaml"
    if not metadata.exists():
        return []

    files: list[Path] = []
    for raw_line in metadata.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line.startswith("- ") or ".db3" not in line:
            continue
        value = line[2:].strip()
        if value.startswith("'") and value.endswith("'"):
            value = value[1:-1].replace("''", "'")
        files.append(directory / value)
    return [path for path in files if path.exists()]


def inspect_topics(db_files: Iterable[Path], type_overrides: dict[str, str]) -> dict[str, TopicDefinition]:
    topics: dict[str, TopicDefinition] = {}
    for db_file in db_files:
        with sqlite3.connect(f"file:{db_file}?mode=ro", uri=True) as connection:
            for name, message_type, qos in connection.execute(
                "SELECT name, type, offered_qos_profiles FROM topics ORDER BY name"
            ):
                final_type = type_overrides.get(str(name), str(message_type))
                topics[str(name)] = TopicDefinition(str(name), final_type, str(qos or ""))
    return topics


def iter_records(db_files: list[Path]) -> Iterable[BagRecord]:
    readers = [ChunkReader(path, index) for index, path in enumerate(db_files)]
    heap: list[tuple[int, int, int, BagRecord]] = []
    try:
        for reader in readers:
            record = reader.next_record()
            if record is not None:
                heapq.heappush(heap, (record.timestamp, reader.index, record.row_id, record))

        while heap:
            _, reader_index, _, record = heapq.heappop(heap)
            yield record
            next_record = readers[reader_index].next_record()
            if next_record is not None:
                heapq.heappush(
                    heap,
                    (next_record.timestamp, reader_index, next_record.row_id, next_record),
                )
    finally:
        for reader in readers:
            reader.close()


def decode_rosbridge_payload(record: BagRecord) -> tuple[str, dict[str, Any]]:
    try:
        payload = json.loads(record.data.decode("utf-8"))
    except Exception as exc:
        raise BagConversionError(
            f"{record.source}:{record.row_id} is not a UTF-8 JSON rosbridge payload"
        ) from exc

    if not isinstance(payload, dict):
        raise BagConversionError(f"{record.source}:{record.row_id} payload is not an object")
    if payload.get("op") not in {None, "publish"}:
        raise BagConversionError(
            f"{record.source}:{record.row_id} has unsupported rosbridge op {payload.get('op')!r}"
        )

    topic = str(payload.get("topic") or record.topic)
    message = payload.get("msg")
    if not isinstance(message, dict):
        raise BagConversionError(f"{record.source}:{record.row_id} payload has no object msg")
    return topic, message


def create_topic_metadata(name: str, message_type: str, qos: str) -> Any:
    from rosbag2_py import TopicMetadata

    kwargs = {
        "name": name,
        "type": message_type,
        "serialization_format": NATIVE_FORMAT,
        "offered_qos_profiles": qos,
    }
    try:
        return TopicMetadata(**kwargs)
    except TypeError:
        kwargs["id"] = 0
        return TopicMetadata(**kwargs)


def open_writer(output: Path, storage_id: str) -> Any:
    try:
        import rosbag2_py
    except ImportError as exc:
        raise BagConversionError(
            "Unable to import rosbag2_py. Source your ROS 2 setup.bash before converting."
        ) from exc

    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=str(output), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=NATIVE_FORMAT,
        output_serialization_format=NATIVE_FORMAT,
    )
    writer.open(storage_options, converter_options)
    return writer


def convert(
    db_files: list[Path],
    output: Path,
    storage_id: str,
    type_overrides: dict[str, str],
    topic_filter: set[str],
    skip_unknown: bool,
    force: bool,
    verbose: bool,
) -> int:
    try:
        from rclpy.serialization import serialize_message
    except ImportError as exc:
        raise BagConversionError(
            "Unable to import rclpy. Source your ROS 2 setup.bash before converting."
        ) from exc

    topics = inspect_topics(db_files, type_overrides)
    if topic_filter:
        topics = {name: topic for name, topic in topics.items() if name in topic_filter}

    hydrator = MessageHydrator(verbose=verbose)
    available_topics: dict[str, TopicDefinition] = {}
    for topic in sorted(topics.values(), key=lambda item: item.name):
        try:
            hydrator.message_class(topic.message_type)
            available_topics[topic.name] = topic
        except Exception as exc:
            if not skip_unknown:
                raise BagConversionError(
                    f"Cannot prepare topic {topic.name} ({topic.message_type}): {exc}"
                ) from exc
            print(
                f"Skipping unavailable topic {topic.name} ({topic.message_type}): {exc}",
                file=sys.stderr,
            )

    if not available_topics:
        raise BagConversionError("No convertible topics found")

    if output.exists():
        if not force:
            raise BagConversionError(f"Output already exists: {output}. Use --force to replace it.")
        shutil.rmtree(output)

    output.parent.mkdir(parents=True, exist_ok=True)
    writer = open_writer(output, storage_id)

    created_topics: set[str] = set()
    for topic in sorted(available_topics.values(), key=lambda item: item.name):
        writer.create_topic(
            create_topic_metadata(
                name=topic.name,
                message_type=topic.message_type,
                qos=topic.offered_qos_profiles,
            )
        )
        created_topics.add(topic.name)

    written = 0
    for record in iter_records(db_files):
        topic_name = record.topic
        if topic_filter and topic_name not in topic_filter:
            continue
        if topic_name not in created_topics:
            continue

        topic_name, message_dict = decode_rosbridge_payload(record)
        if topic_filter and topic_name not in topic_filter:
            continue
        topic = topics.get(topic_name)
        if topic is None or topic_name not in created_topics:
            continue

        try:
            message = hydrator.build(topic.message_type, message_dict)
            writer.write(topic_name, serialize_message(message), record.timestamp)
            written += 1
        except Exception as exc:
            if not skip_unknown:
                raise BagConversionError(
                    f"Failed to convert {topic_name} at {record.timestamp} "
                    f"from {record.source.name}:{record.row_id}: {exc}"
                ) from exc
            print(
                f"Skipping message on {topic_name} at {record.timestamp}: {exc}",
                file=sys.stderr,
            )

    del writer
    return written


def summarize(db_files: list[Path], type_overrides: dict[str, str]) -> tuple[dict[str, TopicDefinition], int]:
    topics = inspect_topics(db_files, type_overrides)
    message_count = 0
    for db_file in db_files:
        with sqlite3.connect(f"file:{db_file}?mode=ro", uri=True) as connection:
            message_count += int(connection.execute("SELECT COUNT(*) FROM messages").fetchone()[0])
    return topics, message_count


def default_output_path(inputs: list[Path]) -> Path:
    first = inputs[0]
    if first.is_dir():
        return first.with_name(f"{first.name}_native_ros2")
    return first.parent / f"{first.stem}_native_ros2"


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert MapEverything rosbridge_json SQLite chunks to a native ROS 2 bag."
    )
    parser.add_argument(
        "inputs",
        nargs="+",
        type=Path,
        help="Bag session directory or one or more .db3 chunk files.",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Native ROS 2 bag output directory. Defaults beside the input.",
    )
    parser.add_argument(
        "--storage-id",
        default="sqlite3",
        help="rosbag2 storage plugin for the output bag. Default: sqlite3.",
    )
    parser.add_argument(
        "--topic",
        action="append",
        default=[],
        help="Only convert this topic. May be provided multiple times.",
    )
    parser.add_argument(
        "--type",
        action="append",
        default=[],
        metavar="TOPIC=TYPE",
        help="Override a topic message type, for example /foo=example_msgs/msg/Foo.",
    )
    parser.add_argument(
        "--skip-unknown",
        action="store_true",
        help="Skip topics or messages that cannot be imported or hydrated.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Replace the output directory if it already exists.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Inspect chunks and print the planned conversion without ROS 2 imports.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print extra conversion detail.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    inputs = [path.expanduser() for path in args.inputs]
    output = args.output.expanduser() if args.output else default_output_path(inputs)

    try:
        type_overrides = parse_type_overrides(args.type)
        db_files = discover_db_files(inputs)
        topics, message_count = summarize(db_files, type_overrides)
        selected_topics = set(args.topic)

        print(f"Input chunks: {len(db_files)}")
        for db_file in db_files:
            print(f"  {db_file}")
        print(f"Messages: {message_count}")
        print("Topics:")
        for topic in sorted(topics.values(), key=lambda item: item.name):
            marker = "convert"
            if selected_topics and topic.name not in selected_topics:
                marker = "skip"
            print(f"  [{marker}] {topic.name} ({topic.message_type})")

        if args.dry_run:
            print(f"Dry run only. Native output would be: {output}")
            return 0

        written = convert(
            db_files=db_files,
            output=output,
            storage_id=args.storage_id,
            type_overrides=type_overrides,
            topic_filter=selected_topics,
            skip_unknown=args.skip_unknown,
            force=args.force,
            verbose=args.verbose,
        )
    except BagConversionError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    print(f"Wrote {written} native ROS 2 messages to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
