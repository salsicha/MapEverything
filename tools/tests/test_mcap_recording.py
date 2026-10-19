"""Run with `python -m unittest discover -s tools/tests`; requires mcap.

The Swift writer interoperability check runs on macOS when swiftc is available.
"""
import importlib.util
import json
from pathlib import Path
import shutil
import sqlite3
import subprocess
import sys
import tempfile
import unittest

from mcap.reader import make_reader
from mcap.writer import Writer

ROOT = Path(__file__).resolve().parents[2]


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'tools' / filename)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


converter = load('mcap_test_converter', 'mapeverything-local-bag-to-ros2.py')
recorder = load('mcap_test_recorder', 'run-rosbridge-recorder.py')


def write_mcap(path, encoding='json'):
    with path.open('wb') as f:
        writer = Writer(f)
        writer.start()
        schema = writer.register_schema(name='std_msgs/msg/String', encoding='jsonschema', data=b'{"type":"object"}')
        channel = writer.register_channel(topic='/mapping/test', message_encoding=encoding, schema_id=schema,
            metadata={'ros_type': 'std_msgs/msg/String', 'serialization_format': 'rosbridge_json'})
        for stamp in [200, 100]:
            writer.add_message(channel_id=channel, log_time=stamp, publish_time=stamp,
                data=json.dumps({'op': 'publish', 'topic': '/mapping/test', 'msg': {'data': str(stamp)}}).encode())
        writer.finish()


class MCAPTests(unittest.TestCase):
    def test_recorder_and_converter_default_to_mcap_with_sqlite_override(self):
        args = recorder.parse_args([])
        cmd = recorder.bag_record_command(args, ['/mapping/test'])
        self.assertEqual(cmd[cmd.index('--storage')+1], 'mcap')
        self.assertEqual(converter.parse_args(['input.mcap']).storage_id, 'mcap')
        self.assertEqual(recorder.parse_args(['--storage', 'sqlite3']).storage, 'sqlite3')
        self.assertEqual(converter.parse_args(['input.db3', '--storage-id', 'sqlite3']).storage_id, 'sqlite3')

    def test_compressed_mcap_input_and_timestamp_sorting(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'test.mcap'
            write_mcap(path)
            self.assertEqual(converter.discover_db_files([Path(directory), path]), [path])
            topics, count = converter.summarize([path], {})
            self.assertEqual(count, 2)
            self.assertEqual(topics['/mapping/test'].message_type, 'std_msgs/msg/String')
            records = list(converter.iter_records([path]))
            self.assertEqual([r.timestamp for r in records], [100, 200])
            self.assertEqual(converter.decode_rosbridge_payload(records[0]), ('/mapping/test', {'data': '100'}))
            reader = converter.MCAPChunkReader(path, 0)
            scratch = Path(reader.temporary.name)
            self.assertTrue(scratch.exists())
            self.assertEqual(reader.next_record().timestamp, 100)
            reader.close()
            self.assertFalse(scratch.exists())
            (Path(directory)/'metadata.yaml').write_text("relative_file_paths:\n  - 'test.mcap'\n")
            self.assertEqual(converter.discover_db_files([Path(directory)]), [path])

    def test_mixed_legacy_sqlite_and_mcap_inputs(self):
        with tempfile.TemporaryDirectory() as directory:
            mcap = Path(directory)/'new.mcap'; db = Path(directory)/'old.db3'
            write_mcap(mcap)
            with sqlite3.connect(db) as conn:
                conn.execute('CREATE TABLE topics(id INTEGER, name TEXT, type TEXT, offered_qos_profiles TEXT)')
                conn.execute('CREATE TABLE messages(id INTEGER, topic_id INTEGER, timestamp INTEGER, data BLOB)')
                conn.execute("INSERT INTO topics VALUES(1,'/mapping/test','std_msgs/msg/String','')")
                conn.execute("INSERT INTO messages VALUES(1,1,150,?)", (b'{"msg":{"data":"legacy"}}',))
            files = converter.discover_db_files([Path(directory)])
            self.assertEqual(converter.summarize(files, {})[1], 3)
            records = list(converter.iter_records(files))
            self.assertEqual([r.timestamp for r in records], [100, 150, 200])

    def test_native_cdr_is_not_misinterpreted_as_json(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'native.mcap'; write_mcap(path, encoding='cdr')
            with self.assertRaisesRegex(converter.BagConversionError, 'native ROS2 MCAP'):
                converter.summarize([path], {})

    @unittest.skipUnless(sys.platform == 'darwin' and shutil.which('swiftc'), 'Requires Swift on macOS')
    def test_production_swift_writer_with_official_mcap_reader(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); binary = root/'fixture'; output = root/'swift.mcap'
            subprocess.run(['swiftc', '-module-cache-path', str(root/'modules'),
                str(ROOT/'MapEverything/MapEverything/LocalMCAP.swift'),
                str(ROOT/'tools/tests/mcap_fixture.swift'), '-o', str(binary)], check=True, capture_output=True)
            subprocess.run([str(binary), str(output)], check=True)
            with output.open('rb') as f:
                reader = make_reader(f)
                self.assertEqual(reader.get_header().library, 'MapEverything')
                messages = list(reader.iter_messages())
            self.assertEqual([m.log_time for _, _, m in messages], [100, 200])
            self.assertEqual([json.loads(m.data)['msg']['data'] for _, _, m in messages], ['first', 'second'])
            self.assertTrue(all(s.encoding == 'jsonschema' and c.message_encoding == 'json' for s,c,m in messages))
            self.assertEqual(converter.summarize([output], {})[1], 2)
            self.assertEqual([r.timestamp for r in converter.iter_records([output])], [100, 200])


if __name__ == '__main__':
    unittest.main()
