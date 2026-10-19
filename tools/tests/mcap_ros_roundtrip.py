#!/usr/bin/env python3
"""Optional ROS2 integration test, in a sourced workspace with rosbridge + MCAP.

python tools/tests/mcap_ros_roundtrip.py --swift-fixture /tmp/swift.mcap --output /tmp/mcap-ros-test
Uses only synthetic std_msgs/String messages on a loopback-only rosbridge.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def read_native(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    messages=[]
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic in ('/mcap_test', '/mapping/test'):
            messages.append((topic, deserialize_message(data, String).data, timestamp))
    return messages


def main():
    p=argparse.ArgumentParser();p.add_argument('--swift-fixture',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    tools=Path(__file__).resolve().parents[1]
    subprocess.run([sys.executable,str(tools/'mapeverything-local-bag-to-ros2.py'),str(args.swift_fixture),
                    '--output',str(args.output/'converted')],check=True)
    converted=read_native(args.output/'converted')
    assert [(m[1],m[2]) for m in converted]==[('first',100),('second',200)],converted
    assert list((args.output/'converted').glob('*.mcap'))
    assert not list((args.output/'converted').glob('*.db3'))

    import rclpy
    from std_msgs.msg import String
    rclpy.init();node=rclpy.create_node('mcap_recording_test');pub=node.create_publisher(String,'/mcap_test',10)
    log=(args.output/'recorder.log').open('w')
    process=subprocess.Popen([sys.executable,str(tools/'run-rosbridge-recorder.py'),
        '--output',str(args.output/'recorded'),'--record-all','--address','127.0.0.1',
        '--port','19099','--bridge-startup-delay','1'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        deadline=time.monotonic()+40
        while pub.get_subscription_count()==0 and time.monotonic()<deadline:
            assert process.poll() is None,'Recorder exited before subscription'
            rclpy.spin_once(node,timeout_sec=.1)
        assert pub.get_subscription_count()>0,'No recorder subscription'
        for i in range(5):
            pub.publish(String(data=f'mcap-{i}'))
            rclpy.spin_once(node,timeout_sec=.2)
        time.sleep(1)
        process.send_signal(signal.SIGINT)
        assert process.wait(timeout=30)==0
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=20)
            except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGTERM)
        log.close();node.destroy_node();rclpy.shutdown()
    recorded=read_native(args.output/'recorded')
    assert [m[1] for m in recorded]==[f'mcap-{i}' for i in range(5)],recorded
    assert list((args.output/'recorded').glob('*.mcap'))
    assert not list((args.output/'recorded').glob('*.db3'))
    assert 'storage_identifier: mcap' in (args.output/'recorded'/'metadata.yaml').read_text()
    print('PASS: Swift JSON MCAP -> native ROS2 MCAP (2 messages); Python recorder default -> native MCAP (5 live messages)')

if __name__=='__main__': main()
