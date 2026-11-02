#!/usr/bin/env python3
"""Measure concurrent ROS header age without deserializing point arrays."""
import argparse
import json
import struct
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

p = argparse.ArgumentParser()
p.add_argument('--seconds', type=float, default=30)
p.add_argument('--interval', type=float, default=0, help='Emit bounded windows during long runs')
p.add_argument('--trace', help='JSONL frame header trace; point arrays remain serialized')
a = p.parse_args()
trace = open(a.trace, 'w') if a.trace else None
rclpy.init()
n = rclpy.create_node('raw_header_latency_probe')
topics = {'/livox/lidar': 'livox_ros_driver2/msg/CustomMsg', '/livox/imu': 'sensor_msgs/msg/Imu', '/Odometry': 'nav_msgs/msg/Odometry'}
samples = {t: [] for t in topics}
subs = []
for topic, typ in topics.items():
    def cb(raw, topic=topic):
        # CDR encapsulation is 4 bytes; Header starts with int32 sec,uint32 nanosec.
        endian = '<' if raw[1] & 1 else '>'
        sec, ns = struct.unpack_from(endian + 'iI', raw, 4)
        stamp = sec + ns / 1e9
        steady = time.monotonic()
        age = n.get_clock().now().nanoseconds / 1e9 - stamp
        samples[topic].append((steady, age, stamp))
        if trace and topic != '/livox/imu':
            row = dict(topic=topic, stamp_ns=sec*1000000000+ns, steady_ns=time.monotonic_ns(), age_s=age)
            if topic == '/livox/lidar':
                length = struct.unpack_from(endian+'I', raw, 12)[0]
                pos = 16 + length
                pos = 4 + ((pos-4+7)//8)*8  # CDR alignment is relative to payload.
                base, points = struct.unpack_from(endian+'QI', raw, pos)
                count = struct.unpack_from(endian+'I', raw, pos+16)[0]
                row.update(timebase_ns=base, points=points, sequence_length=count)
                if count:
                    row['last_offset_ns'] = struct.unpack_from(endian+'I', raw, pos+20+(count-1)*20)[0]
            trace.write(json.dumps(row)+'\n')
    subs.append(n.create_subscription(get_message(typ), topic, cb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), raw=True))
def report(window):
    for topic, data in samples.items():
        ages = sorted(row[1] for row in data)
        print(json.dumps({'topic': topic, 'count': len(data), 'window_s': window,
            'stamp_span_s': data[-1][2]-data[0][2] if data else None,
            'stamp_backtracks': sum(b[2]<x[2] for x,b in zip(data,data[1:])),
            'over_0_5_s': sum(age>.5 for age in ages),
            'under_minus_0_1_s': sum(age<-.1 for age in ages),
            'hz': (len(data)-1)/(data[-1][0]-data[0][0]) if len(data)>1 else 0,
            'age_mean_s': sum(ages)/len(ages) if ages else None,
            'age_p95_s': ages[int((len(ages)-1)*.95)] if ages else None,
            'age_min_s': min(ages) if ages else None, 'age_max_s': max(ages) if ages else None}),flush=True)
        data.clear()
    print(json.dumps({'host_clock_offset_change_s': time.time()-time.monotonic()-wall_offset,
                      'elapsed_s': time.monotonic()-start}),flush=True)
    if trace: trace.flush()
start=time.monotonic()
window_start=start
wall_offset=time.time()-start
while time.monotonic()-start<a.seconds:
    rclpy.spin_once(n,timeout_sec=.1)
    if a.interval and time.monotonic()-window_start>=a.interval:
        report(time.monotonic()-window_start)
        window_start=time.monotonic()
report(time.monotonic()-window_start)
if trace: trace.close()
n.destroy_node()
rclpy.shutdown()
