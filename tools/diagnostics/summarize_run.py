#!/usr/bin/env python3
"""Summarize measured windows and correlate sparse same-frame stage records."""
import argparse
import json
import re
from decimal import Decimal
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('directory');a=p.parse_args();root=Path(a.directory)
def stats(v):
    v=sorted(v)
    return dict(n=len(v),mean=sum(v)/len(v),p95=v[int((len(v)-1)*.95)],min=v[0],max=v[-1]) if v else dict(n=0)
def records(name):
    for line in (root/name).read_text().splitlines():
        try:yield json.loads(line)
        except ValueError:pass
out={}
for name in ['startup_180s.jsonl','continuous_30s.jsonl','final_30s.jsonl']:
    if not (root/name).exists():continue
    rows=list(records(name));out[name]={}
    for topic in ['/livox/lidar','/livox/imu','/Odometry']:
        r=[x for x in rows if x.get('topic')==topic and x['count']]
        count=sum(x['count'] for x in r)
        if count:out[name][topic]=dict(count=count,age_mean_s=sum(x['count']*x['age_mean_s'] for x in r)/count,
            age_max_s=max(x['age_max_s'] for x in r),p95_window_max_s=max(x['age_p95_s'] for x in r),
            over_0_5_s=sum(x['over_0_5_s'] for x in r),stamp_backtracks=sum(x['stamp_backtracks'] for x in r))
    out[name]['clock_offset_change_s']=stats([x['host_clock_offset_change_s'] for x in rows if 'host_clock_offset_change_s' in x])
def kv(line):return dict(re.findall(r'(\w+)=([^\s]+)',line))
frames={};pubs={};fast=[];clocks=[];imus=[]
if (root/'driver_final.log').exists():
    for line in (root/'driver_final.log').read_text().splitlines():
        d=kv(line)
        if line.startswith('frame_diag '):frames[int(d['base_ns'])]=d
        if line.startswith('publish_diag '):pubs[int(d['base_ns'])]=d
        if line.startswith('clock_diag '):clocks.append(d)
        if line.startswith('imu_publish_diag '):imus.append(d)
if (root/'fastlio_final.log').exists():
    for line in (root/'fastlio_final.log').read_text().splitlines():
        if 'latency ' in line:fast.append(kv(line))
out['clock_diag']={key:stats([float(d[key]) for d in clocks]) for key in ['age_ms','wall_step_ms','device_vs_steady_ms','backtrack_ms']}
out['imu_publish']={key:stats([float(d[key]) for d in imus]) for key in ['age_min_ms','age_max_ms','publish_max_ms','queue_max_ms'] if all(key in d for d in imus)}
out['fastlio_sparse']={key:stats([float(d[key]) for d in fast]) for key in ['lidar_in','imu_in','odom','pre','process','update','map','odom_pub','clouds','wait_ms','oldest_ms','undistort_ms','imu_gap']}
matched=[]
for d in fast:
    base=int(Decimal(d['beg'])*Decimal(10**9))
    keys=[k for k in frames if abs(k-base)<512]
    if not keys:continue
    k=keys[0]
    if k not in pubs:continue
    f=frames[k];pub=pubs[k]
    arrival=float(d['steady'])-float(d['wait_ms'])/1000
    matched.append(dict(base_ns=k,receive_to_assembled_ms=float(f['receive_to_assembled_ms']),
        assembled_to_publish_ms=(int(pub['steady_ns'])-int(f['assembled_steady_ns']))/1e6,
        publish_start_to_fast_callback_ms=(arrival-int(pub['steady_ns'])/1e9)*1000,
        scan_duration_ms=int(pub['last_offset_ns'])/1e6,
        wait_ms=float(d['wait_ms']),process_ms=float(d['process'])))
out['same_frame_stages']={key:stats([d[key] for d in matched]) for key in ['receive_to_assembled_ms','assembled_to_publish_ms','publish_start_to_fast_callback_ms','scan_duration_ms','wait_ms','process_ms']}
out['same_frame_matches']=len(matched)
if (root/'final_30s.jsonl').exists():
    rows=list(records('final_30s.jsonl'))
    out['final_stable_after_first_30s']={}
    for topic in ['/livox/lidar','/livox/imu','/Odometry']:
        r=[x for x in rows if x.get('topic')==topic][1:]
        r=[x for x in r if x['count']]
        count=sum(x['count'] for x in r)
        if count:out['final_stable_after_first_30s'][topic]=dict(count=count,
            age_mean_s=sum(x['count']*x['age_mean_s'] for x in r)/count,
            age_max_s=max(x['age_max_s'] for x in r),
            max_window_p95_s=max(x['age_p95_s'] for x in r),
            over_0_5_s=sum(x['over_0_5_s'] for x in r),
            stamp_backtracks=sum(x['stamp_backtracks'] for x in r))
print(json.dumps(out,indent=2))
