#!/usr/bin/env python3
"""Exercise the real Gazebo joints and measure rear-axle ground-truth trajectories."""
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
import yaml
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

rclpy.init()
n = rclpy.create_node('golf_runtime_validation')
pub = n.create_publisher(AckermannDriveStamped, '/golf/drive', 1)
data = {'odom': None, 'joints': None}
n.create_subscription(Odometry, '/golf/odom', lambda m: data.update(odom=m), 20)
n.create_subscription(JointState, '/golf/joint_states', lambda m: data.update(joints=m), 20)
out = {'tests': {}, 'measurement': 'Gazebo physical link pose and joints, no pose teleportation'}
params=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/vehicle.yaml').read_text())
L=params['wheelbase']; track=params['front_track']


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def sample():
    m = data['odom']; j = data['joints']; p = m.pose.pose; q = p.orientation
    return dict(t=stamp(m), x=p.position.x, y=p.position.y, z=p.position.z,
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                vx=m.twist.twist.linear.x, vy=m.twist.twist.linear.y,
                wz=m.twist.twist.angular.z,
                joints=dict(zip(j.name, j.position)))


def run(v, a, seconds, send=True):
    start=stamp(data['odom']); end=time.monotonic()+seconds*4+10
    samples=[]; old=-1; next_pub=0
    while stamp(data['odom'])-start < seconds:
        if time.monotonic()>end:
            raise TimeoutError('Simulation clock stalled')
        if send and time.monotonic() >= next_pub:
            msg=AckermannDriveStamped(); msg.drive.speed=float(v); msg.drive.steering_angle=float(a)
            pub.publish(msg); next_pub=time.monotonic()+0.025
        rclpy.spin_once(n, timeout_sec=0.01)
        if stamp(data['odom']) != old:
            old=stamp(data['odom']); samples.append(sample())
    return samples


def tail(rows, duration=2):
    return [s for s in rows if s['t']>rows[-1]['t']-duration]


def stats(rows):
    return {k: float(np.mean([s[k] for s in rows])) for k in ('vx','vy','wz','z')}


def curve(v, a):
    run(v,a,5)
    rows=run(v,a,10)
    x=np.array([s['x'] for s in rows]); y=np.array([s['y'] for s in rows])
    x-=x.mean(); y-=y.mean()
    b=np.linalg.lstsq(np.column_stack((2*x,2*y,np.ones(len(x)))),x*x+y*y,rcond=None)[0]
    radius=float(np.sqrt(b[2]+b[0]**2+b[1]**2))
    expected=abs(L/math.tan(a)); st=stats(rows)
    angles={side: float(np.mean([s['joints'][f'front_{side}_steer_joint'] for s in rows])) for side in ('left','right')}
    k=math.tan(a)/L
    targets={'left':math.atan2(L*k,1-track*k/2), 'right':math.atan2(L*k,1+track*k/2)}
    error=max(abs(angles[s]-targets[s]) for s in targets)
    return dict(**st, fitted_radius_m=radius, expected_radius_m=expected,
                radius_error_fraction=abs(radius/expected-1), joint_angles=angles,
                expected_angles=targets, max_steer_error_rad=error,
                passed=bool(abs(radius/expected-1)<0.10 and error<0.04 and
                            st['vx']*v>0 and st['wz']*(v*a)>0))


try:
    deadline=time.monotonic()+60
    while any(v is None for v in data.values()):
        if time.monotonic()>deadline: raise TimeoutError('Vehicle did not publish odom/joints')
        rclpy.spin_once(n,timeout_sec=0.1)
    rows=run(0,0,3)
    st=stats(tail(rows)); out['tests']['settle']={**st,'passed':bool(abs(st['vx'])<0.02 and 0.26<st['z']<0.29)}
    rows=run(1,0,7); st=stats(tail(rows)); out['tests']['forward']={**st,'passed':bool(abs(st['vx']-1)<0.08 and abs(st['wz'])<0.015)}
    rows=run(0,0,3,send=False); st=stats(tail(rows,0.5)); out['tests']['watchdog_stop']={**st,'passed':bool(abs(st['vx'])<0.03)}
    rows=run(-1,0,7); st=stats(tail(rows)); out['tests']['reverse']={**st,'passed':bool(abs(st['vx']+1)<0.08 and abs(st['wz'])<0.015)}
    run(0,0,3)
    for name,v,a in [('left',1,0.35),('right',1,-0.35),('reverse_left',-1,0.35)]:
        out['tests'][name]=curve(v,a)
        run(0,0,3)
        print(name, json.dumps(out['tests'][name]), flush=True)
    rows=run(0,8,3); j=rows[-1]['joints']; out['tests']['steering_clamp']={
        'left_rad':j['front_left_steer_joint'],'right_rad':j['front_right_steer_joint'],
        'passed':bool(abs(j['front_left_steer_joint']-math.atan2(math.tan(params['max_steer']),1-track*math.tan(params['max_steer'])/(2*L)))<0.02 and abs(j['front_right_steer_joint']-math.atan2(math.tan(params['max_steer']),1+track*math.tan(params['max_steer'])/(2*L)))<0.02)}
    run(1,0,4)
    rows=run(float('nan'),0,3); st=stats(tail(rows,0.5)); out['tests']['invalid_command_stop']={**st,'passed':bool(abs(st['vx'])<0.03)}
    out['passed']=all(x['passed'] for x in out['tests'].values())
finally:
    msg=AckermannDriveStamped(); pub.publish(msg)
    Path(sys.argv[1]).write_text(json.dumps(out,indent=2))
    n.destroy_node(); rclpy.shutdown()
print(json.dumps(out,indent=2))
sys.exit(0 if out.get('passed',False) else 1)
