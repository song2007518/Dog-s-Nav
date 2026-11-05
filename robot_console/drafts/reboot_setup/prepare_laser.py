#!/usr/bin/env python3
"""Runs on NX: wake only the perception scan pipeline; no movement or mapping."""
import json
import subprocess

CODE = '''import json, time, rclpy
from robots_dog_msgs.srv import MapState
from robots_dog_msgs.msg import PerceptionState
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n=rclpy.create_node('console_prepare_scan')
states=[]; scans=[]
sub=n.create_subscription(PerceptionState,'/robot_perception/perception_state',lambda m:states.append(m.status),10)
scan=n.create_subscription(LaserScan,'/laser_scan',lambda m:scans.append(len(m.ranges)),qos_profile_sensor_data)
client=n.create_client(MapState,'/robot_perception/perception_state_switch')
def wait(predicate, seconds):
 end=time.monotonic()+seconds
 while not predicate() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
 return bool(predicate())
def call(value):
 if not client.wait_for_service(timeout_sec=5):raise RuntimeError('感知服务未就绪')
 request=MapState.Request(); request.data=value
 future=client.call_async(request);rclpy.spin_until_future_complete(n,future,timeout_sec=6)
 if not future.done():raise RuntimeError('感知响应超时，未自动重发')
 if not future.result().success:raise RuntimeError('感知请求被拒绝：'+str(future.result()))
try:
 if not wait(lambda:states,10):raise RuntimeError('未收到感知状态，未发送切换指令')
 state=states[-1]
 if state==0:
  call(7)
  if not wait(lambda:states[-1]==32,20):raise RuntimeError('感知未进入 READY')
  state=32
 if state==32:
  call(11)
  if not wait(lambda:states[-1]==100,10):raise RuntimeError('感知未进入 ACTIVE')
 elif state!=100:raise RuntimeError('感知当前状态不适合自动恢复：'+str(state))
 if not wait(lambda:scans,10):raise RuntimeError('感知已激活，但未收到激光')
 print(json.dumps({'state':states[-1],'scan_ranges':scans[-1],'movement_commands':0}),flush=True)
finally:n.destroy_node();rclpy.shutdown()
'''

command = ('source /opt/ros/humble/setup.bash; '
           'source /opt/robot/robot_slam/install/setup.bash; '
           'export ROS_DOMAIN_ID=24 RMW_IMPLEMENTATION=rmw_zenoh_cpp; exec python3 -')
result = subprocess.run(['bash', '-lc', command], input=CODE, text=True, capture_output=True, timeout=70)
print(json.dumps({'returncode': result.returncode, 'stdout': result.stdout, 'stderr': result.stderr}, ensure_ascii=False))
raise SystemExit(result.returncode)
