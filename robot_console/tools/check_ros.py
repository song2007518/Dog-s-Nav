#!/usr/bin/env python3
"""Synthetic ROS telemetry integration on an isolated localhost domain only."""
import os
from pathlib import Path
import sys
import math
import time
os.environ['ROS_DOMAIN_ID']='124'
os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'package'))
from PyQt5.QtWidgets import QApplication
from PyQt5.QtTest import QTest
import rclpy
from rclpy.context import Context
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import TransformStamped,PoseStamped
from nav_msgs.msg import Odometry,OccupancyGrid,Path as RosPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
from rqt_robot_console.transport import RosMonitor


def main():
    app=QApplication([])
    ctx=Context();rclpy.init(context=ctx,args=[])
    node=rclpy.create_node('robot_console_test_source',context=ctx)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    sourcepub=node.create_publisher(Header,'/tf_manager/odom_type',qos)
    posepub=node.create_publisher(Odometry,'/odom/current_pose',qos)
    scanpub=node.create_publisher(LaserScan,'/laser_scan',qos)
    pathpub=node.create_publisher(RosPath,'/custom_nav/path',qos)
    mappub=node.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    broadcaster=StaticTransformBroadcaster(node)
    a=TransformStamped();a.header.frame_id='map';a.child_frame_id='odom'
    a.transform.translation.x=10.;a.transform.translation.y=20.
    a.transform.rotation.z=math.sin(math.pi/4);a.transform.rotation.w=math.cos(math.pi/4)
    b=TransformStamped();b.header.frame_id='base_link';b.child_frame_id='laser_test'
    b.transform.translation.x=.2;b.transform.rotation.z=math.sin(math.pi/4);b.transform.rotation.w=math.cos(math.pi/4)
    monitor=RosMonitor();data={};errors=[]
    monitor.telemetry.connect(lambda kind,value:data.update({kind:value}))
    monitor.error.connect(errors.append)
    monitor.start()
    try:
        deadline=time.monotonic()+10
        while time.monotonic()<deadline and not all(k in data for k in ('pose','scan','map','path')):
            now=node.get_clock().now().to_msg()
            a.header.stamp=now;b.header.stamp=now;broadcaster.sendTransform([a,b])
            pose=Odometry();pose.header.frame_id='odom';pose.header.stamp=now
            pose.pose.pose.position.x=1.;pose.pose.pose.position.y=2.;pose.pose.pose.orientation.w=1.
            source=Header();source.stamp=now;source.frame_id='localization_odom';sourcepub.publish(source)
            posepub.publish(pose)
            scan=LaserScan();scan.header.frame_id='laser_test';scan.header.stamp=now
            scan.range_min=.1;scan.range_max=5.;scan.angle_increment=.1;scan.ranges=[1.,float('nan'),float('inf')]
            scanpub.publish(scan)
            grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=2;grid.info.height=2
            grid.info.resolution=.1;grid.info.origin.orientation.w=1.;grid.data=[0,100,-1,0]
            mappub.publish(grid)
            route=RosPath();route.header.frame_id='map';route.header.stamp=now
            point=PoseStamped();point.header.frame_id='map';point.pose.position.x=1.;point.pose.position.y=2.
            route.poses=[point];pathpub.publish(route)
            QTest.qWait(100)
        assert not errors,errors
        assert all(k in data for k in ('pose','scan','map')),data
        x,y,yaw,frame=data['pose']
        assert abs(x-8)<1e-5 and abs(y-21)<1e-5 and abs(yaw-math.pi/2)<1e-5 and frame=='map',data['pose']
        assert len(data['scan'])==1 and abs(data['scan'][0][0]-.2)<1e-5 and abs(data['scan'][0][1]-1)<1e-5,data['scan']
        assert data['map'].cells==[0,100,-1,0]
        assert data['odom_type']=='localization_odom'
        assert data['path']==[(1.,2.)],data
        for invalid in ('frame','time','nan'):
            data.clear()
            route.header.frame_id='odom' if invalid=='frame' else 'map'
            route.poses[0].pose.position.x=float('nan') if invalid=='nan' else 1.
            for _ in range(4):
                route.header.stamp=node.get_clock().now().to_msg()
                if invalid=='time': route.header.stamp.sec-=10
                pathpub.publish(route);QTest.qWait(120)
            assert 'path_unavailable' in data and 'path' not in data,(invalid,data)
        data.clear();route.poses=[];route.header.frame_id='map'
        for _ in range(4):
            route.header.stamp=node.get_clock().now().to_msg();pathpub.publish(route);QTest.qWait(120)
        assert data.get('path')==[],data
        print('PASS custom_nav path subscription, frame/time/NaN rejection and empty-path clearing',flush=True)
        # Switching to MC odometry must not appear as a valid map localisation.
        data.clear()
        deadline=time.monotonic()+1.5
        while time.monotonic()<deadline:
            source.frame_id='mc_odom';sourcepub.publish(source)
            pose.header.stamp=node.get_clock().now().to_msg();posepub.publish(pose)
            QTest.qWait(100)
        assert '尚无有效定位' in data.get('pose_unavailable',''),data
        data.clear()
        for _ in range(4):
            sourcepub.publish(source);pose.header.stamp=node.get_clock().now().to_msg();posepub.publish(pose);QTest.qWait(100)
        assert 'pose' not in data,data
        print('PASS ROS pose transformed odom -> map, laser -> base_link, invalid scan filtering, latched map subscription',flush=True)
    finally:
        monitor.shutdown();assert monitor.wait(5000)
        node.destroy_node();ctx.shutdown()
    print('PASS ROS monitor shutdown without publishers or robot control',flush=True)


if __name__=='__main__':main()
