#!/usr/bin/env python3
"""Read-only diagnostics. --robot explicitly enables robot status/network reads."""
import argparse
from datetime import datetime
import importlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'package'))
from rqt_robot_console.core import request,response_for,atomic_json


def command(args,seconds=6):
    try:
        p=subprocess.run(args,capture_output=True,text=True,timeout=seconds)
        return {'returncode':p.returncode,'stdout':p.stdout[-20000:],'stderr':p.stderr[-8000:]}
    except Exception as exc:return {'error':str(exc)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot',action='store_true',help='读取机器狗状态，不发送任何控制命令')
    parser.add_argument('--url',default='ws://192.168.234.234:10010')
    parser.add_argument('--output')
    args=parser.parse_args()
    result={'created':datetime.now().isoformat(),'robot_reads_enabled':args.robot,
            'environment':{k:os.environ.get(k) for k in ('ROS_DISTRO','ROS_DOMAIN_ID','RMW_IMPLEMENTATION','ROS_LOCALHOST_ONLY')},
            'dependencies':{},'commands':{},'websocket':{}}
    for name in ('PyQt5.QtWidgets','yaml','websocket','rclpy','qt_gui.plugin','tf2_ros'):
        try:
            m=importlib.import_module(name);result['dependencies'][name]={'ok':True,'file':getattr(m,'__file__',None)}
        except Exception as exc:result['dependencies'][name]={'ok':False,'error':str(exc)}
    for label,args_cmd in [('system',['uname','-a']),('interfaces',['ip','-brief','address']),('routes',['ip','route'])]:
        result['commands'][label]=command(args_cmd)
    if args.robot:
        import websocket
        for seq,func in enumerate(('get_loc_status','get_mapping_status','get_nav_status'),1):
            sock=None
            try:
                sock=websocket.create_connection(args.url,timeout=4,http_no_proxy=['*'])
                sock.send(json.dumps(request(func,None,seq)))
                deadline=time.monotonic()+5
                while time.monotonic()<deadline:
                    sock.settimeout(max(.1,deadline-time.monotonic()))
                    m=json.loads(sock.recv());res=response_for(m,func,seq)
                    if res is not None:
                        result['websocket'][func]=res;break
                else:result['websocket'][func]={'error':'no matching response'}
            except Exception as exc:result['websocket'][func]={'error':str(exc)}
            finally:
                if sock:
                    try:sock.close(timeout=.2)
                    except Exception:pass
        for label,args_cmd in [
            ('nodes',['ros2','node','list','--no-daemon']),
            ('topics',['ros2','topic','list','-t','--no-daemon']),
            ('services',['ros2','service','list','-t','--no-daemon']),
            ('pose_header',['ros2','topic','echo','/odom/current_pose','--once','--field','header']),
            ('scan_header',['ros2','topic','echo','/laser_scan','--once','--field','header']),
            ('odom_type',['ros2','topic','echo','/tf_manager/odom_type','--once']),
        ]:
            print('读取 '+label,flush=True)
            result['commands'][label]=command(args_cmd)
    dest=Path(args.output) if args.output else Path(__file__).resolve().parents[1]/'runtime'/'diagnostics'/('diagnostics_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
    atomic_json(dest,result)
    print('诊断文件：'+str(dest))
    print('未发送建图、定位重置、导航、站立或速度命令。')


if __name__=='__main__':main()
