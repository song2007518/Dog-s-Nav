#!/usr/bin/env python3
"""Local-only WebSocket protocol simulator. No ROS, SDK or robot connections.

Implements the small RFC6455 subset used by websocket-client for offline testing.
This is a development fixture, not an exposed production WebSocket server.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import socketserver
import struct
import sys
import threading
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'package'))
from rqt_robot_console.core import demo_map


class Device:
    def __init__(self):
        self.lock=threading.Lock()
        self.mapping='MappingReady'
        self.loc='Init'
        self.nav='StandBy'
        self.save_at=0
        self.loc_at=0
        self.nav_at=0
        self.routes={}
        self.received=[]
        self.connections=0
        grid=demo_map()
        self.map={'header':{'frame_id':'map'},'info':{'resolution':grid.resolution,
          'width':grid.width,'height':grid.height,'origin':{'position':{'x':grid.origin[0],'y':grid.origin[1],'z':0},
          'orientation':{'x':0,'y':0,'z':0,'w':1}}},'data':grid.cells,'create_time_ms':1}

    def handle(self,msg):
        with self.lock:
            now=time.monotonic()
            if self.save_at and now>=self.save_at:self.mapping='MappingSaveEnd';self.save_at=0
            if self.loc_at and now>=self.loc_at:self.loc='ContinuousLoc';self.loc_at=0
            if self.nav_at and now>=self.nav_at and self.nav=='Active':self.nav='Succeed';self.nav_at=0
            funcs=msg.get('data',{}).get('req_func',{})
            if not isinstance(funcs,dict) or len(funcs)!=1:raise ValueError('req_func must have one entry')
            fn,val=next(iter(funcs.items()))
            self.received.append((fn,val))
            data=None;status='ok';error=None;response_fn=fn
            if fn=='get_mapping_status':data=self.mapping
            elif fn=='get_loc_status':data=self.loc
            elif fn=='get_nav_status':data=self.nav
            elif fn=='get_all_pgm_map':data={'mock_lab':[self.map,[]]}
            elif fn=='get_pgm_map':data=self.map
            elif fn=='start_mapping':
                if type(val) is not int or val!=0:status='error';error='start_mapping expects numeric zero'
                else:self.mapping='MappingRunning'
            elif fn=='stop_mapping':self.mapping='MappingSaveBegin';self.save_at=now+.8
            elif fn=='loc_load_map':
                response_fn='load_localization_map'
                if val!='mock_lab':status='error';error='unknown map'
                else:self.loc='MapLoading';self.loc_at=now+.5
            elif fn=='reset_loc':self.loc='InitLocalization';self.loc_at=0
            elif fn=='add_nav_path':
                mid,rid,pts=val
                if mid!='mock_lab':status='error';error='unknown map'
                elif rid in self.routes:status='error';error='route exists; use modify_nav_path'
                else:self.routes[rid]=pts
            elif fn=='modify_nav_path':
                mid,old,new,pts=val
                if old not in self.routes:status='error';error='route not found'
                else:del self.routes[old];self.routes[new]=pts
            elif fn=='get_all_paths_by_mapid':data=[val,list(self.routes),self.routes]
            elif fn in ('start_multi_nav_by_points','start_multi_nav'):
                if self.loc!='ContinuousLoc' or self.nav!='StandBy':status='error';error='not ready'
                else:self.nav='Active';self.nav_at=now+30
            elif fn=='pause_nav':self.nav='Pause'
            elif fn=='continue_nav':self.nav='Active';self.nav_at=now+30
            elif fn=='stop_nav':self.nav='StandBy';self.nav_at=0
            else:status='error';error='mock does not implement '+fn
            result={'req_func':response_fn,'status':status,'msg':error,'data':data}
            seq=msg['head']['frame_count']
            # Exercise both manual and deployed wrapped response shapes.
            payload=result if seq%2 else {'AppMapListResponse' if fn=='get_all_pgm_map' else 'AppResponse':result}
            return {'head':{'type':'app_resp','frame_count':seq,'time_stamp':int(time.time()*1000),'source':'mock_device'},
                    'data':{'req_result':payload}}


class Handler(socketserver.StreamRequestHandler):
    def frame(self,opcode,payload):
        length=len(payload)
        header=bytes([0x80|opcode])
        if length<126:header+=bytes([length])
        elif length<65536:header+=b'\x7e'+struct.pack('!H',length)
        else:header+=b'\x7f'+struct.pack('!Q',length)
        self.wfile.write(header+payload);self.wfile.flush()

    def handle(self):
        self.connection.settimeout(10)
        self.server.device.connections+=1
        try:
            first=self.rfile.readline(4096)
            if not first.startswith(b'GET '):return
            headers={}
            for _ in range(64):
                line=self.rfile.readline(8192)
                if line in (b'\r\n',b''):break
                key,value=line.decode().split(':',1);headers[key.lower()]=value.strip()
            accept=base64.b64encode(hashlib.sha1((headers['sec-websocket-key']+'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()
            self.wfile.write(('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: '+accept+'\r\n\r\n').encode())
            self.wfile.flush()
            while True:
                header=self.rfile.read(2)
                if len(header)!=2:return
                opcode=header[0]&15;masked=header[1]&128;length=header[1]&127
                if length==126:length=struct.unpack('!H',self.rfile.read(2))[0]
                elif length==127:length=struct.unpack('!Q',self.rfile.read(8))[0]
                if length>2_000_000:return
                mask=self.rfile.read(4) if masked else b''
                data=self.rfile.read(length)
                if len(data)!=length:return
                if mask:data=bytes(x^mask[i%4] for i,x in enumerate(data))
                if opcode==8:self.frame(8,b'');return
                if opcode==9:self.frame(10,data);continue
                if opcode!=1:return
                response=self.server.device.handle(json.loads(data))
                if response is not None:self.frame(1,json.dumps(response,ensure_ascii=False).encode())
        except (OSError,ValueError,KeyError):
            return
        finally:
            with self.server.device.lock:
                if self.server.device.nav in ('Active','Pause'):
                    self.server.device.nav='StandBy'
                    self.server.device.nav_at=0


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address=True
    daemon_threads=True
    def __init__(self,port=10010):
        self.device=Device()
        super().__init__(('127.0.0.1',port),Handler)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=10010)
    args=parser.parse_args()
    with Server(args.port) as server:
        print('本机模拟设备：ws://127.0.0.1:%d（不连接机器狗）'%server.server_address[1],flush=True)
        try:server.serve_forever()
        except KeyboardInterrupt:pass
