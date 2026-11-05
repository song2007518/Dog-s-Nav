import json
import math
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PyQt5.QtCore import Qt, QPoint, QEvent
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication
from rqt_robot_console.core import (GridMap,Waypoint,demo_map,demo_plan,save_route,load_route,
                                    request,response_for,validate_points,path_payload,rotate_vector)
from rqt_robot_console.app import Console,assets_dir
from rqt_robot_console.transport import WsWorker
from rqt_robot_console.canvas import ScanView

APP=QApplication.instance() or QApplication([])


class GeometryTests(unittest.TestCase):
    def test_full_3d_rotation_respects_sensor_pitch(self):
        q={'x':0,'y':math.sin(math.pi/4),'z':0,'w':math.cos(math.pi/4)}
        x,y,z=rotate_vector(q,(1,0,0))
        self.assertAlmostEqual(x,0)
        self.assertAlmostEqual(y,0)
        self.assertAlmostEqual(z,-1)
    def test_rotated_origin_roundtrip(self):
        grid=GridMap('rotated',4,3,.2,(12,-8,math.pi/3),[0]*12)
        for x,y in [(0,0),(.5,.5),(3.9,2.9),(2,1)]:
            wx,wy=grid.cell_to_world(x,y)
            px,py=grid.world_to_pixel(wx,wy)
            self.assertAlmostEqual(px,x)
            self.assertAlmostEqual(py,3-y)
            xx,yy=grid.pixel_to_world(px,py)
            self.assertAlmostEqual(xx,wx)
            self.assertAlmostEqual(yy,wy)

    def test_boundaries_do_not_truncate_negative_to_zero(self):
        grid=GridMap('tiny',2,2,1,(0,0,0),[0,100,-1,0])
        self.assertIsNone(grid.occupancy(-.01,.2))
        self.assertIsNone(grid.occupancy(2,.2))
        self.assertFalse(grid.valid_point(.2,1.2))
        self.assertFalse(grid.valid_point(1.2,.2))

    def test_image_top_is_ros_last_row(self):
        grid=GridMap('tiny',2,2,1,(0,0,0),[0,100,-1,0])
        img=grid.image()
        self.assertEqual(img.pixelColor(0,0).name(),'#536174')
        self.assertGreater(img.pixelColor(0,1).red(),200)
        self.assertLess(img.pixelColor(1,1).red(),60)

    def test_export_reload_orientation_and_occupancy(self):
        grid=GridMap('tiny',2,2,.05,(-10,4,.4),[0,100,-1,0])
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'map.yaml';grid.export(p)
            restored=GridMap.from_yaml(p)
            self.assertEqual(restored.cells,grid.cells)
            self.assertEqual(restored.origin,grid.origin)

    def test_real_map_load(self):
        grid=GridMap.from_yaml(assets_dir()/'robot_map.yaml')
        self.assertEqual(grid.resolution,.05)
        self.assertEqual(grid.origin,(-18.7,-15.55,0))
        self.assertGreater(grid.width*grid.height,10000)

    def test_payload_rejects_ambiguous_encoding(self):
        p={'info':{'width':2,'height':1,'resolution':1},'data':[0,255]}
        with self.assertRaises(ValueError):GridMap.from_payload('m',p)
        self.assertEqual(GridMap.from_payload('m',p,'pgm').cells,[100,0])

    def test_invalid_map_dimensions(self):
        with self.assertRaises(ValueError): GridMap('m',3,2,1,(0,0,0),[0])
        with self.assertRaises(ValueError): GridMap('m',1,1,0,(0,0,0),[0])

    def test_route_roundtrip_and_nan_rejection(self):
        pts=[Waypoint('中文点',1,-2,math.pi/2)]
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'route.json';save_route(p,'map_a','巡检',pts)
            mid,rid,restored=load_route(p)
            self.assertEqual((mid,rid),('map_a','巡检'))
            self.assertEqual(restored[0],pts[0])
            with self.assertRaises(ValueError):save_route(p,'a','b',[Waypoint('bad',float('nan'),0)])
            self.assertEqual(load_route(p)[2][0],pts[0])

    def test_pose_quaternion_and_duplicate_names(self):
        p=Waypoint('a',1,2,math.pi)
        self.assertAlmostEqual(p.pose()['orientation']['z'],1)
        self.assertAlmostEqual(p.pose()['orientation']['w'],0)
        self.assertEqual(path_payload('m','r',[p])[2][0][0],'a')
        with self.assertRaises(ValueError):validate_points([p,p])

    def test_astar_goes_around_obstacle(self):
        grid=demo_map()
        path=demo_plan(grid,(-.3,-2.5),(3.6,1.7))
        self.assertGreater(len(path),10)
        for x,y in path:self.assertTrue(grid.valid_point(x,y))
        self.assertEqual(demo_plan(grid,(-2,-1),(0,0)),[])


class ProtocolTests(unittest.TestCase):
    def envelope(self,result,seq=7):
        return {'head':{'type':'app_resp','frame_count':seq},'data':{'req_result':result}}

    def test_all_documented_wrappers(self):
        result={'req_func':'get_loc_status','status':'ok','data':'ContinuousLoc'}
        for wrapper in (None,'AppResponse','AppMapListResponse','AppReponseObjectData','AppResponseObjectData'):
            payload=result if wrapper is None else {wrapper:result}
            self.assertEqual(response_for(self.envelope(payload),'get_loc_status',7),result)

    def test_wrong_sequence_function_and_notifications_rejected(self):
        result={'req_func':'stop_mapping','status':'ok'}
        self.assertIsNone(response_for(self.envelope(result,8),'stop_mapping',7))
        self.assertIsNone(response_for(self.envelope(result),'start_mapping',7))
        result['req_func']='notify_stop_mapping_status'
        self.assertIsNone(response_for(self.envelope(result),'stop_mapping',7))

    def test_load_localization_alias(self):
        r={'req_func':'load_localization_map','status':'ok'}
        self.assertEqual(response_for(self.envelope(r),'loc_load_map',7),r)

    def test_request_retains_numeric_zero(self):
        msg=request('start_mapping',0,8)
        self.assertIs(type(msg['data']['req_func']['start_mapping']),int)
        self.assertGreater(msg['head']['time_stamp'],1_000_000_000_000)

    def test_worker_matches_after_unsolicited_and_wrong_response(self):
        results=[];errors=[];connections=[]
        class Socket:
            def __init__(self):self.messages=[];self.closed=False
            def send(self,raw):
                req=json.loads(raw);seq=req['head']['frame_count'];fn=next(iter(req['data']['req_func']))
                self.messages=[json.dumps({'head':{'type':'alg_error_code_notify'},'data':{'items':[]}}),
                  json.dumps({'head':{'type':'app_resp','frame_count':seq+1},'data':{'req_result':{'req_func':fn,'status':'ok','data':'wrong'}}}),
                  json.dumps({'head':{'type':'app_resp','frame_count':seq},'data':{'req_result':{'AppResponse':{'req_func':fn,'status':'ok','data':'ContinuousLoc'}}}})]
            def recv(self):
                if self.messages:return self.messages.pop(0)
                import websocket
                time.sleep(.01)
                raise websocket.WebSocketTimeoutException('idle timeout')
            def settimeout(self,value):pass
            def close(self,**kwargs):self.closed=True
            def abort(self):self.closed=True
        def factory(*args,**kwargs):
            socket=Socket();connections.append(socket);return socket
        w=WsWorker('ws://127.0.0.1:10010')
        w.result.connect(lambda *args:results.append(args))
        w.error.connect(lambda *args:errors.append(args))
        self.assertIsNotNone(w.submit('get_loc_status'))
        self.assertIsNone(w.submit('get_loc_status'))
        w.submit('get_nav_status')
        with patch('websocket.create_connection',factory):
            w.start()
            deadline=time.monotonic()+2
            while len(results)<2 and time.monotonic()<deadline:QTest.qWait(10)
            w.shutdown();self.assertTrue(w.wait(2000))
        self.assertEqual(len(results),2)
        self.assertFalse(errors)
        self.assertEqual(results[0][2],'ContinuousLoc')
        self.assertEqual(len(connections),1)
        self.assertTrue(all(s.closed for s in connections))


class GuiTests(unittest.TestCase):
    def setUp(self):
        self.w=Console();self.w.show();APP.processEvents();self.w.canvas.fit_map()
    def tearDown(self):
        self.w.dirty=False;self.w.shutdown();self.w.close();APP.processEvents()

    def test_scan_pixels_stay_inside_plot_in_small_window(self):
        view=ScanView();view.resize(330,145)
        view.points=[(float(x)/4,float(y)/4) for x in range(-24,25) for y in range(-24,25)]
        view.show();APP.processEvents()
        img=view.grab().toImage();plot=view.plot_rect()
        orange=0
        for y in range(img.height()):
            for x in range(img.width()):
                c=img.pixelColor(x,y)
                if c.red()>200 and 100<c.green()<190 and c.blue()<100:
                    orange+=1
                    self.assertTrue(plot.contains(x+.5,y+.5),(x,y,plot))
        self.assertGreater(orange,10)
        view.close()

    def test_empty_map_never_receives_world_scan_overlay(self):
        self.w.mode.setCurrentIndex(1)
        self.w.ros_data('pose',(1,2,0,'map'))
        self.w.ros_data('scan',[(1,2)])
        self.assertIsNone(self.w.canvas.robot)
        self.assertEqual(self.w.canvas.scan,[])
        self.assertEqual(self.w.scan_view.points,[(1,2)])
        self.assertTrue(self.w.testAttribute(Qt.WA_StyledBackground))

    def test_read_timeout_clears_status_and_avoids_mutation_warning(self):
        self.w.mode.setCurrentIndex(1)
        self.w.status_times['loc']=time.monotonic()
        self.w.on_error(4,'get_loc_status','读取超时')
        self.assertNotIn('loc',self.w.status_times)
        self.assertFalse(any('设备可能已执行' in s for s in self.w.log_lines))

    def test_initial_state_no_device_threads_and_size(self):
        self.assertIsNone(self.w.worker)
        self.assertIsNone(self.w.monitor)
        self.assertLessEqual(self.w.height(),900)
        self.assertFalse(self.w.live)

    def test_preview_plans_all_legs_without_starting_motion(self):
        robot=self.w.robot
        self.w.preview_demo_route()
        self.assertGreater(len(self.w.canvas.path),10)
        self.assertEqual(self.w.canvas.path[-1],(self.w.points[-1].x,self.w.points[-1].y))
        self.assertEqual(self.w.robot,robot)
        self.assertEqual(self.w.demo_mode,'待命')
        self.assertIsNone(self.w.worker)
        self.w.sync_table(1)
        self.w.reorder(1)
        self.assertEqual(self.w.canvas.path,[])

    def test_presentation_keeps_data_source_and_real_controls_locked(self):
        self.w.mode.setCurrentIndex(1)
        self.w.presentation_btn.setChecked(True)
        self.assertIn('实机',self.w.banner.text())
        self.assertIn('只读',self.w.banner.text())
        self.assertTrue(self.w.logs.isHidden())
        self.assertFalse(self.w.write_enabled.isChecked())
        self.assertFalse(self.w.start_nav_btn.isEnabled())
        self.assertFalse(self.w.pad.isEnabled())
        self.w.presentation_btn.setChecked(False)
        self.assertFalse(self.w.logs.isHidden())

    def test_local_map_clears_demo_scan_and_does_not_show_live_status(self):
        self.w.tick()
        self.assertTrue(self.w.scan_view.points)
        self.w.map_choice.setCurrentIndex(1)
        self.assertEqual(self.w.scan_view.points,[])
        self.assertIsNone(self.w.canvas.robot)
        self.assertIn('无实时数据',self.w.metrics['loc'].value.text())
        self.assertFalse(self.w.start_nav_btn.isEnabled())
        self.assertFalse(self.w.pad.isEnabled())

    def test_click_drag_adds_world_pose(self):
        self.w.add_btn.setChecked(True)
        from PyQt5.QtCore import QPointF
        p=self.w.canvas.mapFromScene(QPointF(*self.w.grid.world_to_pixel(-3,0)))
        q=self.w.canvas.mapFromScene(QPointF(*self.w.grid.world_to_pixel(-3,1)))
        before=len(self.w.points)
        QTest.mousePress(self.w.canvas.viewport(),Qt.LeftButton,pos=p)
        QTest.mouseMove(self.w.canvas.viewport(),q)
        QTest.mouseRelease(self.w.canvas.viewport(),Qt.LeftButton,pos=q)
        self.assertEqual(len(self.w.points),before+1)
        wp=self.w.points[-1]
        self.assertAlmostEqual(wp.x,-3,delta=.03)
        self.assertAlmostEqual(wp.y,0,delta=.03)
        self.assertAlmostEqual(wp.yaw,math.pi/2,delta=.03)

    def test_edit_invalid_cell_reverts(self):
        before=self.w.points[0].x
        self.w.table.item(0,1).setText('nan')
        self.assertEqual(self.w.points[0].x,before)

    def test_reorder_delete_and_bounds(self):
        name=self.w.points[0].name
        self.w.table.selectRow(0);self.w.reorder(1)
        self.assertEqual(self.w.points[1].name,name)
        self.w.delete_point();self.assertEqual(len(self.w.points),3)
        self.w.add_point(1000,1000,0);self.assertEqual(len(self.w.points),3)

    def test_full_demo_route_completes_and_locks_editing(self):
        self.w.start_nav()
        self.assertEqual(self.w.states['nav'],'Active')
        self.w.delete_point();self.assertEqual(len(self.w.points),4)
        for _ in range(1200):
            before=self.w.robot[:2]
            target=self.w.demo_target
            self.w.tick()
            self.assertLessEqual(math.dist(before,self.w.robot[:2]),.060001)
            if self.w.demo_target>target:
                point=self.w.points[target]
                self.assertEqual(self.w.robot[:2],(point.x,point.y))
            if self.w.states['nav'] in ('Failed','Succeed'):break
        self.assertEqual(self.w.states['nav'],'Succeed')

    def test_pause_stop_and_loc_failure(self):
        self.w.start_nav();self.w.pause_nav()
        pose=self.w.robot
        for _ in range(4):self.w.tick()
        self.assertEqual(self.w.robot,pose)
        self.w.pause_nav();self.w.inject_loc_failure()
        self.assertEqual(self.w.demo_mode,'手动接管')
        self.assertEqual(self.w.demo_cmd,(0,0))
        self.assertEqual(self.w.states['loc'],'LocLost')
        self.w.start_nav();self.assertNotEqual(self.w.states['nav'],'Active')

    def test_keyboard_release_and_application_deactivate_stop(self):
        self.w.manual_takeover()
        QTest.keyPress(self.w.pad,Qt.Key_W)
        self.assertGreater(self.w.demo_cmd[0],0)
        QTest.keyRelease(self.w.pad,Qt.Key_W)
        self.assertEqual(self.w.demo_cmd,(0,0))
        self.w.pad.press(Qt.Key_W)
        APP.sendEvent(APP,QEvent(QEvent.ApplicationDeactivate))
        self.assertEqual(self.w.demo_cmd,(0,0))

    def test_stop_mapping_waits_for_save_completion(self):
        self.w.start_mapping();self.w.stop_mapping()
        self.assertEqual(self.w.states['mapping'],'MappingSaveBegin')
        self.w.demo_map_saved()
        self.assertEqual(self.w.states['mapping'],'MappingSaveEnd')

    def test_real_mode_never_creates_velocity_publisher_or_thread_on_entry(self):
        self.w.mode.setCurrentIndex(1)
        self.assertTrue(self.w.live)
        self.assertIsNone(self.w.worker)
        self.assertIsNone(self.w.monitor)
        self.assertFalse(self.w.pad.isEnabled())
        self.assertFalse(self.w.write_enabled.isChecked())
        self.assertEqual(self.w.grid.source,'empty')

    def test_cancelled_result_cannot_rebind_localization(self):
        self.w.mode.setCurrentIndex(1)
        self.w.on_result(99,'loc_load_map',None)
        self.assertIsNone(self.w.loaded_map_id)
        self.assertFalse(self.w.online)

    def test_real_navigation_rejects_stale_localization(self):
        self.w.mode.setCurrentIndex(1)
        self.w.grid=demo_map();self.w.grid.source='robot'
        self.w.map_choice.addItem(self.w.grid.map_id,self.w.grid.map_id)
        self.w.map_choice.blockSignals(True);self.w.map_choice.setCurrentIndex(1);self.w.map_choice.blockSignals(False)
        self.w.load_example();self.w.loaded_map_id=self.w.grid.map_id
        self.w.nav_owner.setChecked(True)
        self.w.states={'mapping':'MappingReady','loc':'ContinuousLoc','nav':'StandBy'}
        with patch.object(self.w,'send') as send:
            self.w.start_nav();send.assert_not_called()
        self.assertIn('新鲜',self.w.log_lines[-1])

    def test_demo_obstacle_replans_or_stops(self):
        self.w.start_nav()
        for _ in range(30):self.w.tick()
        self.w.inject_obstacle()
        self.assertIn(self.w.states['nav'],('Active','Failed'))
        for x,y in self.w.demo_path:self.assertTrue(self.w.grid.valid_point(x,y))


if __name__=='__main__':unittest.main(verbosity=2)
