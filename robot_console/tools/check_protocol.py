#!/usr/bin/env python3
"""Full console -> localhost socket -> mock device integration. No robot I/O."""
import os
from pathlib import Path
import sys
import threading
import time
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'package'))
from PyQt5.QtWidgets import QApplication
from PyQt5.QtTest import QTest
from rqt_robot_console.app import Console
from rqt_robot_console.core import Waypoint
from mock_device import Server


def main():
    app=QApplication([])
    server=Server(0)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    w=Console();w.show()
    def wait_for(predicate,label,seconds=5):
        end=time.monotonic()+seconds
        while not predicate() and time.monotonic()<end:QTest.qWait(20)
        assert predicate(),label+' failed\n'+'\n'.join(w.log_lines[-8:])
        print('PASS',label,flush=True)
    try:
        w.mode.setCurrentIndex(1)
        w.url.setText('ws://127.0.0.1:%d'%server.server_address[1])
        w.toggle_connection()
        wait_for(lambda:w.online and w.grid.map_id=='mock_lab','connect, read status and decode map')
        assert not w.write_enabled.isChecked()
        w.send('start_mapping',0)
        QTest.qWait(100)
        assert not any(fn=='start_mapping' for fn,v in server.device.received)
        print('PASS read-only gate')
        w.write_enabled.setChecked(True)
        w.start_mapping()
        wait_for(lambda:server.device.mapping=='MappingRunning','start mapping numeric zero')
        w.stop_mapping()
        wait_for(lambda:w.states['mapping']=='MappingSaveBegin','save is pending, not completed on ack')
        QTest.qWait(900);w.poll()
        wait_for(lambda:w.states['mapping']=='MappingSaveEnd','map save completion from polling')
        w.load_localization()
        wait_for(lambda:w.loaded_map_id=='mock_lab','localization response alias')
        QTest.qWait(650);w.poll()
        wait_for(lambda:w.states['loc']=='ContinuousLoc','continuous localization')
        w.add_point(-3,-2,0);w.add_point(0,-2,1.57)
        w.upload_route()
        wait_for(lambda:'patrol_01' in server.device.routes and 'patrol_01' in w.known_paths.get('mock_lab',set()),
                 'save ordered named route and receive acknowledgement')
        w.nav_owner.setChecked(True);w.start_nav()
        wait_for(lambda:server.device.nav=='Active','start multi-point navigation')
        w.poll();wait_for(lambda:w.states['nav']=='Active','navigation accepted vs active distinction')
        QTest.qWait(300)
        assert server.device.nav=='Active','disconnect-on-poll cancelled navigation'
        assert server.device.connections==1,'polling must reuse the App connection'
        print('PASS persistent connection survives polling without disconnect-stop')
        w.pause_nav();wait_for(lambda:server.device.nav=='Pause','pause navigation')
        w.poll();wait_for(lambda:w.states['nav']=='Pause','pause status')
        w.pause_nav();wait_for(lambda:server.device.nav=='Active','resume navigation')
        w.stop_task();wait_for(lambda:server.device.nav=='StandBy','priority stop navigation')
        w.poll();wait_for(lambda:w.states['nav']=='StandBy','stop acknowledged and state refreshed')
        w.disconnect_device()
        assert not w.write_enabled.isChecked()
        print('PASS disconnect disarms real operations')
        print('ALL PROTOCOL INTEGRATION CHECKS PASSED')
    finally:
        w.dirty=False;w.shutdown();w.close()
        server.shutdown();server.server_close();thread.join(2)


if __name__=='__main__':main()
