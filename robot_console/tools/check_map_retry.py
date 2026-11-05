"""Local regression: lose the first map reply without closing the App session."""
import os,sys,time,threading
from pathlib import Path
from unittest.mock import patch
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'package'))
from PyQt5.QtWidgets import QApplication
from PyQt5.QtTest import QTest
from rqt_robot_console.app import Console
from rqt_robot_console.transport import WsWorker
from mock_device import Server
class FastWorker(WsWorker):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs);self.map_timeout=.2
app=QApplication([]);server=Server(0)
original=server.device.handle;dropped=[]
def drop_first(msg):
    response=original(msg)
    if 'get_all_pgm_map' in msg['data']['req_func'] and not dropped:
        dropped.append(msg);return None
    return response
server.device.handle=drop_first
thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
w=Console();w.show()
try:
    with patch('rqt_robot_console.app.WsWorker',FastWorker):
        w.mode.setCurrentIndex(1);w.url.setText('ws://127.0.0.1:%d'%server.server_address[1]);w.toggle_connection()
    w.poll_timer.setInterval(100);w.map_retry_delay=.1
    end=time.monotonic()+5
    while time.monotonic()<end and w.grid.source!='robot':QTest.qWait(20)
    assert dropped,'fixture did not drop map reply'
    assert w.grid.map_id=='mock_lab','map never recovered: '+str(w.log_lines)
    assert server.device.connections==1,'query timeout must not disconnect'
    assert all(f.startswith('get_') for f,v in server.device.received),'unexpected mutation'
    assert not w.write_enabled.isChecked()
    print('PASS dropped first map response -> automatic read-only retry -> map displayed; same connection; no mutations')
finally:
    w.dirty=False;w.shutdown();w.close();server.shutdown();server.server_close();thread.join(2)
