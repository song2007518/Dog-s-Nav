import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch, Mock

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PyQt5.QtWidgets import QApplication
from rqt_robot_console.stack import StackManager, process_info, runs_script, stack_environment, default_start_all
from rqt_robot_console.app import Console

APP = QApplication.instance() or QApplication([])


def wait_for(predicate, timeout=4):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate():
            return
        time.sleep(.03)
    raise AssertionError('Timed out waiting for local fixture')


class StackTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(prefix='console_stack_test_')
        self.root=Path(self.tmp.name)
        self.script=self.root/'scripts/start_all.sh'
        self.script.parent.mkdir()
        # This fixture contains no ROS, SSH, network or robot commands.
        self.script.write_text('''#!/bin/bash
cd "$(dirname "$0")/.."
mkdir -p logs
echo $$ > logs/boot.pid
trap 'echo stop-motion >> actions; echo lie-down >> actions; sleep 0.2; echo passive >> actions; rm -f logs/boot.pid; exit 0' TERM
echo '=== [1/8] fake startup ==='
/usr/bin/python3 -c 'import os,json; open("environment.json","w").write(json.dumps(dict(os.environ)))'
while true; do sleep 0.1; done
''')
        self.manager=StackManager(self.script,self.root/'logs')

    def tearDown(self):
        state=self.manager.status()
        if state['can_stop']:
            self.manager.stop()
        if self.manager.child:
            if self.manager.child.poll() is None and state['state'] != 'stopping':
                self.manager.child.terminate()
            self.manager.child.wait(timeout=5)
        self.tmp.cleanup()

    def launch(self):
        pid=self.manager.start()
        wait_for(lambda:(self.root/'environment.json').exists())
        return pid

    def test_relocated_source_and_install_choose_own_navigation_repo(self):
        marker = self.root / 'src/custom_nav/package.xml'
        marker.parent.mkdir(parents=True)
        marker.write_text('<package/>')
        for relative in ('robot_console/package/rqt_robot_console/stack.py',
                         'robot_console/install/rqt_robot_console/lib/python3.10/site-packages/rqt_robot_console/stack.py'):
            with patch.dict(os.environ, {}, clear=True), patch('rqt_robot_console.stack.__file__', str(self.root / relative)):
                self.assertEqual(default_start_all(), self.script)
                self.assertEqual(StackManager().script, self.script)

    def test_explicit_script_override_does_not_fall_back_when_missing(self):
        missing = self.root / 'missing/start_all.sh'
        with patch.dict(os.environ, {'ROBOT_CONSOLE_START_ALL': str(missing)}):
            self.assertEqual(StackManager().script, missing)
            self.assertIn(str(missing), StackManager().local_checks())

    def test_start_duplicate_reopen_and_stop_order(self):
        pid=self.launch()
        other=StackManager(self.script,self.root/'logs')
        self.assertEqual(other.status()['processes'][0]['pid'],pid)
        self.assertIn('1/8',other.stage())
        with self.assertRaises(RuntimeError): other.start()
        other.stop()
        self.assertEqual(other.status()['state'],'stopping')
        with self.assertRaises(RuntimeError): other.stop()
        wait_for(lambda:not other.status()['busy'])
        self.assertEqual((self.root/'actions').read_text().splitlines(),['stop-motion','lie-down','passive'])
        self.assertIn('未核验',other.status()['text'])

    def test_stale_pid_never_signals_unrelated_process(self):
        sleeper=subprocess.Popen(['/bin/sleep','30'])
        try:
            self.manager.log_dir.mkdir()
            (self.manager.log_dir/'boot.pid').write_text(str(sleeper.pid))
            self.assertEqual(self.manager.status()['state'],'blocked')
            with self.assertRaises(RuntimeError): self.manager.stop()
            with self.assertRaises(RuntimeError): self.manager.start()
            self.assertIsNone(sleeper.poll())
        finally:
            sleeper.terminate();sleeper.wait()

    def test_bad_pid_file_does_not_prevent_verified_stop(self):
        self.launch()
        (self.manager.log_dir/'boot.pid').write_text('invalid')
        self.manager.stop()
        wait_for(lambda:not self.manager.status()['busy'])
        self.assertIn('passive',(self.root/'actions').read_text())

    def test_wrong_start_time_cannot_claim_stop_in_progress(self):
        pid=self.launch()
        self.manager.save_metadata({'pid':pid,'start':'wrong','stop_at':time.time()})
        self.assertEqual(self.manager.status()['state'],'running')

    def test_launch_environment_isolated_from_offline_and_direct_session(self):
        with patch.dict(os.environ,{'ROS_DOMAIN_ID':'124','ROS_LOCALHOST_ONLY':'1',
                'RMW_IMPLEMENTATION':'rmw_fastrtps_cpp','ZENOH_SESSION_CONFIG_URI':'/tmp/direct.json5',
                'XDG_CONFIG_HOME':'/tmp/console_config'}):
            self.launch()
            env=json.loads((self.root/'environment.json').read_text())
            self.assertEqual(env['ROS_DOMAIN_ID'],'24')
            self.assertEqual(env['RMW_IMPLEMENTATION'],'rmw_zenoh_cpp')
            self.assertNotIn('ROS_LOCALHOST_ONLY',env)
            self.assertNotIn('ZENOH_SESSION_CONFIG_URI',env)
            self.assertNotIn('XDG_CONFIG_HOME',env)
            self.assertEqual(os.environ['ROS_DOMAIN_ID'],'124')

    def test_exit_failure_is_not_success(self):
        self.script.write_text('#!/bin/bash\necho fixture-failure\nexit 9\n')
        self.manager.start()
        wait_for(lambda:self.manager.status()['exit_code']==9)
        self.assertFalse(self.manager.status()['busy'])
        self.assertIn('异常退出（9）',self.manager.status()['text'])
        self.assertIn('fixture-failure',self.manager.log_tail())

    def test_shell_command_string_is_not_script_identity(self):
        info={'args':['bash','-c','echo '+str(self.script)],'cwd':str(self.root)}
        self.assertFalse(runs_script(info,self.script))

    def test_gui_start_and_f12_use_the_managed_script(self):
        w=Console()
        try:
            w.stack=self.manager
            w.stack_state=self.manager.status()
            w.dirty=False
            w.mode.setCurrentIndex(1)
            w.stack_consent.setChecked(True)
            w._stack_pending=True
            # Substitute only the read-only connectivity result; run the actual
            # GUI continuation and process manager against the inert fixture.
            w.stack_checked({'ok':True,'missing':[],'hosts':{'fixture':True}})
            wait_for(lambda:(self.root/'environment.json').exists())
            w.poll_stack()
            self.assertTrue(w.stack_state['busy'])
            self.assertFalse(w.connect_btn.isEnabled())
            self.assertIn('收工',w.stop_btn.text())
            w.stop_task()
            wait_for(lambda:not self.manager.status()['busy'])
            w.poll_stack()
            self.assertEqual((self.root/'actions').read_text().splitlines(),['stop-motion','lie-down','passive'])
            self.assertTrue(w.connect_btn.isEnabled())
            self.assertIsNone(w.worker)  # Never reconnect automatically after stopping.
        finally:
            w.dirty=False;w.shutdown();w.close()


class StackGuiTests(unittest.TestCase):
    def setUp(self):
        self.w=Console()
        self.w.dirty=False
        self.w.mode.setCurrentIndex(1)

    def tearDown(self):
        self.w.dirty=False
        self.w.shutdown()
        self.w.close()

    def test_start_requires_real_mode_and_explicit_posture_choice(self):
        with patch.object(self.w,'check_stack') as check:
            self.w.start_stack()
            check.assert_not_called()
            self.w.stack_consent.setChecked(True)
            self.w.start_stack()
            check.assert_called_once_with(True)

    def test_busy_stack_blocks_websocket_even_for_programmatic_send(self):
        state={'state':'running','busy':True,'processes':[{'pid':42}], 'can_stop':True,
               'text':'fixture running','components':[],'exit_code':None}
        with patch.object(self.w.stack,'status',return_value=state):
            self.w.toggle_connection()
            self.assertIsNone(self.w.worker)
            worker=Mock()
            self.w.worker=worker
            self.w.write_enabled.setChecked(True)
            self.assertIsNone(self.w.send('start_mapping',0))
            worker.submit.assert_not_called()
            self.w.worker=None

    def test_failed_or_cancelled_check_does_not_launch(self):
        self.w._stack_pending=True
        self.w.stack_consent.setChecked(True)
        with patch.object(self.w.stack,'start') as start:
            self.w.stack_checked({'ok':False,'missing':[],'hosts':{'192.168.234.1':False}})
            start.assert_not_called()
            self.assertFalse(self.w._stack_pending)
            self.w._stack_pending=True
            self.w.cancel_stack_start()
            self.w.stack_checked({'ok':True,'missing':[],'hosts':{'fixture':True}})
            start.assert_not_called()

    def test_inflight_mutation_keeps_connections_and_prevents_launch(self):
        self.w._stack_pending=True
        self.w.stack_consent.setChecked(True)
        self.w.pending={1:('reset_loc',None)}
        with patch.object(self.w,'disconnect_device') as disconnect, patch.object(self.w.stack,'start') as start:
            self.w.stack_checked({'ok':True,'missing':[],'hosts':{'fixture':True}})
            disconnect.assert_not_called()
            start.assert_not_called()

    def test_connection_shutdown_finishes_before_launcher(self):
        self.w._stack_pending=True
        self.w._stack_deadline=time.monotonic()+5
        self.w.stack_consent.setChecked(True)
        retired=Mock()
        retired.isRunning.return_value=True
        self.w.retired=[retired]
        with patch.object(self.w.stack,'start') as start:
            self.w._finish_stack_start()
            start.assert_not_called()
            self.w.cancel_stack_start()
        self.w.retired=[]

    def test_window_shutdown_does_not_send_posture_command(self):
        with patch.object(self.w.stack,'stop') as stop:
            self.w.shutdown()
            stop.assert_not_called()

    def test_path_requires_ros_map_and_expires(self):
        from rqt_robot_console.core import GridMap
        self.w.ros_map.setChecked(True)
        self.w.ros_data('path',[(1.,2.),(3.,4.)])
        self.assertEqual(self.w.canvas.path,[])
        self.w.grid=GridMap('ros_live',2,2,1,(0,0,0),[0]*4,source='ros')
        self.w.ros_data('path',[(1.,2.),(3.,4.)])
        self.assertEqual(len(self.w.canvas.path),2)
        self.w.last_path=time.monotonic()-2
        self.w.tick()
        self.assertEqual(self.w.canvas.path,[])
        self.w.ros_data('path',[(1.,2.)])
        self.w.ros_data('path_unavailable','invalid frame')
        self.assertEqual(self.w.canvas.path,[])
