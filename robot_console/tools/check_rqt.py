#!/usr/bin/env python3
"""Load the real plugin in rqt, exercise it, save a screenshot and exit."""
import os
from pathlib import Path
import sys
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from rqt_gui.main import Main


class CheckMain(Main):
    def create_application(self,argv):
        app=super().create_application(argv)
        from PyQt5.QtCore import QTimer
        def inspect():
            from rqt_robot_console.app import Console
            widgets=[w for w in app.allWidgets() if isinstance(w,Console)]
            if not widgets:
                print('FAIL: console plugin did not load',flush=True)
                app.exit(2);return
            w=widgets[0]
            # The top-level rqt window controls the plugin geometry.
            w.window().resize(1440,960)
            w.canvas.fit_map();w.tick();w.start_nav()
            for _ in range(25):w.tick()
            app.processEvents()
            path=Path(os.environ.get('CONSOLE_SCREENSHOT','rqt_preview.png'))
            w.grab().save(str(path))
            print('PASS: real rqt plugin loaded; demo navigation active; screenshot '+str(path),flush=True)
            w.stop_demo();w.dirty=False
            w.window().close()
        QTimer.singleShot(2000,inspect)
        return app


if __name__=='__main__':
    sys.exit(CheckMain().main(argv=['rqt','--force-discover','--standalone','rqt_robot_console/RobotConsole']))
