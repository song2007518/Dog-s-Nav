from qt_gui.plugin import Plugin
from .app import Console


class RobotConsolePlugin(Plugin):
    def __init__(self,context):
        super().__init__(context)
        self.setObjectName('RobotConsolePlugin')
        self.widget=Console()
        if context.serial_number()>1:
            self.widget.setWindowTitle(self.widget.windowTitle()+' (%d)'%context.serial_number())
        context.add_widget(self.widget)

    def shutdown_plugin(self):
        self.widget.shutdown()

    def save_settings(self,plugin_settings,instance_settings):
        # Persist UI preferences only, never arming/motion/connection state.
        instance_settings.set_value('ws_url',self.widget.url.text())

    def restore_settings(self,plugin_settings,instance_settings):
        value=instance_settings.value('ws_url','ws://192.168.234.234:10010')
        self.widget.url.setText(str(value))
