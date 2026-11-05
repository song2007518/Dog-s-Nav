from glob import glob
from setuptools import setup

setup(
    name='rqt_robot_console',version='0.1.0',packages=['rqt_robot_console'],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/rqt_robot_console']),
        ('share/rqt_robot_console',['package.xml','plugin.xml']),
        ('share/rqt_robot_console/assets',glob('assets/*')),
    ],
    install_requires=['setuptools'],zip_safe=False,
    maintainer='Robot Console Team',maintainer_email='robot-console@example.com',
    description='Offline-first ZSL-1W rqt mission console',license='Apache-2.0',
    entry_points={'console_scripts':['robot_console=rqt_robot_console.app:main']},
)
