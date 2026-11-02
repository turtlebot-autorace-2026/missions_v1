import glob

from setuptools import find_packages, setup

package_name = 'autorace_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='turtlebot-autorace-2026',
    maintainer_email='eureka01020304@gmail.com',
    description='Mission FSM and cmd_vel mux for TurtleBot3 AutoRace 2026',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_fsm = autorace_mission.mission_fsm:main',
            'cmd_vel_mux = autorace_mission.cmd_vel_mux:main',
        ],
    },
)
