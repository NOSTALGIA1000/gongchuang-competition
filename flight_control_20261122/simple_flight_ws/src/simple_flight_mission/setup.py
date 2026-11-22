from glob import glob
from setuptools import find_packages, setup

setup(
    name='simple_flight_mission',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/simple_flight_mission']),
        ('share/simple_flight_mission', ['package.xml']),
        ('share/simple_flight_mission/config', glob('config/*')),
        ('share/simple_flight_mission/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': [
        'waypoint_mission = simple_flight_mission.waypoint_node:main',
        'mission_ground_check = simple_flight_mission.ground_check:main',
        'hover_control = simple_flight_mission.hover_node:main',
        'route_control = simple_flight_mission.route_node:main',
        'route_node = simple_flight_mission.route_node:main',
        'competition_mission = simple_flight_mission.competition_node:main',
    ]},
)
