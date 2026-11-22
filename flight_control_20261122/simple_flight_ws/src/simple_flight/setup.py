from glob import glob
from setuptools import find_packages, setup

setup(
    name='simple_flight',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/simple_flight']),
        ('share/simple_flight', ['package.xml']),
        ('share/simple_flight/config', glob('config/*')),
        ('share/simple_flight/launch', glob('launch/*.launch.py')),
        ('share/simple_flight/scripts', glob('scripts/*.sh')),
        ('share/simple_flight/systemd', glob('systemd/*.service')),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': [
        'check_lidar = simple_flight.lidar_check:main',
        'check_pose = simple_flight.pose_check:main',
        'vision_bridge = simple_flight.vision_bridge:main',
        'boot_supervisor = simple_flight.boot_supervisor:main',
        'check_ready = simple_flight.boot_status:main',
    ]},
)
