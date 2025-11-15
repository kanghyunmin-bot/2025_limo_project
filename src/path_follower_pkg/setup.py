from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'path_follower_pkg'

setup(
    name=package_name,
    version='2.7.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ✅ ament_index에 제대로 등록
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
         glob('launch/*.launch.py') if os.path.exists('launch') else []),
        ('share/' + package_name + '/config', 
         glob('config/*.yaml') if os.path.exists('config') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kang Hyunmin',
    maintainer_email='kanghyunmin@example.com',
    description='ROS2 Path Follower',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'follower_node = path_follower_pkg.follower_node:main',
            'control_panel = path_follower_pkg.control_panel.main:main',
            'fake_robot = path_follower_pkg.fake_robot:main',
        ],
    },
)

# scipy가 없다면 추가
# install_requires=['scipy'] 확인
