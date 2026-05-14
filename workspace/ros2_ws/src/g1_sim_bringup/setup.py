from glob import glob
import os

from setuptools import setup

package_name = 'g1_sim_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)],
        ),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luis',
    maintainer_email='luis@example.com',
    description='Bringup launch files for G1 Gazebo Harmonic simulation with ROS 2 bridge.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_and_bridge = g1_sim_bringup.sim_and_bridge:main',
            'demo_cmd_vel_movements = g1_sim_bringup.demo_cmd_vel_movements:main',
            'demo_cmd_vel_movements2 = g1_sim_bringup.demo_cmd_vel_movements2:main',
        ],
    },
)
