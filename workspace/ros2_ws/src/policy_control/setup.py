import os

from setuptools import setup


package_name = 'policy_control'


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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luis',
    maintainer_email='luis@example.com',
    description='Policy control helpers for G1 in Gazebo Harmonic.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'observation_publisher = policy_control.observation_publisher:main',
        ],
    },
)
