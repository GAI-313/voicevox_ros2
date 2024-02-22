from setuptools import setup
from glob import glob
import os
import subprocess

package_name = 'voicevox_ros2'
setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['voicevox_ros2_interface',
                        'setuptools',
                        'simpleaudio'],
    zip_safe=True,
    maintainer='roboworks',
    maintainer_email='nakatogawagai@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "voicevox_ros2_core = voicevox_ros2.run:main",
            "voice_saver = voicevox_ros2.run:voice_saver",
            "pub = voicevox_ros2.pub_test:main",
            "srv = voicevox_ros2.srv_test:main",
        ],
    },
)
