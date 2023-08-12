from setuptools import setup
from glob import glob
import os
import subprocess

package_name = 'voicevox_ros2'

_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_dir = os.path.join(_dir, 'src', 'voicevox_ros2')
if os.path.exists(os.path.join(_dir, "voicevox_core")):
    pass
else:
    with open(os.devnull, 'w') as null_file:
        subprocess.call(["bash", os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "setup.bash")], 
                        stdout=null_file, stderr=subprocess.STDOUT)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roboworks',
    maintainer_email='nakatogawagai@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "voicevox_ros = voicevox_ros2.run:main",
            "pub = voicevox_ros2.pub_test:main",
            "srv = voicevox_ros2.srv_test:main",
        ],
    },
)
