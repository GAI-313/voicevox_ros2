from setuptools import find_packages, setup

package_name = 'voicevox_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'voicevox_core', 'simpleaudio'],
    zip_safe=True,
    maintainer='GAI-313',
    maintainer_email='nakartogawa.drone@gmail.com',
    description='TODO: Voicevox client for ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voicevox_ros2 = voicevox_ros2.voicevox_ros2:main'
        ],
    },
)
