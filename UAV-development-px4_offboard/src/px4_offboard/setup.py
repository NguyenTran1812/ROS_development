import os
from glob import glob
from setuptools import setup

package_name = 'px4_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*rviz'))
        # (os.path.join('share', package_name), ['scripts/TerminatorScript.sh'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Braden',
    maintainer_email='braden@arkelectron.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'offboard_control = px4_offboard.offboard_control:main',
                'visualizer = px4_offboard.visualizer:main',
                'visualizer_real_fly = px4_offboard.visualizer_real_fly:main',
                'velocity_control = px4_offboard.velocity_control:main',
                'control = px4_offboard.control:main',
                'processes = px4_offboard.processes:main', 
                'imu_reader = px4_offboard.imu_reader:main',
                'distance_sensor = px4_offboard.distance_sensor:main',
                'optical_flow_camera_tof_node = optical_flow_camera_tof.optical_flow_camera_tof_node:main',
                #'height_baro = px4_offboard.height_baro:main'
                'baro_to_altitude = px4_offboard.baro_to_altitude:main',
                'mag_raw_publisher = px4_offboard.mag_raw_publisher:main',
                'sensor_splitter = px4_offboard.sensor_splitter:main',
                'imu_raw_publisher = px4_offboard.imu_raw_publisher:main',
 
        ],
    },
)
