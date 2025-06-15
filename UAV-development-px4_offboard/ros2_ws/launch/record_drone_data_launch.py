from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '/fmu/out/vehicle_odometry',
                '/fmu/out/vehicle_attitude',
                '/fmu/out/sensor_combined',
                '/fmu/out/distance_sensor',
                '/fmu/out/battery_status',
                '-o', 'drone_flight_bag'
            ],
            output='screen'
        )
    ])

