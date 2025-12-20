from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    # Arguments
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='pbvs',
        description='Controller type: pbvs or ibvs'
    )

    marker_id_arg = DeclareLaunchArgument(
        'marker_id',
        default_value='0',
        description='Target marker ID'
    )

    controller_type = LaunchConfiguration('controller')
    marker_id = LaunchConfiguration('marker_id')

    # PBVS Node
    pbvs_node = Node(
        package='challenge2_1',
        executable='demo_pbvs_controller',
        name='pbvs_controller',
        output='screen',
        parameters=[{
            'target_marker_id': marker_id,
            'kp_linear': 0.4,
            'kp_angular': 0.8,
            'desired_distance': 0.5
        }],
        condition=IfCondition(controller_type)
    )

    # IBVS Node
    ibvs_node = Node(
        package='challenge2_1',
        executable='demo_ibvs_controller',
        name='ibvs_controller',
        output='screen',
        parameters=[{
            'target_marker_id': marker_id,
            'lambda_gain': 0.5,
            'desired_depth': 0.5
        }],
        condition=IfCondition(controller_type)
    )

    return LaunchDescription([
        controller_arg,
        marker_id_arg,
        pbvs_node,
        ibvs_node
    ])
