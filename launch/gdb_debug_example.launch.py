from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 定义参数
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz?')

    # 定义pointlio_mapping节点，注意处理required和launch-prefix属性
    pointlio_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[{
            'use_imu_as_input': False,  # 修改为True以使用IMU作为Point-LIO的输入
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 1,  # 可选值：4, 3
            'space_down_sample': True,
            'filter_size_surf': 0.3,  # 可选值：0.5, 0.3, 0.2, 0.15, 0.1
            'filter_size_map': 0.2,  # 可选值：0.5, 0.3, 0.15, 0.1
            'cube_side_length': 1000,  # 或2000
            'runtime_pos_log_enable': False  # 修改为True以启用
        }],
        on_exit=ExecuteProcess(
            cmd=['gdb', '-ex', 'run', '--args'],
            shell=True
        ),
        prefix='gdb -ex run --args'
    )

    # 条件启动RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'), 'rviz_cfg', 'loam_livox.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )

    return LaunchDescription([
        rviz_arg,
        pointlio_mapping_node,
        rviz_node
    ])
