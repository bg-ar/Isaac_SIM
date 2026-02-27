import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument

def generate_launch_description():
    # 1. use_sim_time 인자 추가 (기본값 True)
    # 시뮬레이션 환경이므로 반드시 True여야 합니다.
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo/Isaac) clock if true'
    )

    # Declare the fcu_url argument
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14540@127.0.0.1:14557',
        description='URL for MAVROS FCU connection'
    )

    # Declare the pattern argument
    pattern_arg = DeclareLaunchArgument(
        'pattern',
        default_value='square',
        description='Movement pattern to execute (e.g., square)'
    )

    # Define the path to the mavrospy executable
    mavrospy_executable = [LaunchConfiguration('pattern'), TextSubstitution(text="_py")]

    # Path to the px4.launch file in MAVROS and PX4-Autopilot
    px4_launch_path = os.path.expanduser('~/ros2_ws/install/mavros/share/mavros/launch/px4.launch')

    # Launch px4.launch with the fcu_url argument
    # 2. MAVROS 런치 파일에 use_sim_time 전달
    mavros_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(px4_launch_path),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Launch mavrospy node
    # 3. 제어 노드에도 use_sim_time 파라미터 전달
    mavrospy_node = Node(
        package='mavrospy',
        executable=mavrospy_executable,
        name='control_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Build the launch description
    return LaunchDescription([
        use_sim_time_arg, # 인자 등록
        fcu_url_arg,
        pattern_arg,
        mavros_node,
        mavrospy_node
    ])
