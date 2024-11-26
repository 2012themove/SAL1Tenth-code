from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

config_dir = get_package_share_directory("gps_nav")

def generate_launch_description():
    hostname = '10.227.132.243:801'
    buffer_size = 200
    topic_namespace = 'vicon'
    return LaunchDescription([
        Node(
            package='vicon_receiver',
            executable='vicon_client',
            output='screen',
            parameters=[{'hostname': hostname, 'buffer_size': buffer_size, 'namespace': topic_namespace}]
        ),
        Node(
            package='gps_nav',
            executable='route_pose_provider',
            name='route_pose_provider',
            parameters = [
                {'want_loop': False},
                {'state_defs': '{0:\'OFF\', 1:\'ON\', 2:\'OUTSIDE\', 3:\'ENTRY_EXTENSION_PT\', 4:\'EXIT_EXTENSION_PT\', 5:\'EXIT_TURN_PT\', 6:\'START\', 7:\'END\', 8:\'UTURN_PT1\', 9:\'UTURN_PT2\', 10:\'UTURN_PT3\', 11:\'CORNER\', 12:\'END_EXTENSION\'}'},
                {'pose_filename': config_dir + '/data/pose_list_car1.txt'}
            ]
        ),
        Node(
            package='gps_nav',
            executable='route_pose_visualizer',
            name='route_pose_visualizer',
            output='screen'
        ),
        Node(
            package='gps_nav',
            executable='goal_pose_creator',
            name='goal_pose_creator',
            output='screen',
            parameters = [
                {'distBetweenPoints': 0.05} # dist between points on path segments in meters
            ]
        ),
        Node(
            package='gps_nav',
            executable='goal_pose_visualizer',
            name='goal_pose_visualizer',
            output='screen'
        ),
        Node(
            package='gps_nav',
            executable='vehicle_controller',
            name='vehicle_controller',
            output='screen',
            parameters = [
                {'L_wheelbase_m': 0.33}  # dist between axles in meters
            ] 
        ),
        
        Node(
            package='gps_nav',
            executable='motion_spec_provider',
            name='motion_spec_provider',
            output='screen',
            parameters = [
                {'look_ahead_dist': 1.5},  # meters
                {'speed': 0.75}  # meters/sec
            ] 
        ),
        Node(
            package='gps_nav',
            executable='pose_creator_vehicle',
            name='pose_creator_vehicle',
            output='screen'
        ),
        Node(
             package='rviz2',
             namespace='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=["-d", config_dir + "/rviz/gps_nav.rviz"]
        )
    ])
