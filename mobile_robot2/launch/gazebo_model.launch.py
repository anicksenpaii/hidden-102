import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    # Define package and file names
    robotXacroName = 'mydifferential_drive_robot'
    namePackage = 'mobile_robot2'
    modelFileRelativePath = 'model/robot2.xacro'
    worldFileRelativePath = 'model/fourthworld.world'
    slamconfigfilepath = 'config/slam_toolbox_params.yaml'
    nav2configfilepath = 'config/nav2_params.yaml'

    # Get paths
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)
    slam_params_path = os.path.join(get_package_share_directory(namePackage), slamconfigfilepath)
    nav2_params_path = os.path.join(get_package_share_directory(namePackage),'config','nav2_params.yaml')
    
    gazebo_model_path = os.path.join(get_package_share_directory(namePackage), 'model')
    set_gazebo_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path)

    # Get RViz config path
    rviz_config_path = os.path.join(get_package_share_directory(namePackage), 
                                   'rviz', 'robot_config.rviz')

    # Process XACRO file
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # Gazebo launch
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )
    
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments={'world': pathWorldFile}.items()
    )

    # nav2_bringup = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
    #     launch_arguments={'use_sim_time': True}.items()
    # )
    
    # Spawn robot
    spawnModelNode = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robotXacroName],
        output='screen'
    )

    # Robot state publisher
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )

    # Add RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Add point cloud to laser scan conversion (optional)
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'body_link',  # Ensure this is correct
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -1.5708,
            'angle_max': 1.5708,
            'angle_increment': 0.0087,
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 100.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'queue_size': 10  # Ensure this is an integer
            }],
        remappings=[
                ('cloud_in', '/velodyne/PointCloud2'),
                ('scan', '/scan')
        ],
        #output='screen',
            # Add log level for more verbosity
        #arguments=['--ros-args', '--log-level', 'debug']
    )
    # SLAM node
    # slam = Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox_node',
    #     output='screen',
    #     parameters=[slam_params_path],
    #     # remappings=[('/scan', 've')]
    # )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox_node',
        output='screen',
        parameters=[{
            # Plugin params
            'solver_plugin': 'solver_plugins::CeresSolver',
            'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
            'ceres_preconditioner': 'SCHUR_JACOBI',
            'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
            'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
            'ceres_loss_function': 'None',

            # ROS Parameters
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'mode': 'mapping',  # or 'localization'

            # Uncomment these if needed
            # 'map_file_name': 'test_steve',
            # 'map_start_pose': [0.0, 0.0, 0.0],
            # 'map_start_at_dock': True,

            'debug_logging': False,
            'throttle_scans': 1,
            'transform_publish_period': 0.01,  # if 0 never publishes odometry
            'map_update_interval': 2.0,
            'resolution': 0.05,
            'max_laser_range': 20.0,  # for rastering images
            'minimum_time_interval': 0.2,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'stack_size_to_use': 40000000,  # program needs a larger stack size to serialize large maps
            'enable_interactive_mode': True,

            # General Parameters
            'use_scan_matching': True,
            'use_scan_barycenter': True,
            'minimum_travel_distance': 0.5,
            'minimum_travel_heading': 0.5,
            'scan_buffer_size': 10,
            'scan_buffer_maximum_scan_distance': 10.0,
            'link_match_minimum_response_fine': 0.1,
            'link_scan_maximum_distance': 1.5,
            'loop_search_maximum_distance': 3.0,
            'do_loop_closing': True,
            'loop_match_minimum_chain_size': 10,
            'loop_match_maximum_variance_coarse': 3.0,
            'loop_match_minimum_response_coarse': 0.35,
            'loop_match_minimum_response_fine': 0.45,

            # Correlation Parameters
            'correlation_search_space_dimension': 0.5,
            'correlation_search_space_resolution': 0.01,
            'correlation_search_space_smear_deviation': 0.1,

            # Loop Closure Parameters
            'loop_search_space_dimension': 8.0,
            'loop_search_space_resolution': 0.05,
            'loop_search_space_smear_deviation': 0.03,

            # Scan Matcher Parameters
            'distance_variance_penalty': 0.5,
            'angle_variance_penalty': 1.0,
            'fine_search_angle_offset': 0.00349,
            'coarse_search_angle_offset': 0.349,
            'coarse_angle_resolution': 0.0349,
            'minimum_angle_penalty': 0.9,
            'minimum_distance_penalty': 0.5,
            'use_response_expansion': True,
            'use_sim_time' : True
        }],
    )
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
    #     ),
    #     launch_arguments={
    #         'use_sim_time': 'True',
    #         'params_file' : nav2_params_path,
    #         'map' : ''          
    #     }.items()
    # )

    # Nav2 bringup node
    # nav2_bringup = Node(
    #     package='nav2_bringup',
    #     executable='navigation_launch.py',
    #     name='nav2_bringup',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': True,
    #         'autostart': True,
    #         'map_subscribe_transient_local': True,  # Ensure the map is correctly subscribed
    #         'amcl': {
    #             'use_sim_time': True,
    #             'max_particles': 2000,
    #             'min_particles': 500,
    #             'kld_err': 0.05,
    #             'kld_z': 0.99,
    #             'alpha1': 0.2,
    #             'alpha2': 0.2,
    #             'alpha3': 0.2,
    #             'alpha4': 0.2,
    #             'alpha5': 0.2,
    #             'odom_frame_id': 'odom',
    #             'base_frame_id': 'base_link',
    #             'global_frame_id': 'map',
    #             'laser_model_type': 'likelihood_field',
    #             'min_particle_weight': 0.001,
    #             'resample_interval': 1,
    #             'transform_tolerance': 0.1,
    #             'recovery_alpha_slow': 0.001,
    #             'recovery_alpha_fast': 0.1,
    #             # Add other AMCL parameters as needed
    #         },
    #         'map_server': {
    #             'use_sim_time': True,
    #             'yaml_filename': '',  # Ensure this is empty or set correctly for initial loading
    #             'frame_id': 'map',
    #             'map_topic': 'map',
    #             # Add other map server parameters as needed
    #         },
    #         'planner_server': {
    #             'expected_planner_frequency': 20.0,
    #             'planner_plugins': ['GridBased'],
    #             'GridBased': {
    #                 'plugin': 'nav2_navfn_planner/NavfnPlanner',
    #                 'tolerance': 0.5,
    #                 'use_astar': False,
    #                 'allow_unknown': True,
    #                 # Add other planner server parameters as needed
    #             }
    #             # Add other planner server parameters as needed
    #         },
    #         'controller_server': {
    #             'expected_controller_frequency': 20.0,
    #             'controller_plugins': ['FollowPath'],
    #             'FollowPath': {
    #                 'plugin': 'dwb_core::DWBLocalPlanner',
    #                 'debug_trajectory_details': True,
    #                 'min_vel_x': 0.0,
    #                 'min_vel_y': 0.0,
    #                 'max_vel_x': 0.5,
    #                 'max_vel_y': 0.0,
    #                 'min_speed_xy': 0.0,
    #                 'max_speed_xy': 0.5,
    #                 'acc_lim_x': 2.5,
    #                 'acc_lim_y': 0.0,
    #                 'acc_lim_theta': 3.2,
    #                 'xy_goal_tolerance': 0.25,
    #                 'yaw_goal_tolerance': 0.25,
    #                 'stateful': False,
    #                 # Add other controller server parameters as needed
    #             }
    #             # Add other controller server parameters as needed
    #         }
    #     }]
    # )

    nav2_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='nav2_costmap',
        output='screen',
        parameters=[{
            'use_sim_time' : True,
        }]
    )
    # Synchroniser node
    # synchroniser = Node(
    #     package='mobile_robot2',
    #     executable='synchroniser.py',  # Ensure this matches the entry point in setup.py for your package
    #     name='synchroniser',
    #     #output='screen'
    # )

    # Create and return launch description
    ld = LaunchDescription()
    ld.add_action(set_gazebo_model_path)
    ld.add_action(gazeboLaunch)
    ld.add_action(spawnModelNode)
    ld.add_action(nodeRobotStatePublisher)
    ld.add_action(rviz_node)
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(slam)
    ld.add_action(nav2_costmap)
    #ld.add_action(synchroniser)
    
    return ld
