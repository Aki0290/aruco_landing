from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):
    # 'sim'引数の値を取得 ('true'ならシミュレーションモード)
    sim_mode = LaunchConfiguration('sim').perform(context) == 'true'

    # --- MAVROSノードの定義 ---
    if sim_mode:
        # 【修正】14550番ポートにつなぐ設定 (標準的なSITL接続)
        # 意味: 自分のポート14540を開けて、相手の14550にデータを送る
        fcu_url = "udp://:14551@"
    else:
        fcu_url = "/dev/ttyAMA0:921600" # 実機用の設定

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[{'fcu_url': fcu_url}]
    )

    # --- あなたの自作ノードの定義 ---
    aruco_landing_node = Node(
        package='aruco_landing',
        executable='landing_node',
        name='aruco_landing_node',
        output='screen'
    )
    
    # --- Intel RealSenseカメラの起動定義 ---
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py' 
            ])
        ]),
        launch_arguments={
            'pointcloud.enable': 'false',
            'align_depth.enable': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'rgb_camera.profile': '640,480,15',
        }.items()
    )

    # --- 起動するノードのリストを作成 ---
    nodes_to_launch = []
    
    if sim_mode:
        nodes_to_launch.extend([
            mavros_node,
            aruco_landing_node
        ])
    else:
        nodes_to_launch.extend([
            realsense_launch,
            mavros_node,
            aruco_landing_node
        ])

    return nodes_to_launch

def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Set to "true" to run in simulation mode.'
    )

    return LaunchDescription([
        sim_arg,
        OpaqueFunction(function=launch_setup)
    ])
