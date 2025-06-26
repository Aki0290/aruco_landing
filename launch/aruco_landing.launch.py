# aruco_landing.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
# RealSenseのPython launchファイルを読み込むために必要
from launch.launch_description_sources import PythonLaunchDescriptionSource

# launchファイルが実行されるときに呼び出される関数
def launch_setup(context, *args, **kwargs):

    # 'sim'引数の値を取得 ('true'ならシミュレーションモード)
    sim_mode = LaunchConfiguration('sim').perform(context) == 'true'

    # --- MAVROSノードの定義 ---
    if sim_mode:
        fcu_url = "udp://:14540@127.0.0.1:14557"
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
                'rs_launch.py' # RealSenseの標準launchファイル
            ])
        ]),
        # RPi4の負荷を軽減するためのパラメータ設定
        launch_arguments={
            'pointcloud.enable': 'false',    # ポイントクラウドは無効
            'align_depth.enable': 'false',   # 深度とカラーの位置合わせは無効
            'enable_infra1': 'false',        # 赤外線1は無効
            'enable_infra2': 'false',        # 赤外線2は無効
            'rgb_camera.profile': '640,480,15', # RGBカメラを640x480, 15fpsに設定
        }.items()
    )

    # --- 起動するノードのリストを作成 ---
    nodes_to_launch = []
    
    if sim_mode:
        # シミュレーションモードでは、MAVROSと自作ノードだけを起動
        nodes_to_launch.extend([
            mavros_node,
            aruco_landing_node
        ])
    else:
        # 実機モードでは、3つすべてを起動
        nodes_to_launch.extend([
            realsense_launch,
            mavros_node,
            aruco_landing_node
        ])

    return nodes_to_launch


# launchファイルのメイン関数
def generate_launch_description():

    # 'sim'という名前の引数を定義
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Set to "true" to run in simulation mode.'
    )

    return LaunchDescription([
        sim_arg,
        # OpaqueFunctionを使って、launch実行時にlaunch_setup関数を呼び出す
        OpaqueFunction(function=launch_setup)
    ])