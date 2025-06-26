# aruco_landing.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
# Python以外のlaunchファイル(XML, YAML)を読み込むために、AnyLaunchDescriptionSourceを使います
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():

    # --- 引数の定義 ---
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Set to "true" to run in simulation mode.'
    )

    # --- あなたのノードの起動設定 ---
    aruco_landing_node = Node(
        package='aruco_landing',
        executable='landing_node',
        name='aruco_landing_node',
        output='screen'
    )

    # --- MAVROSの起動設定 ---
    # 【シミュレーション用】 'sim'が'true'の時だけ、こちらが実行されます
    mavros_sim_launch = IncludeLaunchDescription(
        # ★★★ ここをAnyLaunchDescriptionSourceに変更 ★★★
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mavros'),
                'launch',
                'apm.launch' # ★★★ .py を削除 ★★★
            ])
        ]),
        launch_arguments={'fcu_url': "udp://127.0.0.1:14550@"}.items(),
        condition=IfCondition(LaunchConfiguration('sim'))
    )

    # 【実機用】 'sim'が'false'（デフォルト）の時だけ、こちらが実行されます
    mavros_real_launch = IncludeLaunchDescription(
        # ★★★ ここもAnyLaunchDescriptionSourceに変更 ★★★
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mavros'),
                'launch',
                'apm.launch' # ★★★ .py を削除 ★★★
            ])
        ]),
        launch_arguments={'fcu_url': "/dev/ttyAMA0:921600"}.items(),
        condition=UnlessCondition(LaunchConfiguration('sim'))
    )

    # --- 全体を組み立てて返す ---
    return LaunchDescription([
        sim_arg,
        aruco_landing_node,
        mavros_sim_launch,
        mavros_real_launch,
    ])