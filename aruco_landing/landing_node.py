# landing_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import cv2
import numpy as np
import cv_bridge
import cv2.aruco as aruco
import time
import asyncio

# ROS 2のメッセージ型とサービス型をインポート
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

class ArucoLandingNode(Node):
    def __init__(self):
        super().__init__('aruco_landing_node')

        # QoSプロファイルの設定 (信頼性を高める)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ArUco関連の初期化
        self.bridge = cv_bridge.CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()
        # あなたのシミュレーションカメラに合わせた値 (要調整)
        self.camera_matrix = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros(5, dtype=np.float32)
        self.marker_length = 0.15  # meters

        # ドローンの状態を保持する変数
        self.current_state = None
        self.current_pose = None
        
        # サブスクライバー (ドローンやカメラからの情報を受け取る)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)
        # Gazeboのカメラトピックに合わせて変更する必要があるかもしれない
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, qos_profile)

        # パブリッシャー (ドローンに目標位置を送る)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # サービス・クライアント (ドローンにコマンドを送る)
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

        self.get_logger().info('Aruco Landing Node has been started.')

    # --- コールバック関数 (データ受信時に呼び出される) ---
    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def image_callback(self, msg):
        if self.current_pose is None or self.current_state is None or not self.current_state.armed:
            return # ドローンが準備できるまで何もしない

        try:
            # ROSの画像メッセージをOpenCVの画像形式に変換
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            tvec = self.detect_aruco(frame)

            if tvec is not None:
                # マーカーを検出した場合の処理
                self.get_logger().info(f"Marker found at tvec: {tvec}")
                self.center_and_land(tvec)
            else:
                self.get_logger().info('Marker not found. Holding position...')
        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f'CV Bridge Error: {e}')

    # --- メインのロジック ---
    def detect_aruco(self, frame):
        """OpenCVのフレームからArUcoマーカーを検出する"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is not None and len(ids) > 0:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            return tvecs[0][0]
        return None

    def center_and_land(self, tvec):
        """マーカーの上空に移動して着陸するロジック"""
        # カメラ座標系からドローンの目標座標系へ変換
        # 下向きカメラの場合: カメラX -> 機体Y, カメラY -> 機体X
        dx = tvec[0]
        dy = tvec[1]
        dz = tvec[2] # マーカーまでの距離

        # 現在位置を基準に目標位置を設定
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = 'map' # 通常はmapやodomフレーム
        
        # 目標は、マーカーの真上、現在の高さ
        target_pose.pose.position.x = self.current_pose.pose.position.x + dy
        target_pose.pose.position.y = self.current_pose.pose.position.y + dx
        target_pose.pose.position.z = self.current_pose.pose.position.z
        target_pose.pose.orientation = self.current_pose.pose.orientation

        # 真上に来るまで移動
        if abs(dx) > 0.1 or abs(dy) > 0.1:
            self.get_logger().info(f"Centering... Moving to x:{target_pose.pose.position.x:.2f}, y:{target_pose.pose.position.y:.2f}")
            self.setpoint_pub.publish(target_pose)
        else:
            # 真上に到達したら着陸
            self.get_logger().info("Marker centered. Landing now.")
            # 着陸サービスを呼び出す (非同期で)
            asyncio.ensure_future(self.call_land_service())
            self.destroy_node() # ノードを終了
            rclpy.shutdown()

    async def run_initial_mission(self):
        """離陸までの初期ミッション"""
        while self.current_state is None or self.current_pose is None:
            self.get_logger().info("Waiting for drone state and pose...")
            await asyncio.sleep(1)

        self.get_logger().info("Drone is ready. Starting mission.")
        
        # 継続的にオフボード位置指令を送るための準備
        # MAVROSがOFFBOARDモードに移行するには、setpointのストリーミングが必要
        offboard_setpoint = PoseStamped()
        offboard_setpoint.header.frame_id = "map"
        for i in range(100): # 100回ほど送って接続を安定させる
            offboard_setpoint.header.stamp = self.get_clock().now().to_msg()
            offboard_setpoint.pose.position.x = self.current_pose.pose.position.x
            offboard_setpoint.pose.position.y = self.current_pose.pose.position.y
            offboard_setpoint.pose.position.z = 2.5 # 目標高度
            self.setpoint_pub.publish(offboard_setpoint)
            await asyncio.sleep(0.02)

        # 1. モードをOFFBOARDに設定
        set_mode_req = SetMode.Request()
        set_mode_req.custom_mode = 'OFFBOARD'
        await self.call_service(self.set_mode_client, set_mode_req)
        await asyncio.sleep(1)

        # 2. アーム
        arm_req = CommandBool.Request()
        arm_req.value = True
        await self.call_service(self.arming_client, arm_req)

        self.get_logger().info("Takeoff sequence completed. Now searching for marker.")
        # この後はimage_callbackがメインループの役割を果たす

    async def call_land_service(self):
        req = CommandTOL.Request()
        await self.call_service(self.land_client, req)

    async def call_service(self, client, request):
        """非同期でサービスを呼び出すヘルパー"""
        await client.wait_for_service()
        future = client.call_async(request)
        await future
        return future.result()

# ... （クラス定義は同じ） ...

def main_sync(args=None):
    asyncio.run(main_async(args))

async def main_async(args=None):
    rclpy.init(args=args)
    node = ArucoLandingNode()

    # 離陸ミッションの実行
    await node.run_initial_mission()

    # rclpy.spinでコールバックを処理し続ける
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main_sync()