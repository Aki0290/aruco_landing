import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import cv2
import numpy as np
import cv_bridge
import cv2.aruco as aruco
import asyncio
import math
import time
from enum import Enum

# ROS 2のメッセージ型とサービス型をインポート
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

# ミッションの状態を定義するEnum
class MissionState(Enum):
    IDLE = 0
    TAKEOFF_SEQUENCE = 1
    SEARCHING = 2
    CENTERING = 3
    LANDING = 4
    MISSION_COMPLETE = 5

class ArucoLandingNode(Node):
    def __init__(self):
        super().__init__('aruco_landing_node')

        # --- ミッション関連のパラメータ ---
        self.landing_marker_id = 102 # 着陸目標のマーカーID
        self.marker_length = 0.15      # マーカーの1辺の長さ (m)

        # --- 状態管理 ---
        self.mission_state = MissionState.IDLE
        self.takeoff_position = None
        self.last_marker_pose = None # マーカーを見失った場合に備えて最後の位置を記憶

        # --- 探索ロジック用パラメータ ---
        self.search_timer = None
        self.search_radius = 1.0  # 探索開始半径 (m)
        self.search_height = 2.5  # 探索高度 (m)
        self.max_search_radius = 3.5 # 最大探索半径 (m)
        self.search_angle = 0.0
        self.search_radius_step = 0.3 # 1周ごとの半径増加量
        self.search_angle_step = 0.4  # 角度の増加量 (rad)

        # --- ROS 2のセットアップ ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.bridge = cv_bridge.CvBridge()
        self.current_state = None
        self.current_pose = None
        
        # ArUcoのセットアップ
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()
        # あなたのシミュレーションカメラに合わせた値 (要調整)
        self.camera_matrix = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros(5, dtype=np.float32)
        
        # サブスクライバー
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, qos_profile)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, qos_profile)

        # パブリッシャー
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # サービス・クライアント
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

        self.get_logger().info('Aruco Landing Node has been started.')

    # --- コールバック関数 ---
    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def image_callback(self, msg):
        if self.current_pose is None or self.current_state is None or not self.current_state.armed:
            return

        # 探索中またはセンタリング中のみ画像処理を実行
        if self.mission_state not in [MissionState.SEARCHING, MissionState.CENTERING]:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            tvec, detected_id = self.detect_aruco(frame)

            if detected_id == self.landing_marker_id:
                # 探索中にマーカーを発見
                if self.mission_state == MissionState.SEARCHING:
                    self.get_logger().info(f'Landing marker {self.landing_marker_id} found!')
                    self.stop_search()
                    self.mission_state = MissionState.CENTERING

                # マーカー上空へ移動
                if self.mission_state == MissionState.CENTERING:
                    self.center_over_marker(tvec)
            
            elif self.mission_state == MissionState.CENTERING:
                # センタリング中にマーカーを見失った場合
                self.get_logger().warn('Lost the marker. Going back to search mode.')
                self.mission_state = MissionState.SEARCHING
                self.start_search()

        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f'CV Bridge Error: {e}')

    # --- ArUco検出 ---
    def detect_aruco(self, frame):
        """フレームからArUcoマーカーを検出し、IDと相対位置ベクトルを返す"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None and len(ids) > 0:
            # 複数のマーカーが検出されても、最初のものだけを処理する
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )
            return tvecs[0][0], ids[0][0]
        
        return None, -1 # 見つからなかった場合

    # --- ミッション制御ロジック ---
    def center_over_marker(self, tvec):
        """マーカーがカメラの中心に来るようにドローンを制御し、真上に来たら着陸を開始する"""
        dx, dy = tvec[0], tvec[1]
        centering_tolerance = 0.05  # 中心の許容誤差 (m)

        if abs(dx) < centering_tolerance and abs(dy) < centering_tolerance:
            self.get_logger().info("Marker centered. Landing now.")
            self.mission_state = MissionState.LANDING
            asyncio.ensure_future(self.land_drone())
        else:
            # カメラ座標系から機体座標系への変換と目標位置の設定
            # 下向きカメラの場合: カメラX -> 機体Y, カメラY -> 機体-X (PX4のNED座標系を想定)
            target_x = self.current_pose.pose.position.x - dy
            target_y = self.current_pose.pose.position.y + dx
            # 高度は現在の高さを維持
            target_z = self.current_pose.pose.position.z 
            
            self.get_logger().info(f"Centering... Moving to x:{target_x:.2f}, y:{target_y:.2f}")

            target_pose = PoseStamped()
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.header.frame_id = 'map'
            target_pose.pose.position.x = target_x
            target_pose.pose.position.y = target_y
            target_pose.pose.position.z = target_z
            target_pose.pose.orientation = self.current_pose.pose.orientation
            
            self.setpoint_pub.publish(target_pose)
            self.last_marker_pose = target_pose

    def start_search(self):
        """螺旋探索を開始する"""
        if self.search_timer is None or self.search_timer.cancelled():
            self.get_logger().info("Starting search pattern...")
            self.search_timer = self.create_timer(0.2, self.execute_search_pattern)

    def stop_search(self):
        """探索を停止する"""
        if self.search_timer is not None:
            self.search_timer.cancel()
            self.search_timer = None
            self.get_logger().info("Search pattern stopped.")

    def execute_search_pattern(self):
        """螺旋状に飛行して探索するパターンを実行"""
        if self.takeoff_position is None:
            return

        # 離陸地点を中心に螺旋を描く
        # X = X_takeoff + R * cos(theta)
        # Y = Y_takeoff + R * sin(theta)
        x = self.takeoff_position.pose.position.x + self.search_radius * math.cos(self.search_angle)
        y = self.takeoff_position.pose.position.y + self.search_radius * math.sin(self.search_angle)

        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = 'map'
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = self.search_height
        target_pose.pose.orientation = self.current_pose.pose.orientation # 機首方向は一定

        self.setpoint_pub.publish(target_pose)

        # 次の探索ポイントの計算
        self.search_angle += self.search_angle_step
        if self.search_angle >= 2 * math.pi:
            self.search_angle = 0.0
            self.search_radius += self.search_radius_step
            if self.search_radius > self.max_search_radius:
                self.search_radius = 1.0 # 半径をリセットして再探索
                self.get_logger().warn("Max search radius reached. Restarting search from smaller radius.")

    # --- 非同期サービス呼び出し ---
    async def call_service(self, client, request):
        """非同期でサービスを呼び出すヘルパー関数"""
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {client.srv_name} not available')
            return None
        future = client.call_async(request)
        await future
        return future.result()

    async def land_drone(self):
        """着陸サービスを呼び出し、ミッションを完了する"""
        self.stop_search()
        req = CommandTOL.Request()
        # オプション: 着陸地点の緯度経度や高度を指定することも可能
        # req.latitude = ...
        # req.longitude = ...
        # req.altitude = ...
        await self.call_service(self.land_client, req)
        
        self.mission_state = MissionState.MISSION_COMPLETE
        self.get_logger().info("Mission complete. Shutting down in 5 seconds.")
        await asyncio.sleep(5)
        self.destroy_node()
        rclpy.shutdown()

    async def run_mission(self):
        """離陸から探索開始までの初期シーケンスを実行する"""
        self.mission_state = MissionState.TAKEOFF_SEQUENCE
        while self.current_state is None or self.current_pose is None:
            self.get_logger().info("Waiting for drone state and pose...")
            await asyncio.sleep(1)

        self.get_logger().info("Drone is ready. Starting mission sequence.")
        
        # OFFBOARDモードに移行するために、setpointのストリーミングを開始
        offboard_setpoint = PoseStamped()
        offboard_setpoint.header.frame_id = "map"
        offboard_setpoint.pose.position.z = self.search_height
        
        for i in range(100): # 接続を安定させるために100回ほど送信
            offboard_setpoint.header.stamp = self.get_clock().now().to_msg()
            offboard_setpoint.pose.position.x = self.current_pose.pose.position.x
            offboard_setpoint.pose.position.y = self.current_pose.pose.position.y
            self.setpoint_pub.publish(offboard_setpoint)
            await asyncio.sleep(0.02)
        
        # 1. モードをGUIDEDに設定
        set_mode_req = SetMode.Request()
        set_mode_req.custom_mode = 'GUIDED'
        await self.call_service(self.set_mode_client, set_mode_req)
        await asyncio.sleep(0.5)

        # 2. アーム
        arm_req = CommandBool.Request()
        arm_req.value = True
        await self.call_service(self.arming_client, arm_req)
        await asyncio.sleep(2) # アーム完了を待つ

        # 3. 離陸
        self.get_logger().info(f"Taking off to {self.search_height}m...")
        start_time = time.time()
        while time.time() - start_time < 15: # タイムアウトを15秒に設定
            offboard_setpoint.header.stamp = self.get_clock().now().to_msg()
            offboard_setpoint.pose.position.x = self.current_pose.pose.position.x
            offboard_setpoint.pose.position.y = self.current_pose.pose.position.y
            self.setpoint_pub.publish(offboard_setpoint)
            
            # 目標高度に到達したかチェック
            if abs(self.current_pose.pose.position.z - self.search_height) < 0.2:
                self.get_logger().info("Takeoff complete.")
                break
            await asyncio.sleep(0.1)
        
        # 離陸地点を保存
        self.takeoff_position = self.current_pose
        self.get_logger().info(f"Takeoff position saved: x={self.takeoff_position.pose.position.x:.2f}, y={self.takeoff_position.pose.position.y:.2f}")

        # 4. 探索開始
        self.mission_state = MissionState.SEARCHING
        self.start_search()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLandingNode()

    # 非同期のメインループを作成
    loop = asyncio.get_event_loop()
    
    # run_missionを非同期タスクとして実行
    mission_task = loop.create_task(node.run_mission())
    
    # rclpy.spinを別スレッドで実行してコールバックを処理
    spin_thread = asyncio.to_thread(rclpy.spin, node)

    # 両方のタスクが完了するまで待機
    loop.run_until_complete(asyncio.gather(mission_task, spin_thread))

if __name__ == '__main__':
    main()