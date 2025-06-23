# coding: utf-8
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading
import time
import math
from enum import Enum

import cv2
import numpy as np
import cv_bridge
import cv2.aruco as aruco

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

# ミッションの状態を明確に管理するためのEnum
class MissionState(Enum):
    WAITING_FOR_CONNECTION = 0
    SETTING_MODE = 1
    ARMING = 2
    TAKING_OFF = 3
    SEARCHING = 4
    CENTERING = 5
    LANDING = 6
    MISSION_COMPLETE = 7

class ArucoLandingNode(Node):
    def __init__(self):
        super().__init__('aruco_landing_node')

        # --- パラメータ定義 ---
        self.landing_marker_id = 102 # あなたの指定したID
        self.marker_length = 0.15
        self.search_height = 1.0
        self.centering_tolerance = 0.1

        # --- 状態管理変数 ---
        self.mission_state = MissionState.WAITING_FOR_CONNECTION
        self.current_state = None
        self.current_pose = None
        self.takeoff_position = None
        
        # --- 探索ロジック用変数 ---
        self.search_radius = 0.5
        self.max_search_radius = 3.2
        self.search_angle = 0.0
        self.search_radius_step = 0.3
        self.search_angle_step = math.radians(1)

        # --- ROS 2セットアップ ---
        mavros_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, mavros_qos)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, mavros_qos)
        self.image_sub = self.create_subscription(Image, '/camera/image', self.image_callback, mavros_qos)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', mavros_qos)
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        
        # --- OpenCVセットアップ ---
        self.bridge = cv_bridge.CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters_create()
        self.camera_matrix = np.array([[554.25, 0.0, 320.5],[0.0, 554.25, 240.5],[0.0, 0.0, 1.0]])
        self.dist_coeffs = np.zeros(5, dtype=np.float32)

        # --- 司令塔となる制御ループタイマー (10Hz) ---
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Aruco Landing Node started. Waiting for MAVROS connection...')

    # --- コールバック関数 (外部からのイベントを受け取るだけ) ---
    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg
    
    def image_callback(self, msg):
        if self.mission_state not in [MissionState.SEARCHING, MissionState.CENTERING]: return
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            tvec, detected_id = self.detect_aruco(frame)
            if detected_id == self.landing_marker_id:
                if self.mission_state == MissionState.SEARCHING:
                    self.get_logger().info(f'Landing marker {self.landing_marker_id} found! Switching to CENTERING mode.')
                    self.mission_state = MissionState.CENTERING
                self.center_over_marker(tvec)
        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f'CV Bridge Error: {e}')

    # --- 司令塔：制御ループ ---
    def control_loop(self):
        if not self.current_state or not self.current_pose:
            return

        if self.mission_state == MissionState.WAITING_FOR_CONNECTION:
            if self.current_state.connected:
                self.get_logger().info("MAVROS Connected. Proceeding to set mode.")
                self.mission_state = MissionState.SETTING_MODE
                self.set_mode_client.call_async(SetMode.Request(custom_mode='GUIDED'))

        elif self.mission_state == MissionState.SETTING_MODE:
            if self.current_state.mode == 'GUIDED':
                self.get_logger().info("Mode is now GUIDED. Proceeding to arm.")
                self.mission_state = MissionState.ARMING
                self.arming_client.call_async(CommandBool.Request(value=True))

        elif self.mission_state == MissionState.ARMING:
            if self.current_state.armed:
                self.get_logger().info("Vehicle is armed. Proceeding to takeoff.")
                self.mission_state = MissionState.TAKING_OFF
                self.takeoff_position = self.current_pose
                self.takeoff_client.call_async(CommandTOL.Request(altitude=self.search_height, latitude=float('nan'), longitude=float('nan')))

        elif self.mission_state == MissionState.TAKING_OFF:
            if abs(self.current_pose.pose.position.z - self.search_height) < 0.5:
                self.get_logger().info("Takeoff complete. Switching to SEARCHING mode.")
                self.mission_state = MissionState.SEARCHING

        elif self.mission_state == MissionState.SEARCHING:
            self.execute_search_pattern()

    def detect_aruco(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        # --- デバッグ用のログ表示を追加 ---
        if ids is not None:
            # まず、何かしらのマーカーが見つかった場合に、そのIDをすべて表示する
            # flatten()は、[[102], [3], [50]] のような形を [102, 3, 50] のように見やすくするためのものです
            self.get_logger().info(f'Found ArUco markers with IDs: {ids.flatten()}')

            for i, marker_id in enumerate(ids):
                # 見つかったIDの中に、探しているIDがあるかチェック
                if marker_id[0] == self.landing_marker_id:
                    # 目標のマーカーが見つかったら、そのことを知らせる
                    self.get_logger().info(f'>>> Target marker {self.landing_marker_id} FOUND! <<<')

                    # ポーズ推定を実行
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers([corners[i]], self.marker_length, self.camera_matrix, self.dist_coeffs)
                    
                    # 計算された3D位置ベクトル(tvec)も表示してみる
                    tvec = tvecs[0][0]
                    self.get_logger().info(f'    - Position from camera (tvec): x={tvec[0]:.3f}, y={tvec[1]:.3f}, z={tvec[2]:.3f}')
                    
                    return tvec, ids[i][0]

        # 何も見つからなかった場合はNoneを返す
        return None, -1

    def center_over_marker(self, tvec):
        dx, dy = tvec[0], tvec[1]
        if abs(dx) < self.centering_tolerance and abs(dy) < self.centering_tolerance:
            self.get_logger().info("Marker centered. Requesting LAND mode.")
            self.mission_state = MissionState.LANDING
            self.set_mode_client.call_async(SetMode.Request(custom_mode='LAND'))
        else:
            # --- ここからがデバッグログを追加した部分 ---
            
            # 1. 現在の状態をログに出力
            current_x = self.current_pose.pose.position.x
            current_y = self.current_pose.pose.position.y
            self.get_logger().info(
                f'Centering... '
                f'Error(dx, dy)=({dx:+.2f}, {dy:+.2f}), '
                f'Current(Cx, Cy)=({current_x:.2f}, {current_y:.2f})'
            )

            # 2. 新しい目標位置を計算
            target_pose = PoseStamped()
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.header.frame_id = 'map'
            target_pose.pose.position.x = current_x + dx
            target_pose.pose.position.y = current_y - dy 
            target_pose.pose.position.z = self.search_height
            target_pose.pose.orientation = self.current_pose.pose.orientation
            
            # 3. 計算された目標値をログに出力
            new_target_x = target_pose.pose.position.x
            new_target_y = target_pose.pose.position.y
            self.get_logger().info(
                f'          -> New Target(Tx, Ty)=({new_target_x:.2f}, {new_target_y:.2f}). Publishing...'
            )

            self.setpoint_pub.publish(target_pose)

    def execute_search_pattern(self):
        if not self.takeoff_position: return
        x = self.takeoff_position.pose.position.x + self.search_radius * math.cos(self.search_angle)
        y = self.takeoff_position.pose.position.y + self.search_radius * math.sin(self.search_angle)
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = 'map'
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = self.search_height
        target_pose.pose.orientation = self.current_pose.pose.orientation
        self.setpoint_pub.publish(target_pose)
        self.search_angle += self.search_angle_step
        if self.search_angle >= 2 * math.pi:
            self.search_angle = 0.0
            self.search_radius += self.search_radius_step
            self.get_logger().info(f"Increasing search radius to {self.search_radius:.2f}m")
            if self.search_radius > self.max_search_radius:
                self.get_logger().warn("Max search radius reached. Landing as failsafe.")
                self.mission_state = MissionState.LANDING
                self.set_mode_client.call_async(SetMode.Request(custom_mode='LAND'))

# --- メイン関数 ---
def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArucoLandingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()