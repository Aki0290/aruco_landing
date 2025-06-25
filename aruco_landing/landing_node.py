# coding: utf-8
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading
import time
import math
from enum import IntEnum
import os

import cv2
import numpy as np
import cv_bridge
import cv2.aruco as aruco

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

# ミッションの状態を明確に管理するためのEnum
class MissionState(IntEnum):
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
        self.landing_marker_id = 102
        self.marker_length = 0.15
        self.search_height = 0.5
        self.centering_tolerance = 0.1

        # --- 状態管理変数 ---
        self.mission_state = MissionState.WAITING_FOR_CONNECTION
        self.current_state = None
        self.current_pose = None
        self.takeoff_position = None
        
        # --- 探索ロジック用変数 ---
        self.search_radius = 0.5
        self.max_search_radius = 3.5
        self.search_angle = 0.0
        self.search_radius_step = 0.5
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
        self.camera_matrix = np.array([[332.5, 0.0, 320],[0.0, 332.5, 240],[0.0, 0.0, 2.0]])
        self.dist_coeffs = np.zeros(5, dtype=np.float32)

        ### <<< 追加/変更部分 ここから >>> ###

        # --- 黄緑色の物体検知用パラメータ ---
        self.hsv_lower_green = np.array([35, 100, 100])
        self.hsv_upper_green = np.array([85, 255, 255])
        self.min_object_area = 500
        
        # --- 複数物体管理用パラメータ ---
        # 検出する物体の最大数
        self.max_objects_to_detect = 20
        # 新しい物体と既存の物体を区別するための最小距離（メートル）
        self.min_distance_between_objects = 0.5
        # 発見した物体のワールド座標を保存するリスト
        self.detected_objects_positions = []
        
        # 保存するファイル名
        self.object_log_filename = "detected_objects_locations.txt"
        
        # 起動時に以前のログファイルを削除
        if os.path.exists(self.object_log_filename):
            os.remove(self.object_log_filename)
        self.get_logger().info(f"'{self.object_log_filename}' will be created upon object detection.")


        # 発見した物体のワールド座標を保存するリスト（重複検知用）
        self.detected_objects_positions = []
        # 発見した物体のカメラ座標を保存するリスト（ファイル書き出し用）
        self.detected_objects_camera_coords = []
    

        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Aruco Landing Node started. Waiting for MAVROS connection...')

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg
    
    def image_callback(self, msg):
        if self.current_pose is None or self.takeoff_position is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f'CV Bridge Error: {e}')
            return

        # --- ArUcoマーカー着陸ロジック（変更なし） ---
        if self.mission_state in [MissionState.SEARCHING, MissionState.CENTERING]:
            tvec, detected_id = self.detect_aruco(frame)
            if detected_id == self.landing_marker_id:
                if self.mission_state == MissionState.SEARCHING:
                    self.get_logger().info(f'Landing marker {self.landing_marker_id} found! Switching to CENTERING mode.')
                    self.mission_state = MissionState.CENTERING
                self.center_over_marker(tvec)
        
        ### <<< 追加/変更部分 ここから >>> ###
        
        # --- 黄緑色の物体検知と位置保存ロジック ---
        # ミッションが探索段階以降で、かつ、まだ最大数まで発見していない場合のみ実行
        if self.mission_state >= MissionState.SEARCHING and len(self.detected_objects_positions) < self.max_objects_to_detect:
            self.detect_and_manage_objects(frame)
            
        ### <<< 追加/変更部分 ここまで >>> ###

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
            if abs(self.current_pose.pose.position.z - self.search_height) < 0.3:
                self.get_logger().info("Takeoff complete. Switching to SEARCHING mode.")
                self.mission_state = MissionState.SEARCHING

        elif self.mission_state == MissionState.SEARCHING:
            self.execute_search_pattern()

    def detect_aruco(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            self.get_logger().info(f'Found ArUco markers with IDs: {ids.flatten()}', throttle_duration_sec=1.0)
            for i, marker_id in enumerate(ids):
                if marker_id[0] == self.landing_marker_id:
                    self.get_logger().info(f'>>> Target marker {self.landing_marker_id} FOUND! <<<')
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers([corners[i]], self.marker_length, self.camera_matrix, self.dist_coeffs)
                    tvec = tvecs[0][0]
                    self.get_logger().info(f'    - Position from camera (tvec): x={tvec[0]:.3f}, y={tvec[1]:.3f}, z={tvec[2]:.3f}')
                    return tvec, ids[i][0]

        return None, -1

    def center_over_marker(self, tvec):
        dx, dy = tvec[0], tvec[1]
        if abs(dx) < self.centering_tolerance and abs(dy) < self.centering_tolerance:
            self.get_logger().info("Marker centered. Requesting LAND mode.")
            self.mission_state = MissionState.LANDING
            self.set_mode_client.call_async(SetMode.Request(custom_mode='LAND'))
        else:
            current_x = self.current_pose.pose.position.x
            current_y = self.current_pose.pose.position.y
            self.get_logger().info(
                f'Centering... Error(dx, dy)=({dx:+.2f}, {dy:+.2f})', throttle_duration_sec=1.0
            )
            target_pose = PoseStamped()
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.header.frame_id = 'map'
            target_pose.pose.position.x = current_x + dx
            target_pose.pose.position.y = current_y - dy 
            target_pose.pose.position.z = self.search_height
            target_pose.pose.orientation = self.current_pose.pose.orientation
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


    def detect_and_manage_objects(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower_green, self.hsv_upper_green)
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.min_object_area]
        
        if not valid_contours:
            return

        new_object_found = False
        for contour in valid_contours:
            if len(self.detected_objects_positions) >= self.max_objects_to_detect:
                break
            
            M = cv2.moments(contour)
            if M["m00"] == 0: continue
            center_u = int(M["m10"] / M["m00"])
            center_v = int(M["m01"] / M["m00"])

            # 座標変換を実行
            camera_coords, world_coords = self.transform_pixel_to_frames(center_u, center_v)
            
            # 重複検知はワールド座標で行う
            is_new = True
            for saved_pos in self.detected_objects_positions:
                dist = math.sqrt((world_coords[0] - saved_pos[0])**2 + (world_coords[1] - saved_pos[1])**2)
                if dist < self.min_distance_between_objects:
                    is_new = False
                    break
            
            # 新しい物体であれば、ワールド座標とカメラ座標の両方を保存
            if is_new:
                self.get_logger().info(f'>>> New green object DETECTED! Total: {len(self.detected_objects_positions) + 1} <<<')
                self.detected_objects_positions.append(world_coords)
                self.detected_objects_camera_coords.append(camera_coords)
                new_object_found = True

        if new_object_found:
            self.update_log_file()
            if len(self.detected_objects_positions) == self.max_objects_to_detect:
                self.get_logger().info(f'Found all {self.max_objects_to_detect} objects. Stopping search.')
        
    def transform_pixel_to_frames(self, u, v):
        """ピクセル座標(u,v)をカメラ座標とワールド座標に変換する"""
        # --- カメラ座標系での位置 (Xc, Yc, Zc) を計算 ---
        Zc = self.current_pose.pose.position.z
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        Xc = (u - cx) * Zc / fx
        Yc = (v - cy) * Zc / fy
        camera_coords = (Xc, Yc, Zc)

        # --- ワールド座標系での位置 (x, y) を計算 ---
        yaw = self.get_yaw_from_pose(self.current_pose.pose)
        
        # 「カメラ映像の上が機体の前方」と仮定した変換
        dx = -Yc
        dy = -Xc

        world_offset_x = dx * math.cos(yaw) - dy * math.sin(yaw)
        world_offset_y = dx * math.sin(yaw) + dy * math.cos(yaw)

        object_world_x = self.current_pose.pose.position.x + world_offset_x
        object_world_y = self.current_pose.pose.position.y + world_offset_y
        world_coords = (object_world_x, object_world_y)
        
        return camera_coords, world_coords


    def update_log_file(self):
        """発見した全オブジェクトの、離陸地点を原点とするワールド座標をファイルに書き出す"""
        try:
            with open(self.object_log_filename, 'w') as f:
                f.write(f"# Detected Objects: {len(self.detected_objects_positions)} / {self.max_objects_to_detect}\n")
                f.write(f"# Coordinates are in the world frame, relative to the takeoff point (meters).\n\n")

                # 離陸地点の座標を取得
                takeoff_x = self.takeoff_position.pose.position.x
                takeoff_y = self.takeoff_position.pose.position.y

                # 保存されている各オブジェクトの「絶対ワールド座標」を取り出す
                for i, pos in enumerate(self.detected_objects_positions):
                    # pos は (object_world_x, object_world_y) のタプル

                    # 絶対座標から離陸地点の座標を引いて、相対座標を計算
                    relative_x = pos[0] - takeoff_x
                    relative_y = pos[1] - takeoff_y
                    
                    f.write(f"[Object {i+1}]\n")
                    f.write(f"x: {relative_x:.4f}\n")
                    f.write(f"y: {relative_y:.4f}\n\n")
            
            self.get_logger().info(f"Updated object locations in '{self.object_log_filename}'.")
        except IOError as e:
            self.get_logger().error(f"Could not write to file: {e}")
        
    def get_yaw_from_pose(self, pose):
        orientation = pose.orientation
        q_x, q_y, q_z, q_w = orientation.x, orientation.y, orientation.z, orientation.w
        t3 = +2.0 * (q_w * q_z + q_x * q_y)
        t4 = +1.0 - 2.0 * (q_y * q_y + q_z * q_z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoLandingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()