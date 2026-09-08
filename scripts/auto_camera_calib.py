import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
import yaml

class CameraCalibrator(Node):
    def __init__(self):
        super().__init__('camera_calibrator')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        self.cols = 9
        self.rows = 6
        self.pattern_size = (self.cols, self.rows)
        self.target_count = 20
        
        self.img_dir = '/root/zenith_ws/calib_images'
        self.vis_dir = '/root/zenith_ws/calib_annotated'
        os.makedirs(self.img_dir, exist_ok=True)
        os.makedirs(self.vis_dir, exist_ok=True)
        
        self.objp = np.zeros((self.rows * self.cols, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2) # Unit square
        
        self.objpoints = []
        self.imgpoints = []
        self.captured_images = []
        
        self.last_capture_time = 0
        self.last_heartbeat = 0
        self.min_interval = 0.8 # seconds between captures
        self.gray_shape = None
        
        self.get_logger().info("==========================================================")
        self.get_logger().info("   Zenith Drone: Camera Intrinsic Auto-Calibrator")
        self.get_logger().info("==========================================================")
        self.get_logger().info("Aim camera at the 9x6 Chessboard on your monitor.")
        self.get_logger().info("Slowly tilt and move the camera (center, angles, edges).")
        self.get_logger().info(f"Target: {self.target_count} valid chessboard frames.")
        self.get_logger().info("==========================================================")

    def image_callback(self, msg):
        if len(self.imgpoints) >= self.target_count:
            return
        
        now = time.time()
        
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        self.gray_shape = gray.shape[::-1] # (width, height)
        
        find_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, flags=find_flags)
        
        if not ret:
            if now - self.last_heartbeat > 2.0:
                print(f"[대기중] 카메라 작동 중 ({self.gray_shape[0]}x{self.gray_shape[1]}). 모니터 체커보드를 화면에 비춰주세요... (현재 수집: {len(self.imgpoints)}/{self.target_count})")
                self.last_heartbeat = now
            return

        if now - self.last_capture_time < self.min_interval:
            return
        
        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
            corners_sub = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            self.objpoints.append(self.objp.copy())
            self.imgpoints.append(corners_sub)
            self.last_capture_time = now
            
            idx = len(self.imgpoints)
            raw_path = os.path.join(self.img_dir, f"frame_{idx:02d}.jpg")
            cv2.imwrite(raw_path, cv_img)
            
            vis_img = cv_img.copy()
            cv2.drawChessboardCorners(vis_img, self.pattern_size, corners_sub, ret)
            vis_path = os.path.join(self.vis_dir, f"vis_{idx:02d}.jpg")
            cv2.imwrite(vis_path, vis_img)
            
            print(f"📸 [{idx:02d}/{self.target_count:02d}] 캡처 성공! (카메라를 다른 각도로 살짝 기울여주세요)", flush=True)
            
            if idx >= self.target_count:
                print(f"\n🎉 [완료!] 총 {self.target_count}장 수집 완료! 렌즈 왜곡 및 초점거리 계산 시작...", flush=True)
                self.run_calibration()
                rclpy.shutdown()

    def run_calibration(self):
        print("\n==========================================================")
        print("          Computing Camera Intrinsics & Distortion...     ")
        print("==========================================================")
        
        rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, self.gray_shape, None, None
        )
        
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        d = dist.ravel()
        d0, d1, d2, d3 = float(d[0]), float(d[1]), float(d[2]), float(d[3])
        
        print(f"RMS Reprojection Error: {rms:.4f} pixels (Ideal is < 0.5 px)")
        print(f"Image Resolution:      {self.gray_shape[0]} x {self.gray_shape[1]}")
        print(f"Focal Length (fx, fy): {fx:.3f}, {fy:.3f}")
        print(f"Principal Pt (cx, cy): {cx:.3f}, {cy:.3f}")
        print(f"Distortion Coefficients (k1, k2, p1, p2):")
        print(f"  k1 (cam_d0): {d0:.6f}")
        print(f"  k2 (cam_d1): {d1:.6f}")
        print(f"  p1 (cam_d2): {d2:.6f}")
        print(f"  p2 (cam_d3): {d3:.6f}")
        print("==========================================================")
        
        # Write to zenith_camera_pinhole.yaml
        pinhole_yaml_path = '/root/zenith_ws/src/FAST-LIVO2/config/zenith_camera_pinhole.yaml'
        pinhole_cfg = {
            '/laserMapping': {
                'ros__parameters': {
                    'camera': {
                        'cam_model': 'Pinhole',
                        'cam_width': int(self.gray_shape[0]),
                        'cam_height': int(self.gray_shape[1]),
                        'scale': 1.0,
                        'cam_fx': round(float(fx), 4),
                        'cam_fy': round(float(fy), 4),
                        'cam_cx': round(float(cx), 4),
                        'cam_cy': round(float(cy), 4),
                        'cam_d0': round(d0, 6),
                        'cam_d1': round(d1, 6),
                        'cam_d2': round(d2, 6),
                        'cam_d3': round(d3, 6)
                    }
                }
            }
        }
        with open(pinhole_yaml_path, 'w') as f:
            yaml.dump(pinhole_cfg, f, default_flow_style=False)
            
        print(f"\n>>> [UPDATED] {pinhole_yaml_path} has been successfully updated with new calibration! <<<")
        print("Annotated corner detection images saved in: calib_annotated/")

def main():
    rclpy.init()
    node = CameraCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
