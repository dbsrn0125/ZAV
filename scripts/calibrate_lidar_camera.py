#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Zenith Aerial Vehicle (ZAV) - LiDAR-Camera Extrinsic Calibration Tool
Solves for Rcl (3x3 rotation matrix) and Pcl (3x1 translation vector) using
2D-3D point correspondence via OpenCV solvePnP and Open3D point picking.
"""

import os
import sys
import time
import yaml
import numpy as np
import cv2

def load_camera_intrinsics(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    
    params = data.get('/laserMapping', {}).get('ros__parameters', {}).get('camera', {})
    if 'cam_fx' in params:
        fx = float(params['cam_fx'])
        fy = float(params['cam_fy'])
        cx = float(params['cam_cx'])
        cy = float(params['cam_cy'])
        d0 = float(params.get('cam_d0', 0.0))
        d1 = float(params.get('cam_d1', 0.0))
        d2 = float(params.get('cam_d2', 0.0))
        d3 = float(params.get('cam_d3', 0.0))
        K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.array([d0, d1, d2, d3, 0.0], dtype=np.float64)
        return K, dist
    else:
        raise ValueError(f"Could not parse camera intrinsics from {yaml_path}")

def pick_image_points(img_path):
    img = cv2.imread(img_path)
    if img is None:
        raise FileNotFoundError(f"Cannot load image: {img_path}")
    
    clone = img.copy()
    points = []
    
    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            points.append((x, y))
            idx = len(points)
            cv2.circle(clone, (x, y), 4, (0, 0, 255), -1)
            cv2.circle(clone, (x, y), 7, (0, 255, 255), 1)
            cv2.putText(clone, f"#{idx}", (x + 8, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("1. Camera Image - Click feature points", clone)
            print(f"  [Image] Point #{idx} clicked at (x={x}, y={y})")
    
    win_name = "1. Camera Image - Click feature points"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 960, 768)
    cv2.imshow(win_name, clone)
    cv2.setMouseCallback(win_name, on_mouse)
    
    print("\n=======================================================")
    print(" [Step 1] Pick Points on Camera Image")
    print(" - Left Click: Add point")
    print(" - 'u' or 'z': Undo last point")
    print(" - 'r': Reset all points")
    print(" - [Space] or [Enter]: Confirm and proceed (min 4 points)")
    print("=======================================================")
    
    while True:
        key = cv2.waitKey(20) & 0xFF
        if key in [13, 32]: # Enter or Space
            if len(points) < 4:
                print(f"[WARN] Minimum 4 points required for PnP! (Currently: {len(points)})")
            else:
                break
        elif key in [ord('u'), ord('z')]:
            if len(points) > 0:
                points.pop()
                clone = img.copy()
                for i, (px, py) in enumerate(points):
                    cv2.circle(clone, (px, py), 4, (0, 0, 255), -1)
                    cv2.circle(clone, (px, py), 7, (0, 255, 255), 1)
                    cv2.putText(clone, f"#{i+1}", (px + 8, py - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow(win_name, clone)
                print(f"  [Image] Undone. Points remaining: {len(points)}")
        elif key == ord('r'):
            points.clear()
            clone = img.copy()
            cv2.imshow(win_name, clone)
            print("  [Image] Reset all points.")
        elif key == 27: # ESC
            print("[ABORT] User cancelled.")
            sys.exit(0)
            
    cv2.destroyWindow(win_name)
    return np.array(points, dtype=np.float64)

def pick_pointcloud_points(pcd_path, target_num):
    import open3d as o3d
    pcd = o3d.io.read_point_cloud(pcd_path)
    if pcd.is_empty():
        raise ValueError(f"Point cloud is empty: {pcd_path}")
    
    print("\n=======================================================")
    print(f" [Step 2] Pick {target_num} Points on LiDAR 3D Point Cloud")
    print(f" - Hold [Shift] + Left Click on points in the EXACT same order (#1 to #{target_num})")
    print(" - Left Drag: Rotate 3D view")
    print(" - Right Drag: Pan view")
    print(" - Scroll: Zoom in/out")
    print(" - Press 'Q' when finished picking points.")
    print("=======================================================")
    
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name=f"2. LiDAR 3D - Shift+Click {target_num} points in order (Press Q when done)", width=1024, height=768)
    vis.add_geometry(pcd)
    vis.run()
    picked_indices = vis.get_picked_points()
    vis.destroy_window()
    
    pts_all = np.asarray(pcd.points)
    if len(picked_indices) != target_num:
        print(f"\n[WARN] Picked {len(picked_indices)} points in LiDAR, but had {target_num} points in image!")
        if len(picked_indices) < 4:
            raise ValueError("Need at least 4 points to compute extrinsic calibration.")
        picked_indices = picked_indices[:target_num]
        
    picked_3d = pts_all[picked_indices]
    for i, p in enumerate(picked_3d):
        print(f"  [LiDAR] Point #{i+1}: X={p[0]:.3f}m, Y={p[1]:.3f}m, Z={p[2]:.3f}m")
        
    return picked_3d, pts_all

def compute_extrinsic(obj_pts, img_pts, K, dist):
    success, rvec, tvec = cv2.solvePnP(
        obj_pts, img_pts, K, dist,
        flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not success:
        raise RuntimeError("cv2.solvePnP failed to converge.")
    
    Rcl, _ = cv2.Rodrigues(rvec)
    Pcl = tvec.flatten()
    
    # Reprojection error
    proj_pts, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
    proj_pts = proj_pts.reshape(-1, 2)
    errors = np.linalg.norm(img_pts - proj_pts, axis=1)
    mean_err = np.mean(errors)
    
    return Rcl, Pcl, rvec, tvec, mean_err, errors

def project_and_visualize(img_path, pts_lidar, K, dist, rvec, tvec, save_path):
    img = cv2.imread(img_path)
    h, w = img.shape[:2]
    
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3, 1)
    pts_cam = (R @ pts_lidar.T + t).T
    
    valid_mask = pts_cam[:, 2] > 0.2
    pts_valid = pts_lidar[valid_mask]
    depths = pts_cam[valid_mask, 2]
    
    if len(pts_valid) > 30000:
        step = len(pts_valid) // 30000
        pts_valid = pts_valid[::step]
        depths = depths[::step]
        
    proj_pts, _ = cv2.projectPoints(pts_valid, rvec, tvec, K, dist)
    proj_pts = proj_pts.reshape(-1, 2)
    
    # 0.5m ~ 8m depth normalization
    depth_norm = np.clip((depths - 0.5) / 7.5, 0.0, 1.0)
    colors = cv2.applyColorMap((depth_norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    
    overlay = img.copy()
    for (px, py), col in zip(proj_pts, colors):
        ix, iy = int(round(px)), int(round(py))
        if 0 <= ix < w and 0 <= iy < h:
            cv2.circle(overlay, (ix, iy), 2, (int(col[0,0]), int(col[0,1]), int(col[0,2])), -1)
            
    blend = cv2.addWeighted(img, 0.45, overlay, 0.55, 0)
    cv2.imwrite(save_path, blend)
    print(f"\n>>> [SAVED] Reprojection verification image saved to: {save_path} <<<")
    return blend

def update_fast_livo_config(config_path, Rcl, Pcl):
    with open(config_path, 'r') as f:
        content = f.read()
        
    r_flat = [round(float(x), 6) for x in Rcl.flatten()]
    p_flat = [round(float(x), 6) for x in Pcl]
    
    backup_path = f"{config_path}.bak.{int(time.time())}"
    with open(backup_path, 'w') as f:
        f.write(content)
    print(f"[BACKUP] Existing config backed up to {backup_path}")
    
    with open(config_path, 'r') as f:
        data = yaml.safe_load(f)
        
    extrin = data['/laserMapping']['ros__parameters']['extrin_calib']
    extrin['Rcl'] = r_flat
    extrin['Pcl'] = p_flat
    
    with open(config_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=None)
        
    print(f"\n>>> [UPDATED] {config_path} successfully updated with new Rcl & Pcl! <<<")

def capture_snapshot_live():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from livox_ros_driver2.msg import CustomMsg
    
    class SnapshotNode(Node):
        def __init__(self):
            super().__init__('snapshot_collector')
            self.bridge = CvBridge()
            self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.img_cb, 10)
            self.lidar_sub = self.create_subscription(CustomMsg, '/livox/lidar', self.lidar_cb, 10)
            self.captured_img = None
            self.points = []
            self.lidar_target_scans = 25 # ~2.5 seconds accumulation
            self.scan_count = 0
            
        def img_cb(self, msg):
            if self.captured_img is None:
                self.captured_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.get_logger().info(f"[SNAPSHOT] Camera frame captured: {self.captured_img.shape}")
                
        def lidar_cb(self, msg):
            if self.scan_count < self.lidar_target_scans:
                for p in msg.points:
                    if not (p.x == 0.0 and p.y == 0.0 and p.z == 0.0):
                        self.points.append([p.x, p.y, p.z])
                self.scan_count += 1
                if self.scan_count % 5 == 0:
                    print(f"  [LIDAR] Accumulating scans: {self.scan_count}/{self.lidar_target_scans} ({len(self.points)} points)...")
                    
    rclpy.init()
    node = SnapshotNode()
    print("\nWaiting for /camera/image_raw and /livox/lidar topics...")
    t_start = time.time()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.captured_img is not None and node.scan_count >= node.lidar_target_scans:
            break
        if time.time() - t_start > 25.0:
            print("[ERROR] Timeout waiting for sensor data. Are the camera and lidar driver nodes running?")
            rclpy.shutdown()
            sys.exit(1)
            
    img = node.captured_img
    pts = np.array(node.points, dtype=np.float32)
    rclpy.shutdown()
    return img, pts

def main():
    import argparse
    parser = argparse.ArgumentParser(description="LiDAR-Camera Extrinsic Calibration Tool")
    parser.add_argument("--img", type=str, default=None)
    parser.add_argument("--pcd", type=str, default=None)
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--cam_yaml", type=str, default="/root/zenith_ws/src/FAST-LIVO2/config/zenith_camera_pinhole.yaml")
    parser.add_argument("--livo_yaml", type=str, default="/root/zenith_ws/src/FAST-LIVO2/config/zenith_mid360s.yaml")
    args = parser.parse_args()
    
    calib_dir = "/root/zenith_ws/calib_extrinsic"
    os.makedirs(calib_dir, exist_ok=True)
    
    img_path = args.img
    pcd_path = args.pcd
    
    if args.live or (img_path is None and pcd_path is None):
        print("\n===========================================================")
        print("  Zenith Drone: LiDAR-Camera Extrinsic Calibration")
        print("===========================================================")
        print("Starting live snapshot acquisition...")
        img, pts = capture_snapshot_live()
        
        img_path = os.path.join(calib_dir, "calib_snap.png")
        pcd_path = os.path.join(calib_dir, "calib_snap.pcd")
        cv2.imwrite(img_path, img)
        
        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        o3d.io.write_point_cloud(pcd_path, pcd)
        print(f"Snapshot saved to:\n  - {img_path}\n  - {pcd_path}")
        
    K, dist = load_camera_intrinsics(args.cam_yaml)
    print("\n[INFO] Loaded Camera Intrinsics K:\n", K)
    print("[INFO] Distortion Coeffs:", dist.ravel())
    
    img_pts = pick_image_points(img_path)
    N = len(img_pts)
    
    obj_pts, pts_all = pick_pointcloud_points(pcd_path, N)
    
    Rcl, Pcl, rvec, tvec, mean_err, errors = compute_extrinsic(obj_pts, img_pts, K, dist)
    
    print("\n===========================================================")
    print("           CALIBRATION RESULTS (PnP Optimization)          ")
    print("===========================================================")
    print(f"Reprojection Error: Mean = {mean_err:.2f} pixels")
    for i, err in enumerate(errors):
        print(f"  Point #{i+1}: {err:.2f} px")
        
    print("\n>>> Rotation Matrix Rcl (3x3):")
    print(Rcl)
    print("\n>>> Translation Vector Pcl (x, y, z in meters):")
    print(f"[{Pcl[0]:.6f}, {Pcl[1]:.6f}, {Pcl[2]:.6f}]  (Distance: {np.linalg.norm(Pcl)*100:.1f} cm)")
    
    save_overlay = os.path.join(calib_dir, "reprojection_verification.jpg")
    blend = project_and_visualize(img_path, pts_all, K, dist, rvec, tvec, save_overlay)
    
    win_name = "3. Calibration Verification (LiDAR points projected on Camera)"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 960, 768)
    cv2.imshow(win_name, blend)
    print("\nPress any key in the verification image window to continue...")
    cv2.waitKey(0)
    cv2.destroyWindow(win_name)
    
    update_fast_livo_config(args.livo_yaml, Rcl, Pcl)
    print("\n[SUCCESS] Extrinsic calibration completed successfully!")

if __name__ == '__main__':
    main()
