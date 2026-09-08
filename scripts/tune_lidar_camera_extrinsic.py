#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Zenith Aerial Vehicle (ZAV) - Real-Time Visual Extrinsic Tuner
Real-time keyboard-based alignment of LiDAR point cloud over camera image.
"""

import os
import sys
import time
import yaml
import numpy as np
import cv2
import open3d as o3d

def load_camera_intrinsics(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    params = data['/laserMapping']['ros__parameters']['camera']
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

def load_initial_extrinsics(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    extrin = data['/laserMapping']['ros__parameters']['extrin_calib']
    R_flat = extrin['Rcl']
    P_flat = extrin['Pcl']
    R_mat = np.array(R_flat, dtype=np.float64).reshape(3, 3)
    P_vec = np.array(P_flat, dtype=np.float64)
    return R_mat, P_vec

def save_extrinsics(yaml_path, R_mat, P_vec):
    backup_path = f"{yaml_path}.bak.{int(time.time())}"
    with open(yaml_path, 'r') as f:
        content = f.read()
    with open(backup_path, 'w') as f:
        f.write(content)
        
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    extrin = data['/laserMapping']['ros__parameters']['extrin_calib']
    extrin['Rcl'] = [round(float(x), 6) for x in R_mat.flatten()]
    extrin['Pcl'] = [round(float(x), 6) for x in P_vec]
    
    with open(yaml_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=None)
    print(f"\n>>> [SUCCESS] Saved updated Rcl and Pcl to {yaml_path}! <<<")
    print(f"Backup saved to: {backup_path}")

def make_rotation_matrix(rx, ry, rz):
    Rx, _ = cv2.Rodrigues(np.array([rx, 0.0, 0.0]))
    Ry, _ = cv2.Rodrigues(np.array([0.0, ry, 0.0]))
    Rz, _ = cv2.Rodrigues(np.array([0.0, 0.0, rz]))
    return Rz @ Ry @ Rx

def main():
    img_path = "/root/zenith_ws/calib_extrinsic/corridor_test.png"
    pcd_path = "/root/zenith_ws/calib_extrinsic/corridor_test.pcd"
    cam_yaml = "/root/zenith_ws/src/FAST-LIVO2/config/zenith_camera_pinhole.yaml"
    livo_yaml = "/root/zenith_ws/src/FAST-LIVO2/config/zenith_mid360s.yaml"
    
    if len(sys.argv) > 1 and os.path.exists(sys.argv[1]):
        img_path = sys.argv[1]
    if len(sys.argv) > 2 and os.path.exists(sys.argv[2]):
        pcd_path = sys.argv[2]
        
    K, dist = load_camera_intrinsics(cam_yaml)
    
    # 22-degree forward tilt baseline (physically correct sign)
    s22 = np.sin(np.radians(22.0))
    c22 = np.cos(np.radians(22.0))
    R_base_22 = np.array([
        [ 0.0, -1.0,  0.0],
        [ s22,  0.0, -c22],
        [ c22,  0.0,  s22]
    ], dtype=np.float64)
    P_base_22 = np.array([0.0, 0.0, -0.05], dtype=np.float64)
    
    R_curr = R_base_22.copy()
    P_curr = P_base_22.copy()
    
    img = cv2.imread(img_path)
    if img is None:
        print(f"[ERROR] Cannot load image: {img_path}")
        return
    H, W = img.shape[:2]
    
    pcd = o3d.io.read_point_cloud(pcd_path)
    pts_raw = np.asarray(pcd.points)
    if len(pts_raw) > 25000:
        step = len(pts_raw) // 25000
        pts_raw = pts_raw[::step]
        
    print("\n=======================================================")
    print("   Zenith Drone: Real-Time Visual Extrinsic Tuner      ")
    print("=======================================================")
    print("  [Baseline: LiDAR 22 deg Forward-Tilt Applied]        ")
    print("  -----------------------------------------------------")
    print("  [Rotation (deg)]        [Translation (cm)]")
    print("   W / S : Pitch (Tilt)     I / K : Forward / Back (X)")
    print("   A / D : Yaw (Pan)        J / L : Left / Right (Y)")
    print("   Q / E : Roll             U / O : Up / Down (Z)")
    print("  -----------------------------------------------------")
    print("   [1] / [2] : Finer / Coarser step size")
    print("   [0]       : Reset to 22 deg forward baseline")
    print("   [Enter]   : SAVE to config and exit")
    print("   [ESC]     : Exit without saving")
    print("=======================================================\n")
    
    rot_step_deg = 0.5
    trans_step_m = 0.005 # 5mm
    
    win_name = "Zenith Visual Tuner (W/S/A/D/Q/E=Rot, I/K/J/L/U/O=Pos, Enter=Save)"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 1100, 850)
    
    while True:
        rvec, _ = cv2.Rodrigues(R_curr)
        tvec = P_curr.reshape(3, 1)
        
        pts_cam = (R_curr @ pts_raw.T + tvec).T
        valid = pts_cam[:, 2] > 0.3
        pts_v = pts_raw[valid]
        depths = pts_cam[valid, 2]
        
        proj_pts, _ = cv2.projectPoints(pts_v, rvec, tvec, K, dist)
        proj_pts = proj_pts.reshape(-1, 2)
        
        depth_norm = np.clip((depths - 0.5) / 10.0, 0.0, 1.0)
        colors = cv2.applyColorMap((depth_norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
        
        overlay = img.copy()
        for (px, py), col in zip(proj_pts, colors):
            ix, iy = int(round(px)), int(round(py))
            if 0 <= ix < W and 0 <= iy < H:
                cv2.circle(overlay, (ix, iy), 2, (int(col[0,0]), int(col[0,1]), int(col[0,2])), -1)
                
        blend = cv2.addWeighted(img, 0.38, overlay, 0.62, 0)
        
        # HUD
        hud_h = 75
        cv2.rectangle(blend, (0, 0), (W, hud_h), (25, 25, 25), -1)
        cv2.putText(blend, f"Pos(cm): X={P_curr[0]*100:.1f}, Y={P_curr[1]*100:.1f}, Z={P_curr[2]*100:.1f} | Step: Rot={rot_step_deg:.1f}d, Pos={trans_step_m*100:.1f}cm",
                    (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
        cv2.putText(blend, "[W/S/A/D] Tilt/Pan | [Q/E] Roll | [I/K/J/L/U/O] Pos | [Enter] SAVE | [ESC] Cancel",
                    (15, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
                    
        cv2.imshow(win_name, blend)
        key = cv2.waitKey(0) & 0xFF
        
        if key in [13, 32]: # Enter / Space
            save_extrinsics(livo_yaml, R_curr, P_curr)
            cv2.imwrite("/root/zenith_ws/calib_extrinsic/tuned_verification.jpg", blend)
            break
        elif key == 27: # ESC
            print("[INFO] Cancelled without saving.")
            break
            
        step_rad = np.radians(rot_step_deg)
        d_rot = np.zeros(3)
        if key == ord('w'): d_rot[0] -= step_rad # Pitch up
        elif key == ord('s'): d_rot[0] += step_rad # Pitch down
        elif key == ord('a'): d_rot[1] -= step_rad # Yaw left
        elif key == ord('d'): d_rot[1] += step_rad # Yaw right
        elif key == ord('q'): d_rot[2] -= step_rad # Roll ccw
        elif key == ord('e'): d_rot[2] += step_rad # Roll cw
        
        if np.any(d_rot != 0):
            dR = make_rotation_matrix(d_rot[0], d_rot[1], d_rot[2])
            R_curr = dR @ R_curr
            
        if key == ord('i'): P_curr[0] += trans_step_m
        elif key == ord('k'): P_curr[0] -= trans_step_m
        elif key == ord('j'): P_curr[1] -= trans_step_m
        elif key == ord('l'): P_curr[1] += trans_step_m
        elif key == ord('u'): P_curr[2] += trans_step_m
        elif key == ord('o'): P_curr[2] -= trans_step_m
        
        elif key == ord('0'):
            R_curr = R_base_22.copy()
            P_curr = P_base_22.copy()
            print("[INFO] Reset to 22-degree forward baseline.")
            
        elif key == ord('1'):
            rot_step_deg = max(0.1, rot_step_deg / 2.0)
            trans_step_m = max(0.001, trans_step_m / 2.0)
        elif key == ord('2'):
            rot_step_deg = min(5.0, rot_step_deg * 2.0)
            trans_step_m = min(0.05, trans_step_m * 2.0)
            
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
