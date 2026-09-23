#!/usr/bin/env python3
import os
import sys
import time

SRC_DIR = "/home/radxa/zenith_ws/bags"
DST_DIR = (
    "/run/user/1000/gvfs/google-drive:host=gmail.com,user=yoongu11/"
    "0AFhbznbgR_pjUk9PVA/12zjfLYFuOLTps7h3mFpw0S9nDX3dNfIC/"
    "1eLaXcic7KFHjsrUkKs7gy9SM4st02onC/1uMtH9IcmYo4t8uXpnZhG8_lDtXF9cshZ/"
    "1bqlKkglchJvRTkQCV3TSCMyN8Zat8pJG/bags"
)

BAG_ORDER = [
    "scan_sensors_20260918_095452",
    "scan_sensors_20260918_094749",
    "scan_sensors_20260918_094324",
    "scan_sensors_20260917_170833",
    "scan_sensors_20260918_092923",
]

CHUNK_SIZE = 16 * 1024 * 1024  # 16 MB chunks

def format_bytes(num_bytes):
    for unit in ['B', 'KB', 'MB', 'GB', 'TB']:
        if abs(num_bytes) < 1024.0:
            return f"{num_bytes:.2f} {unit}"
        num_bytes /= 1024.0
    return f"{num_bytes:.2f} PB"

def format_eta(seconds):
    if seconds < 0 or seconds > 86400 * 7:
        return "--:--:--"
    m, s = divmod(int(seconds), 60)
    h, m = divmod(m, 60)
    return f"{h:02d}:{m:02d}:{s:02d}"

def copy_file_with_progress(src_path, dst_path):
    total_size = os.path.getsize(src_path)
    
    # Check if already exists and matches size
    if os.path.exists(dst_path):
        try:
            dst_size = os.path.getsize(dst_path)
            if dst_size == total_size:
                print(f"  [ALREADY DONE] {os.path.basename(src_path)} ({format_bytes(total_size)}) matches dst. Skipping.", flush=True)
                return True
        except Exception:
            pass

    print(f"  [UPLOADING] {os.path.basename(src_path)} ({format_bytes(total_size)})...", flush=True)
    
    t_start = time.time()
    last_print = t_start
    copied = 0

    with open(src_path, "rb") as f_src, open(dst_path, "wb") as f_dst:
        while True:
            chunk = f_src.read(CHUNK_SIZE)
            if not chunk:
                break
            f_dst.write(chunk)
            copied += len(chunk)

            now = time.time()
            if now - last_print >= 5.0 or copied == total_size:
                elapsed = now - t_start
                speed = copied / elapsed if elapsed > 0 else 0
                pct = (copied / total_size) * 100 if total_size > 0 else 100
                remaining_bytes = total_size - copied
                eta = remaining_bytes / speed if speed > 0 else 0
                print(
                    f"    -> {pct:5.1f}% ({format_bytes(copied)} / {format_bytes(total_size)}) "
                    f"@ {format_bytes(speed)}/s | ETA: {format_eta(eta)}",
                    flush=True,
                )
                last_print = now

    total_time = time.time() - t_start
    avg_speed = total_size / total_time if total_time > 0 else 0
    print(f"  [DONE] Completed in {format_eta(total_time)} (avg {format_bytes(avg_speed)}/s)", flush=True)
    return True

def main():
    print("=" * 60, flush=True)
    print("      Zenith WS -> Google Drive ROS 2 Bag Uploader", flush=True)
    print("=" * 60, flush=True)
    print(f"Source:      {SRC_DIR}", flush=True)
    print(f"Destination: {DST_DIR}", flush=True)
    print("-" * 60, flush=True)

    if not os.path.exists(DST_DIR):
        print(f"Creating destination directory: {DST_DIR}", flush=True)
        os.makedirs(DST_DIR, exist_ok=True)

    total_uploaded = 0
    all_bags = [b for b in BAG_ORDER if os.path.isdir(os.path.join(SRC_DIR, b))]
    # Add any other bags not explicitly in the list
    for b in sorted(os.listdir(SRC_DIR)):
        if b not in all_bags and os.path.isdir(os.path.join(SRC_DIR, b)):
            all_bags.append(b)

    total_bag_size = 0
    for b in all_bags:
        b_path = os.path.join(SRC_DIR, b)
        for root, _, files in os.walk(b_path):
            for f in files:
                total_bag_size += os.path.getsize(os.path.join(root, f))

    print(f"Total Bags to upload: {len(all_bags)} ({format_bytes(total_bag_size)})", flush=True)
    print("-" * 60, flush=True)

    start_all = time.time()
    for idx, bname in enumerate(all_bags, 1):
        src_bag = os.path.join(SRC_DIR, bname)
        dst_bag = os.path.join(DST_DIR, bname)
        os.makedirs(dst_bag, exist_ok=True)

        bag_size = sum(os.path.getsize(os.path.join(src_bag, f)) for f in os.listdir(src_bag))
        print(f"\n[{idx}/{len(all_bags)}] Uploading Bag: {bname} ({format_bytes(bag_size)})", flush=True)

        files = sorted(os.listdir(src_bag))
        # Ensure metadata.yaml is uploaded after or alongside .db3
        for fname in files:
            src_file = os.path.join(src_bag, fname)
            dst_file = os.path.join(dst_bag, fname)
            copy_file_with_progress(src_file, dst_file)

    total_elapsed = time.time() - start_all
    print("\n" + "=" * 60, flush=True)
    print(f"[SUCCESS] All {len(all_bags)} bags uploaded to Google Drive successfully!", flush=True)
    print(f"Total time elapsed: {format_eta(total_elapsed)}", flush=True)
    print("=" * 60, flush=True)

if __name__ == "__main__":
    main()
