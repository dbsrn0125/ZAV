#!/usr/bin/env python3
import os
import sys
import time
import json
import signal
import socket
import subprocess
from http.server import HTTPServer, BaseHTTPRequestHandler
import socketserver

WORKSPACE_DIR = "/home/radxa/zenith_ws"
AUDIO_DIR = os.path.join(WORKSPACE_DIR, "scripts", "audio")
BEEP_START = os.path.join(AUDIO_DIR, "beep_start.wav")
BEEP_STOP = os.path.join(AUDIO_DIR, "beep_stop.wav")
SCAN_SCRIPT = os.path.join(WORKSPACE_DIR, "scripts", "run_corridor_scan.sh")
STATUS_FILE = os.path.join(WORKSPACE_DIR, ".sensor_status.json")
PORT = 5000

# Global state
scan_proc = None
scan_start_time = None
scan_status = "idle"  # "idle", "recording", "stopping"
last_log_msg = "대기 중입니다."

def play_audio(filepath, repeat=1, delay=0.15):
    for _ in range(repeat):
        if os.path.exists(filepath):
            subprocess.run(["aplay", "-q", filepath], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            sys.stdout.write('\a')
            sys.stdout.flush()
        if repeat > 1:
            time.sleep(delay)

def get_ip_addresses():
    ips = []
    try:
        out = subprocess.check_output(["hostname", "-I"], text=True)
        for ip in out.strip().split():
            if ip and not ip.startswith("127.") and not ip.startswith("172.17."):
                ips.append(ip)
    except Exception:
        pass
    if not ips:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ips.append(s.getsockname()[0])
            s.close()
        except Exception:
            pass
    return ips if ips else ["127.0.0.1"]

def get_latest_bag_size_mb():
    try:
        bags_dir = os.path.join(WORKSPACE_DIR, "bags")
        if not os.path.exists(bags_dir):
            return 0.0
        dirs = [os.path.join(bags_dir, d) for d in os.listdir(bags_dir) if os.path.isdir(os.path.join(bags_dir, d))]
        if not dirs:
            return 0.0
        latest_dir = max(dirs, key=os.path.getmtime)
        total_size = sum(os.path.getsize(os.path.join(dp, f)) for dp, _, fn in os.walk(latest_dir) for f in fn)
        return round(total_size / (1024 * 1024), 1)
    except Exception:
        return 0.0

HTML_PAGE = """<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Zenith 드론 스캔 리모컨</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
        body {
            background-color: #0f172a;
            color: #f8fafc;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            min-height: 100vh;
            padding: 20px 16px;
            text-align: center;
        }
        .header {
            margin-bottom: 20px;
            position: relative;
            width: 100%;
            max-width: 380px;
        }
        .header h1 {
            font-size: 22px;
            font-weight: 700;
            letter-spacing: -0.5px;
            color: #38bdf8;
            margin-bottom: 4px;
        }
        .header p {
            font-size: 13px;
            color: #94a3b8;
        }
        .audio-toggle {
            position: absolute;
            right: 0;
            top: 2px;
            background: #1e293b;
            border: 1px solid #334155;
            color: #94a3b8;
            padding: 4px 10px;
            border-radius: 9999px;
            font-size: 12px;
            cursor: pointer;
        }

        /* Red Warning Alert Banner */
        .alert-banner {
            display: none;
            width: 100%;
            max-width: 380px;
            margin-bottom: 16px;
            padding: 14px 16px;
            border-radius: 16px;
            text-align: left;
            animation: alertPulse 1.2s infinite;
        }
        .alert-error {
            background: linear-gradient(135deg, #b91c1c 0%, #dc2626 100%);
            border: 2px solid #f87171;
            color: #ffffff;
            display: block;
        }
        .alert-warning {
            background: linear-gradient(135deg, #b45309 0%, #d97706 100%);
            border: 2px solid #fbbf24;
            color: #ffffff;
            display: block;
        }
        @keyframes alertPulse {
            0% { transform: scale(1); box-shadow: 0 0 0 0 rgba(239, 68, 68, 0.6); }
            50% { transform: scale(1.02); box-shadow: 0 0 20px 4px rgba(239, 68, 68, 0.8); }
            100% { transform: scale(1); box-shadow: 0 0 0 0 rgba(239, 68, 68, 0.6); }
        }
        .alert-title {
            font-size: 15px;
            font-weight: 800;
            display: flex;
            align-items: center;
            gap: 8px;
            margin-bottom: 4px;
        }
        .alert-desc {
            font-size: 13px;
            font-weight: 500;
            line-height: 1.4;
            opacity: 0.95;
        }

        /* Status Main Card */
        .status-card {
            background: #1e293b;
            border-radius: 20px;
            padding: 20px 24px;
            width: 100%;
            max-width: 380px;
            margin-bottom: 24px;
            box-shadow: 0 10px 25px -5px rgba(0, 0, 0, 0.4);
            border: 1px solid #334155;
            transition: all 0.3s ease;
        }
        .status-card.error {
            border-color: #ef4444;
            box-shadow: 0 0 25px rgba(239, 68, 68, 0.3);
        }
        .badge {
            display: inline-flex;
            align-items: center;
            gap: 8px;
            padding: 6px 14px;
            border-radius: 9999px;
            font-size: 13px;
            font-weight: 600;
            margin-bottom: 10px;
        }
        .badge-idle { background: rgba(34, 197, 94, 0.15); color: #4ade80; border: 1px solid #22c55e44; }
        .badge-recording { background: rgba(239, 68, 68, 0.15); color: #f87171; border: 1px solid #ef444444; }
        .badge-stopping { background: rgba(245, 158, 11, 0.15); color: #fbbf24; border: 1px solid #f59e0b44; }
        .badge-alert { background: rgba(239, 68, 68, 0.3); color: #fee2e2; border: 1px solid #ef4444; }
        .dot {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            background-color: currentColor;
        }
        .dot.pulse {
            animation: pulse 1.5s infinite;
        }
        @keyframes pulse {
            0% { opacity: 1; transform: scale(1); }
            50% { opacity: 0.4; transform: scale(1.3); }
            100% { opacity: 1; transform: scale(1); }
        }
        .timer {
            font-size: 38px;
            font-weight: 800;
            letter-spacing: -1px;
            font-variant-numeric: tabular-nums;
            color: #f1f5f9;
        }
        .timer-label {
            font-size: 12px;
            color: #64748b;
            margin-top: 2px;
        }

        /* 2x2 Sensor Health Grid */
        .sensor-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
            margin-top: 18px;
            padding-top: 16px;
            border-top: 1px solid #334155;
            text-align: left;
        }
        .sensor-card {
            background: #0f172a;
            border-radius: 12px;
            padding: 10px 12px;
            border: 1px solid #334155;
            transition: all 0.2s ease;
        }
        .sensor-card.warning {
            border-color: #f59e0b;
            background: rgba(245, 158, 11, 0.15);
        }
        .sensor-card.error {
            border-color: #ef4444;
            background: rgba(239, 68, 68, 0.15);
            animation: borderFlash 1s infinite;
        }
        @keyframes borderFlash {
            0%, 100% { border-color: #ef4444; }
            50% { border-color: #fca5a5; }
        }
        .sensor-header {
            font-size: 11px;
            font-weight: 600;
            color: #94a3b8;
            display: flex;
            align-items: center;
            justify-content: space-between;
            margin-bottom: 4px;
        }
        .sensor-val {
            font-size: 15px;
            font-weight: 700;
            color: #f8fafc;
        }
        .sensor-val.error {
            color: #f87171;
        }
        .sensor-status-tag {
            font-size: 10px;
            font-weight: 700;
            padding: 1px 5px;
            border-radius: 4px;
        }
        .tag-ok { background: rgba(34, 197, 94, 0.2); color: #4ade80; }
        .tag-warn { background: rgba(245, 158, 11, 0.2); color: #fbbf24; }
        .tag-err { background: rgba(239, 68, 68, 0.2); color: #f87171; }
        .tag-idle { background: rgba(148, 163, 184, 0.2); color: #94a3b8; }

        /* Action button */
        .action-container {
            width: 100%;
            max-width: 380px;
        }
        .btn {
            width: 100%;
            padding: 22px;
            border-radius: 20px;
            border: none;
            font-size: 19px;
            font-weight: 700;
            cursor: pointer;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 12px;
            transition: all 0.15s ease;
            box-shadow: 0 12px 28px -6px rgba(0, 0, 0, 0.5);
            -webkit-tap-highlight-color: transparent;
        }
        .btn:active {
            transform: scale(0.97);
        }
        .btn-start {
            background: linear-gradient(135deg, #10b981 0%, #059669 100%);
            color: white;
            box-shadow: 0 12px 28px -6px rgba(16, 185, 129, 0.4);
        }
        .btn-stop {
            background: linear-gradient(135deg, #ef4444 0%, #dc2626 100%);
            color: white;
            box-shadow: 0 12px 28px -6px rgba(239, 68, 68, 0.4);
        }
        .btn-disabled {
            background: #475569;
            color: #94a3b8;
            cursor: not-allowed;
            box-shadow: none;
            transform: none !important;
        }
        .msg {
            margin-top: 20px;
            font-size: 13px;
            color: #94a3b8;
            line-height: 1.5;
            max-width: 360px;
        }
        .footer {
            margin-top: 36px;
            font-size: 12px;
            color: #475569;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>Zenith Mobile Scanner</h1>
        <p>FAST-LIVO2 원격 스캔 컨트롤러 <span id="heartbeat-dot" style="display:inline-block;width:7px;height:7px;border-radius:50%;background:#10b981;margin-left:5px;vertical-align:middle;opacity:0.3;transition:opacity 0.25s;"></span></p>
        <button id="audio-toggle-btn" class="audio-toggle" onclick="toggleAudio()">🔔 소리 켬</button>
    </div>

    <!-- Alert Banner (shown only on sensor error/warning) -->
    <div id="alert-banner" class="alert-banner">
        <div class="alert-title">
            <span id="alert-icon">⚠️</span>
            <span id="alert-title-text">센서 오류 감지!</span>
        </div>
        <div id="alert-desc" class="alert-desc">카메라 영상 수신이 중단되었습니다. 자동 복구 중...</div>
    </div>

    <div id="status-card" class="status-card">
        <div id="status-badge" class="badge badge-idle">
            <span id="status-dot" class="dot"></span>
            <span id="status-text">대기 중 (준비완료)</span>
        </div>
        <div id="timer" class="timer">00:00</div>
        <div class="timer-label">녹화 경과 시간</div>

        <!-- 4 Sensor Cards Grid -->
        <div class="sensor-grid">
            <div id="cam-card" class="sensor-card">
                <div class="sensor-header">
                    <span>📷 카메라</span>
                    <span id="cam-tag" class="sensor-status-tag tag-idle">대기</span>
                </div>
                <div id="cam-val" class="sensor-val">- Hz</div>
            </div>

            <div id="lidar-card" class="sensor-card">
                <div class="sensor-header">
                    <span>📡 라이다</span>
                    <span id="lidar-tag" class="sensor-status-tag tag-idle">대기</span>
                </div>
                <div id="lidar-val" class="sensor-val">- Hz</div>
            </div>

            <div id="imu-card" class="sensor-card">
                <div class="sensor-header">
                    <span>🧭 IMU</span>
                    <span id="imu-tag" class="sensor-status-tag tag-idle">대기</span>
                </div>
                <div id="imu-val" class="sensor-val">- Hz</div>
            </div>

            <div id="bag-card" class="sensor-card">
                <div class="sensor-header">
                    <span>💾 저장소</span>
                    <span id="bag-tag" class="sensor-status-tag tag-idle">대기</span>
                </div>
                <div id="bag-val" class="sensor-val">- MB</div>
            </div>

            <div id="mem-card" class="sensor-card">
                <div class="sensor-header">
                    <span>🧠 메모리</span>
                    <span id="mem-tag" class="sensor-status-tag tag-idle">대기</span>
                </div>
                <div id="mem-val" class="sensor-val">- GB</div>
            </div>

            <div id="cpu-card" class="sensor-card">
                <div class="sensor-header">
                    <span>⚡ CPU/온도</span>
                    <span id="cpu-tag" class="sensor-status-tag tag-idle">대기</span>
                </div>
                <div id="cpu-val" class="sensor-val">- °C</div>
            </div>
        </div>
    </div>

    <div class="action-container">
        <button id="main-btn" class="btn btn-start" onclick="toggleScan()">
            <span>▶ 스캔 녹화 시작</span>
        </button>
    </div>

    <div id="log-msg" class="msg">출발점에 서서 녹화 시작 버튼을 누르세요.</div>
    <div class="footer">Radxa Dragon Q6A · Zenith aerial vehicle</div>

    <script>
        let currentState = "idle";
        let isProcessing = false;
        let isMuted = false;
        let audioCtx = null;
        let lastBeepTime = 0;

        function initAudio() {
            if (!audioCtx) {
                audioCtx = new (window.AudioContext || window.webkitAudioContext)();
            }
        }

        function toggleAudio() {
            isMuted = !isMuted;
            const btn = document.getElementById('audio-toggle-btn');
            if (isMuted) {
                btn.textContent = "🔕 음소거";
                btn.style.color = "#ef4444";
            } else {
                btn.textContent = "🔔 소리 켬";
                btn.style.color = "#94a3b8";
                initAudio();
                if (audioCtx && audioCtx.state === 'suspended') {
                    audioCtx.resume();
                }
            }
        }

        function triggerAlarmSound() {
            if (isMuted) return;
            const now = Date.now();
            if (now - lastBeepTime < 2200) return; // at most once per 2.2s
            lastBeepTime = now;

            try {
                initAudio();
                if (audioCtx && audioCtx.state === 'suspended') {
                    audioCtx.resume();
                }
                if (audioCtx) {
                    const osc = audioCtx.createOscillator();
                    const gain = audioCtx.createGain();
                    osc.type = 'sawtooth';
                    osc.frequency.setValueAtTime(880, audioCtx.currentTime);
                    osc.frequency.setValueAtTime(440, audioCtx.currentTime + 0.15);
                    osc.frequency.setValueAtTime(880, audioCtx.currentTime + 0.3);
                    gain.gain.setValueAtTime(0.3, audioCtx.currentTime);
                    gain.gain.exponentialRampToValueAtTime(0.01, audioCtx.currentTime + 0.45);
                    osc.connect(gain);
                    gain.connect(audioCtx.destination);
                    osc.start();
                    osc.stop(audioCtx.currentTime + 0.45);
                }
            } catch(e) {}

            if ('vibrate' in navigator) {
                navigator.vibrate([250, 100, 250, 100, 250]);
            }
        }

        async function updateStatus() {
            try {
                const res = await fetch('/api/status');
                const data = await res.json();
                
                const hDot = document.getElementById('heartbeat-dot');
                if (hDot) {
                    hDot.style.opacity = '1.0';
                    setTimeout(() => { hDot.style.opacity = '0.3'; }, 250);
                }

                currentState = data.status;
                const statusCard = document.getElementById('status-card');
                const badge = document.getElementById('status-badge');
                const dot = document.getElementById('status-dot');
                const statusText = document.getElementById('status-text');
                const timer = document.getElementById('timer');
                const btn = document.getElementById('main-btn');
                const logMsg = document.getElementById('log-msg');
                const alertBanner = document.getElementById('alert-banner');
                const alertTitleText = document.getElementById('alert-title-text');
                const alertDesc = document.getElementById('alert-desc');
                const alertIcon = document.getElementById('alert-icon');

                const camCard = document.getElementById('cam-card');
                const camVal = document.getElementById('cam-val');
                const camTag = document.getElementById('cam-tag');

                const lidarCard = document.getElementById('lidar-card');
                const lidarVal = document.getElementById('lidar-val');
                const lidarTag = document.getElementById('lidar-tag');

                const imuCard = document.getElementById('imu-card');
                const imuVal = document.getElementById('imu-val');
                const imuTag = document.getElementById('imu-tag');

                const bagCard = document.getElementById('bag-card');
                const bagVal = document.getElementById('bag-val');
                const bagTag = document.getElementById('bag-tag');

                const memCard = document.getElementById('mem-card');
                const memVal = document.getElementById('mem-val');
                const memTag = document.getElementById('mem-tag');

                const cpuCard = document.getElementById('cpu-card');
                const cpuVal = document.getElementById('cpu-val');
                const cpuTag = document.getElementById('cpu-tag');

                const sensors = data.sensors;
                const hasAlert = (sensors && (sensors.alert_level === 'error' || sensors.alert_level === 'warning'));

                if (currentState === "idle") {
                    alertBanner.style.display = "none";
                    statusCard.className = "status-card";
                    badge.className = "badge badge-idle";
                    dot.className = "dot";
                    statusText.textContent = "대기 중 (준비완료)";
                    timer.textContent = "00:00";
                    btn.className = "btn btn-start";
                    btn.innerHTML = "<span>▶ 스캔 녹화 시작</span>";
                    btn.disabled = false;
                    logMsg.textContent = data.msg || "출발점에 서서 녹화 시작 버튼을 누르세요.";

                    camCard.className = "sensor-card";
                    camTag.className = "sensor-status-tag tag-idle";
                    camTag.textContent = "대기";
                    camVal.textContent = "- Hz";

                    lidarCard.className = "sensor-card";
                    lidarTag.className = "sensor-status-tag tag-idle";
                    lidarTag.textContent = "대기";
                    lidarVal.textContent = "- Hz";

                    imuCard.className = "sensor-card";
                    imuTag.className = "sensor-status-tag tag-idle";
                    imuTag.textContent = "대기";
                    imuVal.textContent = "- Hz";

                    bagCard.className = "sensor-card";
                    bagTag.className = "sensor-status-tag tag-idle";
                    bagTag.textContent = "대기";
                    bagVal.textContent = sensors && sensors.disk_free_gb ? `여유 ${sensors.disk_free_gb}G` : "- MB";

                    memCard.className = "sensor-card";
                    memTag.className = "sensor-status-tag tag-idle";
                    memTag.textContent = "대기";
                    if (sensors && sensors.memory) {
                        const used = (sensors.memory.used_gb != null) ? Number(sensors.memory.used_gb).toFixed(1) : (sensors.memory.total_gb - sensors.memory.avail_gb).toFixed(1);
                        memVal.textContent = `${used}G (${sensors.memory.percent}%)`;
                    } else {
                        memVal.textContent = "- GB";
                    }

                    cpuCard.className = "sensor-card";
                    cpuTag.className = "sensor-status-tag tag-idle";
                    cpuTag.textContent = "대기";
                    cpuVal.textContent = sensors && sensors.cpu ? `${sensors.cpu.temp_c}°C` : "- °C";

                } else if (currentState === "recording") {
                    const sec = Math.floor(data.duration);
                    const m = String(Math.floor(sec / 60)).padStart(2, '0');
                    const s = String(sec % 60).padStart(2, '0');
                    timer.textContent = `${m}:${s}`;
                    
                    btn.className = "btn btn-stop";
                    btn.innerHTML = "<span>⏹ 녹화 종료 및 저장</span>";
                    btn.disabled = false;

                    // Sensor monitoring active
                    if (sensors) {
                        // Camera
                        if (sensors.camera) {
                            camVal.textContent = `${sensors.camera.fps} Hz`;
                            if (sensors.camera.ok) {
                                camCard.className = "sensor-card";
                                camVal.className = "sensor-val";
                                camTag.className = "sensor-status-tag tag-ok";
                                camTag.textContent = "정상";
                            } else {
                                camCard.className = "sensor-card error";
                                camVal.className = "sensor-val error";
                                camTag.className = "sensor-status-tag tag-err";
                                camTag.textContent = sensors.state === "INITIALIZING" ? "연결중" : "중단";
                            }
                        }

                        // LiDAR
                        if (sensors.lidar) {
                            lidarVal.textContent = `${sensors.lidar.fps} Hz`;
                            if (sensors.lidar.ok) {
                                lidarCard.className = "sensor-card";
                                lidarVal.className = "sensor-val";
                                lidarTag.className = "sensor-status-tag tag-ok";
                                lidarTag.textContent = "정상";
                            } else {
                                lidarCard.className = "sensor-card error";
                                lidarVal.className = "sensor-val error";
                                lidarTag.className = "sensor-status-tag tag-err";
                                lidarTag.textContent = sensors.state === "INITIALIZING" ? "연결중" : "중단";
                            }
                        }

                        // IMU
                        if (sensors.imu) {
                            imuVal.textContent = `${sensors.imu.fps} Hz`;
                            if (sensors.imu.ok) {
                                imuCard.className = "sensor-card";
                                imuVal.className = "sensor-val";
                                imuTag.className = "sensor-status-tag tag-ok";
                                imuTag.textContent = "정상";
                            } else {
                                imuCard.className = "sensor-card error";
                                imuVal.className = "sensor-val error";
                                imuTag.className = "sensor-status-tag tag-err";
                                imuTag.textContent = sensors.state === "INITIALIZING" ? "연결중" : "중단";
                            }
                        }

                        // Bag & Storage
                        if (sensors.bag) {
                            const mb = Number(sensors.bag.size_mb) || 0;
                            if (mb >= 1024) {
                                bagVal.textContent = `${(mb / 1024).toFixed(2)} GB`;
                            } else {
                                bagVal.textContent = `${mb.toFixed(1)} MB`;
                            }
                            bagCard.className = "sensor-card";
                            bagTag.className = "sensor-status-tag tag-ok";
                            bagTag.textContent = sensors.disk_free_gb ? `여유 ${sensors.disk_free_gb}G` : "기록 중";
                        }

                        // Memory (RAM)
                        if (sensors.memory) {
                            const used = (sensors.memory.used_gb != null) ? Number(sensors.memory.used_gb).toFixed(1) : (sensors.memory.total_gb - sensors.memory.avail_gb).toFixed(1);
                            memVal.textContent = `${used}G (${sensors.memory.percent}%)`;
                            if (sensors.memory.avail_gb < 0.8) {
                                memCard.className = "sensor-card error";
                                memVal.className = "sensor-val error";
                                memTag.className = "sensor-status-tag tag-err";
                                memTag.textContent = "위험";
                            } else if (sensors.memory.avail_gb < 1.8) {
                                memCard.className = "sensor-card warning";
                                memVal.className = "sensor-val";
                                memTag.className = "sensor-status-tag tag-warn";
                                memTag.textContent = "주의";
                            } else {
                                memCard.className = "sensor-card";
                                memVal.className = "sensor-val";
                                memTag.className = "sensor-status-tag tag-ok";
                                memTag.textContent = "정상";
                            }
                        }

                        // CPU & Temperature
                        if (sensors.cpu) {
                            cpuVal.textContent = `${sensors.cpu.percent}% · ${sensors.cpu.temp_c}°C`;
                            if (sensors.cpu.temp_c >= 85.0) {
                                cpuCard.className = "sensor-card error";
                                cpuVal.className = "sensor-val error";
                                cpuTag.className = "sensor-status-tag tag-err";
                                cpuTag.textContent = "과열";
                            } else if (sensors.cpu.temp_c >= 78.0) {
                                cpuCard.className = "sensor-card warning";
                                cpuVal.className = "sensor-val";
                                cpuTag.className = "sensor-status-tag tag-warn";
                                cpuTag.textContent = "주의";
                            } else {
                                cpuCard.className = "sensor-card";
                                cpuVal.className = "sensor-val";
                                cpuTag.className = "sensor-status-tag tag-ok";
                                cpuTag.textContent = "정상";
                            }
                        }

                        // Check Alerts
                        if (sensors.alert_level === 'error') {
                            alertBanner.className = "alert-banner alert-error";
                            alertTitleText.textContent = "🚨 센서 오류 감지!";
                            alertDesc.textContent = sensors.alert_msg || "센서 수신 이상이 발생했습니다.";
                            statusCard.className = "status-card error";
                            badge.className = "badge badge-alert";
                            dot.className = "dot pulse";
                            statusText.textContent = "⚠️ 센서 오류 감지!";
                            logMsg.textContent = sensors.alert_msg;
                            triggerAlarmSound();
                        } else if (sensors.alert_level === 'warning') {
                            alertBanner.className = "alert-banner alert-warning";
                            alertTitleText.textContent = "⚠️ 주의 사항";
                            alertDesc.textContent = sensors.alert_msg || "시스템 경고가 발생했습니다.";
                            statusCard.className = "status-card";
                            badge.className = "badge badge-recording";
                            dot.className = "dot pulse";
                            statusText.textContent = "녹화 진행 중 (주의)";
                        } else {
                            alertBanner.style.display = "none";
                            statusCard.className = "status-card";
                            badge.className = "badge badge-recording";
                            dot.className = "dot pulse";
                            statusText.textContent = "녹화 진행 중 (정상)";
                            logMsg.textContent = "모든 센서가 정상 동작 중입니다. 복도를 천천히 걸으세요.";
                        }
                    } else {
                        badge.className = "badge badge-recording";
                        dot.className = "dot pulse";
                        statusText.textContent = "녹화 준비 중...";
                    }

                } else if (currentState === "stopping") {
                    alertBanner.style.display = "none";
                    statusCard.className = "status-card";
                    badge.className = "badge badge-stopping";
                    dot.className = "dot pulse";
                    statusText.textContent = "저장 중...";
                    btn.className = "btn btn-disabled";
                    btn.innerHTML = "<span>저장 중입니다...</span>";
                    btn.disabled = true;
                    logMsg.textContent = "센서 백파일 및 3D 맵을 안전하게 저장하고 있습니다...";
                }
            } catch (err) {
                console.error("Status check failed:", err);
            }
        }

        async function toggleScan() {
            if (isProcessing) return;
            isProcessing = true;
            initAudio();

            const btn = document.getElementById('main-btn');
            btn.disabled = true;

            try {
                if (currentState === "idle") {
                    await fetch('/api/start', { method: 'POST' });
                } else if (currentState === "recording") {
                    await fetch('/api/stop', { method: 'POST' });
                }
            } catch (err) {
                alert("명령 전송 실패: " + err);
            } finally {
                setTimeout(() => {
                    isProcessing = false;
                    updateStatus();
                }, 500);
            }
        }

        setInterval(updateStatus, 800);
        updateStatus();
    </script>
</body>
</html>
"""

class RequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return

    def do_GET(self):
        global scan_status, scan_start_time, last_log_msg
        if self.path == "/" or self.path == "/index.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))
        elif self.path == "/api/status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.end_headers()
            dur = 0.0
            if scan_status == "recording" and scan_start_time:
                dur = time.time() - scan_start_time

            # Read sensor status if available
            sensor_data = None
            if os.path.exists(STATUS_FILE):
                try:
                    with open(STATUS_FILE, "r") as f:
                        data = json.load(f)
                    age = time.time() - data.get("updated_at", 0)
                    if scan_status == "recording" and dur > 6.0 and age > 3.5:
                        data["overall_ok"] = False
                        data["alert_level"] = "error"
                        data["alert_msg"] = "⚠️ 센서 상태 모니터 응답 지연 (노드 확인 중...)"
                    if scan_status == "recording" and "bag" in data:
                        data["bag"]["size_mb"] = get_latest_bag_size_mb()
                    sensor_data = data
                except Exception:
                    pass

            payload = {
                "status": scan_status,
                "duration": dur,
                "msg": last_log_msg,
                "sensors": sensor_data
            }
            self.wfile.write(json.dumps(payload).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        global scan_proc, scan_start_time, scan_status, last_log_msg
        if self.path == "/api/start":
            if scan_status == "idle":
                print("\n>>> [스마트폰 원격 명령] 녹화 시작! <<<")
                play_audio(BEEP_START, repeat=1)
                try:
                    init_data = {
                        "updated_at": time.time(),
                        "uptime": 0.0,
                        "state": "INITIALIZING",
                        "overall_ok": True,
                        "alert_level": "ok",
                        "alert_msg": None,
                        "camera": {"ok": True, "fps": 0.0, "status": "연결 중"},
                        "lidar": {"ok": True, "fps": 0.0, "status": "연결 중"},
                        "imu": {"ok": True, "fps": 0.0, "status": "연결 중"},
                        "bag": {"ok": True, "size_mb": 0, "status": "대기 중"}
                    }
                    with open(STATUS_FILE, "w") as f:
                        json.dump(init_data, f)
                    os.chmod(STATUS_FILE, 0o666)
                except Exception:
                    pass

                scan_proc = subprocess.Popen(
                    [SCAN_SCRIPT, "1"],
                    preexec_fn=os.setpgrp,
                    cwd=WORKSPACE_DIR
                )
                scan_start_time = time.time()
                scan_status = "recording"
                last_log_msg = "녹화가 진행 중입니다."
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"ok": True}).encode('utf-8'))

        elif self.path == "/api/stop":
            if scan_status == "recording":
                print("\n>>> [스마트폰 원격 명령] 녹화 종료 요청! <<<")
                scan_status = "stopping"
                last_log_msg = "백파일을 저장하고 있습니다..."
                
                # 1. Terminate container nodes with SIGINT
                subprocess.run([
                    "docker", "exec", "zenith_dev", "bash", "-c",
                    "pkill -2 -f '[f]astlivo_mapping'; pkill -2 -f '[r]os2 bag record'; sleep 1; pkill -2 -f '[m]vs_camera'; pkill -2 -f '[l]ivox_ros_driver2'; pkill -2 -f '[z]enith_health_monitor'; pkill -2 -f '[z]enith_corridor_scan'"
                ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

                # 2. Terminate host runner script
                if scan_proc:
                    try:
                        os.killpg(os.getpgid(scan_proc.pid), signal.SIGINT)
                        scan_proc.wait(timeout=4)
                    except Exception:
                        try:
                            os.killpg(os.getpgid(scan_proc.pid), signal.SIGTERM)
                            scan_proc.wait(timeout=2)
                        except Exception:
                            pass
                    scan_proc = None
                
                try:
                    idle_data = {
                        "updated_at": time.time(),
                        "state": "IDLE",
                        "overall_ok": True,
                        "alert_level": "ok",
                        "alert_msg": None
                    }
                    with open(STATUS_FILE, "w") as f:
                        json.dump(idle_data, f)
                    os.chmod(STATUS_FILE, 0o666)
                except Exception:
                    pass

                scan_status = "idle"
                last_log_msg = "백파일 저장이 성공적으로 완료되었습니다!"
                print(">>> [완료] 백파일 저장 완료! <<<")
                play_audio(BEEP_STOP, repeat=2, delay=0.15)

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"ok": True}).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

class ThreadedHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

def main():
    ips = get_ip_addresses()
    server_address = ('0.0.0.0', PORT)
    httpd = ThreadedHTTPServer(server_address, RequestHandler)

    print("==========================================================")
    print("   Zenith Drone: 스마트폰 웹 리모컨 서버 가동 완료!")
    print("==========================================================")
    print("스마트폰(사파리/크롬) 브라우저를 열고 아래 주소로 접속하세요:")
    for ip in ips:
        print(f"   👉 http://{ip}:{PORT}")
    print("----------------------------------------------------------")
    print("※ 스마트폰과 라덱사 보드가 같은 Wi-Fi 또는 핫스팟에 연결되어 있어야 합니다.")
    print("※ 서버를 끄려면 터미널에서 [Ctrl+C]를 누르세요.")
    print("==========================================================")
    sys.stdout.flush()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[INFO] 웹 리모컨 서버를 종료합니다.")
        if scan_proc:
            try:
                os.killpg(os.getpgid(scan_proc.pid), signal.SIGINT)
                scan_proc.wait(timeout=10)
            except Exception:
                pass
        httpd.server_close()
        print("[INFO] 정상 종료되었습니다.")

if __name__ == "__main__":
    main()
