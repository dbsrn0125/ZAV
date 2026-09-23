#!/usr/bin/env python3
import os
import sys
import time
import signal
import select
import tty
import termios
import subprocess
import glob

WORKSPACE_DIR = "/home/radxa/zenith_ws"
AUDIO_DIR = os.path.join(WORKSPACE_DIR, "scripts", "audio")
BEEP_START = os.path.join(AUDIO_DIR, "beep_start.wav")
BEEP_STOP = os.path.join(AUDIO_DIR, "beep_stop.wav")
SCAN_SCRIPT = os.path.join(WORKSPACE_DIR, "scripts", "run_corridor_scan.sh")

def play_audio(filepath, repeat=1, delay=0.15):
    """Play audio file via aplay with fallback to terminal bell."""
    for _ in range(repeat):
        if os.path.exists(filepath):
            subprocess.run(["aplay", "-q", filepath], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            sys.stdout.write('\a')
            sys.stdout.flush()
        if repeat > 1:
            time.sleep(delay)

def get_single_key(timeout=0.1):
    """Read a key or ANSI escape sequence from stdin with timeout."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if not rlist:
            return None
        
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            # Check for escape sequence
            seq = ch
            while True:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not rlist:
                    break
                seq += sys.stdin.read(1)
            return seq
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def decode_key(key_seq):
    """Decode raw key string to human readable name."""
    if key_seq is None:
        return ""
    mapping = {
        '\x1b[6~': 'PAGE_DOWN (Presenter Next / Down)',
        '\x1b[5~': 'PAGE_UP (Presenter Prev / Up)',
        '\x1b[B':  'ARROW_DOWN',
        '\x1b[A':  'ARROW_UP',
        '\x1b[C':  'ARROW_RIGHT',
        '\x1b[D':  'ARROW_LEFT',
        '\x1b[15~': 'F5 (Slide Show Start)',
        '\x1b':    'ESC / Escape',
        '\n':      'ENTER (Return)',
        '\r':      'ENTER (Return)',
        ' ':       'SPACEBAR',
        'b':       'B (Black Screen)',
        'B':       'B (Black Screen)',
    }
    return mapping.get(key_seq, f"Key: {repr(key_seq)}")

def run_test_mode():
    print("==========================================================")
    print("   [TEST MODE] 무선 프레젠터 / 레이저 리모컨 입력 테스트")
    print("==========================================================")
    print("리모컨의 아무 버튼이나 눌러보세요 (다음, 이전, 레이저 등).")
    print("버튼을 누를 때마다 화면에 키 이름이 출력되고 비프음이 울립니다.")
    print("테스트를 종료하려면 Ctrl+C 또는 'q'를 누르세요.")
    print("----------------------------------------------------------")
    
    while True:
        try:
            k = get_single_key(timeout=0.2)
            if k:
                if k == 'q' or k == '\x03':  # 'q' or Ctrl+C
                    print("\n[INFO] 테스트 모드를 종료합니다.")
                    break
                key_name = decode_key(k)
                print(f"  >>> 버튼 감지됨: {key_name} (Raw: {repr(k)})")
                play_audio(BEEP_START, repeat=1)
        except KeyboardInterrupt:
            print("\n[INFO] 테스트 종료.")
            break

def main():
    if "--test" in sys.argv or "-t" in sys.argv:
        run_test_mode()
        return

    print("==========================================================")
    print("   Zenith Drone: 무선 리모컨(프레젠터) 원격 스캔 컨트롤러")
    print("==========================================================")
    print("사용법:")
    print("  1. 핸드캐리 리그를 들고 원하는 출발점으로 걸어갑니다.")
    print("  2. 출발점에서 리모컨 버튼을 [딸깍!] 1번 누르면 녹화가 시작됩니다.")
    print("     (삐-! 시작 신호음 울림)")
    print("  3. 복도 스캔을 마친 뒤 리모컨 버튼을 [딸깍!] 다시 누르면 종료됩니다.")
    print("     (삐- 삐-! 2번 종료 및 저장 완료 신호음 울림)")
    print("----------------------------------------------------------")
    print("※ 프로그램을 강제 종료하려면 키보드에서 [Ctrl+C]를 누르세요.")
    print("==========================================================")

    scan_proc = None
    last_trigger_time = 0.0
    DEBOUNCE_SEC = 2.5  # Ignore duplicate clicks within 2.5s

    print("\n[대기 중] 리모컨 버튼 입력을 기다리고 있습니다...")
    sys.stdout.flush()

    try:
        while True:
            k = get_single_key(timeout=0.2)
            now = time.time()

            if k:
                # Ctrl+C from keyboard
                if k == '\x03':
                    raise KeyboardInterrupt
                
                # Debounce check
                if now - last_trigger_time < DEBOUNCE_SEC:
                    continue
                last_trigger_time = now

                key_name = decode_key(k)
                print(f"\n[리모컨 신호 감지] {key_name}")

                if scan_proc is None:
                    # START RECORDING
                    print("==========================================================")
                    print(">>> [녹화 시작] LIVO2 센서 스캔 백파일 녹화 시작! <<<")
                    print("==========================================================")
                    play_audio(BEEP_START, repeat=1)

                    # Launch run_corridor_scan.sh 1 in new process group
                    scan_proc = subprocess.Popen(
                        [SCAN_SCRIPT, "1"],
                        preexec_fn=os.setpgrp,
                        cwd=WORKSPACE_DIR
                    )
                    print("\n[녹화 진행 중] 복도를 걸으시면서 스캔하세요...")
                    print("스캔을 마치면 리모컨 버튼을 다시 한번 누르세요.\n")
                    sys.stdout.flush()

                else:
                    # STOP RECORDING
                    print("==========================================================")
                    print(">>> [녹화 종료] 정지 신호 전송 중... 백파일 안전 저장 중... <<<")
                    print("==========================================================")
                    
                    try:
                        # Send SIGINT to process group (Ctrl+C equivalent)
                        os.killpg(os.getpgid(scan_proc.pid), signal.SIGINT)
                        scan_proc.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        print("[경고] 프로세스가 15초 내에 종료되지 않아 강제 종료합니다.")
                        os.killpg(os.getpgid(scan_proc.pid), signal.SIGTERM)
                        scan_proc.wait()
                    except Exception as e:
                        print(f"[오류] 종료 처리 중 예외 발생: {e}")

                    scan_proc = None
                    print(">>> [완료] 백파일 및 PCD 저장이 완료되었습니다! <<<")
                    play_audio(BEEP_STOP, repeat=2, delay=0.15)
                    print("----------------------------------------------------------")
                    print("[대기 중] 다음 스캔을 시작하려면 다시 리모컨 버튼을 누르세요...")
                    sys.stdout.flush()

    except KeyboardInterrupt:
        print("\n\n[INFO] 컨트롤러를 종료합니다.")
        if scan_proc is not None:
            print("[INFO] 실행 중인 스캔 프로세스를 안전하게 종료합니다...")
            try:
                os.killpg(os.getpgid(scan_proc.pid), signal.SIGINT)
                scan_proc.wait(timeout=10)
            except Exception:
                pass
        print("[INFO] 정상 종료되었습니다.")

if __name__ == "__main__":
    main()
