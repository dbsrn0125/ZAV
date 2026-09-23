#!/usr/bin/env python3
import time
from periphery import GPIO

CHIP = "/dev/gpiochip4"
LINE = 99  # Physical Pin 40

print(f"[TEST] Opening Pin 40 (Chip: {CHIP}, Line: {LINE})...")
gpio = GPIO(CHIP, LINE, "out")

try:
    print("[TEST] Setting Pin 40 to LOW (0V)...")
    gpio.write(False)
    time.sleep(1.0)

    print("[TEST] >>> Setting Pin 40 to HIGH (3.3V) for 3 seconds! <<<")
    gpio.write(True)
    time.sleep(3.0)

    print("[TEST] >>> Setting Pin 40 back to LOW (0V)... <<<")
    gpio.write(False)
    time.sleep(1.0)

finally:
    gpio.write(False)
    gpio.close()
    print("[TEST] Done. Pin 40 closed.")
