#!/usr/bin/env python3
import time
from periphery import GPIO

# Radxa Dragon Q6A 40-pin Header:
# Pin 39: GND           -> Black Wire (-)
# Pin 40: GPIO (Line 99)-> White Wire (+)

OUTPUT_PIN_CHIP = "/dev/gpiochip4"
OUTPUT_PIN_NUMBER = 99  # Physical Pin 40

print("==================================================")
print("  Hikrobot Camera 10Hz Hardware Trigger Generator")
print("==================================================")
print(f"Target: Physical Pin 40 (Chip: {OUTPUT_PIN_CHIP}, Line: {OUTPUT_PIN_NUMBER})")
print("Ground: Physical Pin 39 (GND)")
print("Pulse:  10ms HIGH (3.3V), 90ms LOW (0V) -> 10.0 Hz")
print("Press Ctrl+C to stop.\n")

try:
    gpio_out = GPIO(OUTPUT_PIN_CHIP, OUTPUT_PIN_NUMBER, "out")
except Exception as e:
    print(f"[ERROR] Failed to open GPIO: {e}")
    print("Please make sure python3-periphery is installed and run with sudo if permission is denied.")
    exit(1)

interval = 0.100       # 100ms = 10.0 Hz
pulse_width = 0.010    # 10ms high trigger pulse

try:
    next_time = time.time()
    count = 0
    print("[Trigger] Running 10Hz pulses...")
    while True:
        # Trigger HIGH (3.3V) -> Camera shutter initiates
        gpio_out.write(True)
        time.sleep(pulse_width)

        # Trigger LOW (0V) -> Settle
        gpio_out.write(False)

        count += 1
        if count % 50 == 0:
            print(f"[Trigger] {count} pulses emitted (~{count // 10} seconds)")

        # Precise non-drifting interval timing
        next_time += interval
        sleep_dur = next_time - time.time()
        if sleep_dur > 0:
            time.sleep(sleep_dur)
        else:
            next_time = time.time()

except KeyboardInterrupt:
    print("\n[Trigger] Interrupted by user (Ctrl+C).")
finally:
    gpio_out.write(False)
    gpio_out.close()
    print("[Trigger] Pin 40 released and safely set to LOW.")
