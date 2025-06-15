#!/usr/bin/env python3

import subprocess
import time

def main():
    commands = [
        #"echo -1 > /sys/devices/platform/soc/a84000.qcom,qup_uart/power/autosuspend_delay_ms"
        #"MicroXRCEAgent serial --dev /dev/ttyHS2 -b 460800"
        #"MicroXRCEAgent udp4 -p 8888",
        # Add more commands if needed
        # "cd ~/PX4-Autopilot && make px4_sitl gazebo-classic",
        # "cd ~/QGroundControl && ./QGroundControl.AppImage"
    ]

    for idx, command in enumerate(commands):
        session_name = f"process_{idx}"
        subprocess.run(["screen", "-dmS", session_name, "bash", "-c", command])
        time.sleep(1)

if __name__ == "__main__":
    main()

