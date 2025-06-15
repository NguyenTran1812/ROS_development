#!/usr/bin/env python3

import os
import yaml
import subprocess
from datetime import datetime

# Topics cần ghi
ROS_TOPICS = [
    "/fmu/out/vehicle_odometry",
    "/fmu/out/vehicle_attitude",
    "/fmu/out/sensor_combined",
    "/fmu/out/distance_sensor",
    #"/fmu/out/battery_status",
    "/fmu/out/vehicle_local_position",
    "/vehicle_height",
    "/sim/accel_raw",
    "/sim/gyro_raw",
    "/sim/mag_raw",
    "/sim/baro_altitude_raw",
]

def main():
    print("=== Drone Data Recorder ===")

    # 1. Nhập thông tin
    #environment = input("Môi trường (indoor/outdoor): ").strip()
    #location = input("Vị trí bay (ví dụ: test_lab_a1): ").strip()
    gps = input("Có GPS? (yes/no): ").strip().lower()
    #light = input("Điều kiện ánh sáng (ví dụ: artificial_stable): ").strip()
    #notes = input("Ghi chú thêm: ").strip()

    gps_signal = True if gps in ["yes", "y"] else False

    # 2. Tạo thư mục
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = os.path.expanduser(f"~/rosbags/session_{timestamp}")
    os.makedirs(session_dir, exist_ok=True)

    # 3. Tạo metadata.yaml
    metadata = {
        #"environment": environment,
        #"location": location,
        "gps_signal": gps_signal,
        #"light_condition": light,
        #"notes": notes,
    }

    metadata_path = os.path.join(session_dir, "metadata.yaml")
    with open(metadata_path, "w") as f:
        yaml.dump(metadata, f)

    print(f" Metadata đã lưu tại: {metadata_path}")

    # 4. Tạo lệnh ros2 bag
    bag_output = os.path.join(session_dir, "flight_data")
    # Đường dẫn tới file qos_overrides.yaml (nằm cùng thư mục script)
    qos_config_path = os.path.join(os.path.dirname(__file__), "qos_overrides.yaml")
    if not os.path.exists(qos_config_path):
        print(f"[ERROR] Không tìm thấy file QoS override: {qos_config_path}")
        return
    # Lệnh ros2 bag record kèm profile QoS
    cmd = [
        "ros2", "bag", "record",
        "-o", bag_output,
        "--qos-profile-overrides-path", qos_config_path
    ] + ROS_TOPICS


    print(f" Bắt đầu ghi rosbag... (Nhấn Ctrl+C để dừng)")
    print(f" Ghi vào thư mục: {session_dir}")
    print(f" Các topic: {', '.join(ROS_TOPICS)}")

    subprocess.run(cmd)
    print(f" Ghi hoàn tất. Dữ liệu nằm trong: {session_dir}")
    info_path = os.path.join(session_dir, "info.txt")
    with open(info_path, "w") as f:
        subprocess.run(["ros2", "bag", "info", bag_output], stdout=f)
    print(f" Thông tin rosbag lưu tại: {info_path}")

if __name__ == "__main__":
    main()

