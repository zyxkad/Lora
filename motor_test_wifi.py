import os
import time
import csv
import threading
from datetime import datetime
from pymavlink import mavutil
import pandas as pd
import matplotlib.pyplot as plt

stop_battery_logging = threading.Event()
connection = None

def send_motor_test_command(connection, motor_id, throttle, duration):
    connection.mav.command_long_send(
        0,
        0,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,
        motor_id,
        mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,
        throttle,
        duration,
        0,
        0,
        0
    )
    # print(f"Sent motor test command to motor {motor_id}")

def listen_and_log_battery_info(connection, filename):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['timestamp', 'drone_id', 'voltage', 'current'])

        while not stop_battery_logging.is_set():
            if connection:
                msg = connection.recv_match(type='SYS_STATUS', blocking=False)
                if msg:
                    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    drone_id = msg.get_srcSystem()
                    voltage = msg.voltage_battery / 1000.0
                    current = msg.current_battery / 100.0

                    writer.writerow([timestamp, drone_id, voltage, current])
                    file.flush()
            time.sleep(0.1)

def visualize_battery_data(data, output_dir, drone_id, timestamp):
    data['timestamp'] = pd.to_datetime(data['timestamp'])
    file_prefix = os.path.join(output_dir, f'battery_info_{drone_id}_{timestamp}')

    plt.figure(figsize=(12, 6))
    plt.plot(data['timestamp'], data['voltage'], label='Voltage', color='b')
    plt.xlabel('Timestamp')
    plt.ylabel('Voltage (V)')
    plt.title(f'Drone {drone_id} Battery Voltage Over Time')
    plt.legend()
    plt.grid(True)
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.savefig(f'{file_prefix}_voltage.png')
    plt.close()

    plt.figure(figsize=(12, 6))
    plt.plot(data['timestamp'], data['current'], label='Current', color='r')
    plt.xlabel('Timestamp')
    plt.ylabel('Current (A)')
    plt.title(f'Drone {drone_id} Battery Current Over Time')
    plt.legend()
    plt.grid(True)
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.savefig(f'{file_prefix}_current.png')
    plt.close()

def split_and_visualize_battery_data(base_folder, filename):
    data = pd.read_csv(filename)
    drone_ids = data['drone_id'].unique()
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    for drone_id in drone_ids:
        drone_data = data.loc[data['drone_id'] == drone_id].copy()
        drone_folder = os.path.join(base_folder, f'drone_{drone_id}_batteryinfo_{timestamp}')
        os.makedirs(drone_folder, exist_ok=True)
        drone_filename = os.path.join(drone_folder, f'battery_info_{drone_id}_{timestamp}.csv')
        drone_data.to_csv(drone_filename, index=False)
        visualize_battery_data(drone_data, drone_folder, drone_id, timestamp)

def main():
    global connection
    current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
    base_folder = current_time

    connection = mavutil.mavlink_connection('192.168.0.251:14550')
    connection.wait_heartbeat()

    drone_ids = set()
    start_time = time.time()
    print("Scanning for drones...")
    while time.time() - start_time < 10:
        msg = connection.recv_match(type='SYS_STATUS', blocking=False)
        if msg:
            drone_id = msg.get_srcSystem()
            drone_ids.add(drone_id)
        time.sleep(0.1)

    user_input = input(f"Do you want to proceed with testing drones with IDs {list(drone_ids)}? (y/n): ")
    if user_input.lower() != 'y':
        print("Test aborted.")
        return

    if not os.path.exists(base_folder):
        os.makedirs(base_folder)
    filename = os.path.join(base_folder, f'battery_info_{current_time}.csv')

    print("Starting battery logging...")
    battery_logging_thread = threading.Thread(target=listen_and_log_battery_info, args=(connection, filename))
    battery_logging_thread.start()
    time.sleep(5)

    throttle = 60
    duration = 30
    for i in range(2):
        for motor_id in range(1, 5):
            send_motor_test_command(connection, motor_id, throttle, duration)
            time.sleep(0.05)

    time.sleep(duration + 5)

    stop_battery_logging.set()
    battery_logging_thread.join()
    split_and_visualize_battery_data(base_folder, filename)
    print("Test completed.")

def stop_all_testing_motors():
    for i in range(2):
        for motor_id in range(1, 5):
            send_motor_test_command(connection, motor_id, 0, 0)
            time.sleep(0.05)

if __name__ == '__main__':
    try:
        main()
    finally:
        stop_all_testing_motors()
