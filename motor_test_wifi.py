import os
import time
import csv
import threading
from datetime import datetime
from pymavlink import mavutil
import pandas as pd
import matplotlib.pyplot as plt

stop_battery_logging = threading.Event()

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
    print(f"Sent motor test command to motor {motor_id}")

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

def visualize_battery_data(filename):
    data = pd.read_csv(filename)

    data['timestamp'] = pd.to_datetime(data['timestamp'])
    file_prefix = filename.rsplit('.', 1)[0]

    plt.figure(figsize=(12, 6))
    plt.plot(data['timestamp'], data['voltage'], label='Voltage', color='b')
    plt.xlabel('Timestamp')
    plt.ylabel('Voltage (V)')
    plt.title('Drone Battery Voltage Over Time')
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
    plt.title('Drone Battery Current Over Time')
    plt.legend()
    plt.grid(True)
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.savefig(f'{file_prefix}_current.png')
    plt.close()

def main():
    current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
    folder_name = current_time

    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
    filename = os.path.join(folder_name, f'battery_info_{current_time}.csv')

    # Change the IP address to the IP address of the computer running the GCS
    connection = mavutil.mavlink_connection('192.168.0.251:14550')
    connection.wait_heartbeat()

    drone_ids = set()
    start_time = time.time()
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

    print("Starting battery logging...")
    battery_logging_thread = threading.Thread(target=listen_and_log_battery_info, args=(connection, filename))
    battery_logging_thread.start()
    time.sleep(5)

    throttle = 60
    duration = 30
    for i in range(2):
        for motor_id in range(1, 5):
            send_motor_test_command(connection, motor_id, throttle, duration)
            time.sleep(0.01)

    time.sleep(duration + 5)

    stop_battery_logging.set()
    battery_logging_thread.join()
    visualize_battery_data(filename)
    print("Test completed.")

if __name__ == '__main__':
    main()
