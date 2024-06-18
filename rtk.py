from flask import Flask, render_template, request, jsonify, session
from pymavlink import mavutil
from flask_caching import Cache
from threading import Thread, Timer
from flask_socketio import SocketIO, emit
import subprocess
import logging
import os
import csv
from datetime import datetime
import serial
import struct
import serial.tools.list_ports
import time

app = Flask(__name__)
app.secret_key = 'AUAVDRONE'
os.environ['MAVLINK20'] = '1'
flight_modes = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALTHOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    9: "LAND",
    11: "DRIFT",
    13: "SPORT",
    14: "FLIP",
    15: "AUTOTUNE",
    16: "POSHOLD",
    17: "BRAKE",
    18: "THROW",
    19: "AVOID_ADSB",
    20: "GUIDED_NOGPS",
    21: "SMART_RTL",
    22: "FLOWHOLD",
    23: "FOLLOW",
    24: "ZIGZAG",
    25: "SYSTEMID",
    26: "HELI_AUTOROTATE",
    27: "AUTO RTL"
}

#TODO: mavlink v2
app.config['CACHE_TYPE'] = 'simple'
cache = Cache(app)
socketio = SocketIO(app)
global_connection = None
global_rtk_connection = None
rtk_status = {"satellites_visible": 0, "survey_in_progress": True, "rtk_precision": None}
last_update_time = {}
log = logging.getLogger('werkzeug')
log.setLevel(logging.INFO)

class LogHandler(logging.Handler):
    def emit(self, record):
        log_entry = self.format(record)
        socketio.emit('log_message', {'data': log_entry})
        print(log_entry)

log.addHandler(LogHandler())

def setup_connection(com_port):
    global global_connection
    baud_rate = 115200
    global_connection = mavutil.mavlink_connection(com_port, baud=baud_rate)

def setup_rtk_connection(com_port, baud_rate):
    global global_rtk_connection
    global_rtk_connection = serial.Serial(com_port, baudrate=baud_rate, timeout=1)
    print("2")

def listen_to_drones():
    while True:
        if global_connection:
            msg = global_connection.recv_match(type=['SYS_STATUS', 'GPS_RAW_INT', 'HEARTBEAT'], blocking=False)
            if msg:
                drone_id = msg.get_srcSystem()
                update_drone_status(drone_id, msg)
        # time.sleep(0.1)

def listen_to_rtk():
    global rtk_status
    while True:
        if global_rtk_connection:
            data = global_rtk_connection.read(1024)
            if data:
                print(f"Received {len(data)} bytes: {data}")
                try:
                    satellites_visible = data[0]
                    survey_in_progress = data[1] == 1
                    rtk_precision = struct.unpack('<f', data[2:6])[0]
                    rtk_status = {
                        "satellites_visible": satellites_visible,
                        "survey_in_progress": survey_in_progress,
                        "rtk_precision": rtk_precision
                    }
                except Exception as e:
                    print(f"Error parsing RTK data: {e}")
                send_rtk_data_to_drones(data)

def set_message_interval(drone_id):
    if global_connection:
        global_connection.mav.command_long_send(
            drone_id,
            1,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            291,
            1000000,
            0, 0, 0, 0,
            0
        )
        print(f"Message interval set for drone {drone_id}")

def esc_record():
    current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f'esc_status_{current_time}.csv'
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        
        writer.writerow(['timestamp', 'drone_id', 'index', 'rpm1', 'rpm2', 'rpm3', 'rpm4', 'voltage1', 'voltage2', 'voltage3', 'voltage4', 'current1', 'current2', 'current3', 'current4'])
        
        while True:
            if global_connection:
                msg = global_connection.recv_match(type=['ESC_STATUS'], blocking=False)
                print(f"Received ESC_STATUS message: {msg}")
                if msg:
                    
                    timestamp = msg.time_usec
                    drone_id = msg.get_srcSystem()
                    index = msg.index
                    rpm = msg.rpm
                    voltage = msg.voltage
                    current = msg.current

                    writer.writerow([timestamp, drone_id, index] + rpm + voltage + current)
                    
                    file.flush()
                
def send_rtk_data_to_drones(data):
    if global_connection:
        max_packet_size = 180
        sequence_id = 0
        for i in range(0, len(data), max_packet_size):
            chunk = data[i:i + max_packet_size]
            flags = (sequence_id << 3)
            sequence_id += 1
            #TODO: keep sequence_id and send it twice
            if len(chunk) < max_packet_size:
                chunk += b'\x00' * (max_packet_size - len(chunk))

            global_connection.mav.gps_rtcm_data_send(
                flags,
                len(chunk),
                chunk
            )

def read_and_send_rtk_data():
    sequence_id = 0
    msglen = 100
    #TODO: msglen = 100
    while True:
        data = global_rtk_connection.read(1024)
        for i in range(0, len(data), msglen):
            chunk = data[i:i + msglen]
            start_time = time.time()
            print(f"Received {len(chunk)}, GOOD!")
            if (len(chunk) % msglen == 0):
                msgs = len(chunk) // msglen
            else:
                msgs = (len(chunk) // msglen) + 1
            for a in range(msgs):
                flags = 0
                if msgs > 1:
                    flags = 1
                flags |= (a & 0x3) << 1
                flags |= (sequence_id & 0x1f) << 3
                amount = min(len(chunk) - a * msglen, msglen)
                datachunk = chunk[a * msglen : a * msglen + amount]
                
                global_connection.mav.gps_rtcm_data_send(
                    flags,
                    len(datachunk),
                    bytearray(datachunk.ljust(180, b'\0'))
                )
            if msgs < 4 and len(chunk) % msglen == 0 and len(chunk) > msglen:
                flags = 1 | (msgs & 0x3) << 1 | (sequence_id & 0x1f) << 3
                global_connection.mav.gps_rtcm_data_send(
                    flags,
                    0,
                    bytearray("".ljust(180, b'\0'))
                )
            sequence_id += 1

def update_drone_status(drone_id, msg):
    current_time = time.time()
    last_update_time[drone_id] = current_time
    #TODO: actived drone number wrong
    drone_info = cache.get(f'drone_status_{drone_id}')
    if not drone_info:
        drone_info = {
            "id": drone_id,
            "mode": "--",
            "voltage": "--",
            "current": "--",
            "gps_type": "--",
            "gps_coords": ("--", "--")
        }
    if msg.get_type() == 'HEARTBEAT':
        drone_info["mode"] = flight_modes.get(msg.custom_mode, "Unknown")
    elif msg.get_type() == 'SYS_STATUS':
        drone_info["voltage"] = msg.voltage_battery / 1000.0
        drone_info["current"] = msg.current_battery / 100.0
    elif msg.get_type() == 'GPS_RAW_INT':
        drone_info["gps_type"] = msg.fix_type
        drone_info["gps_coords"] = (msg.lat / 1e7, msg.lon / 1e7)

    cache.set(f'drone_status_{drone_id}', drone_info)
    Timer(3 , check_drone_timeout, [drone_id]).start()

def send_motor_test_command(drone_id, motor_instance, throttle):
    global_connection.mav.command_long_send(
        drone_id,
        1,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,
        motor_instance,
        mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,
        throttle,
        10,
        0,
        0,
        0
    )

def check_drone_timeout(drone_id):
    current_time = time.time()
    if drone_id in last_update_time and (current_time - last_update_time[drone_id]) > 3:
        drone_info = {
            "id": drone_id,
            "mode": "--",
            "voltage": "--",
            "current": "--",
            "gps_type": "--",
            "gps_coords": ("--", "--")
        }
        cache.set(f'drone_status_{drone_id}', drone_info)
        print(f"Drone {drone_id} status reset due to timeout: {drone_info}") 

def send_led_control_message(drone_id, r, g, b, duration_msec, flashes):
    target_component = 1
    instance = 42
    pattern = 42
    custom_len = 5
    custom_bytes = [r, g, b, duration_msec % 256, duration_msec // 256] + [0]*19
    
    for _ in range(flashes):
        global_connection.mav.led_control_send(
            drone_id, target_component, instance, pattern, custom_len, custom_bytes
        )
        time.sleep(duration_msec / 1000 + 0.5)

def get_drones():
    drone_status_keys = cache.cache._cache.keys()
    drones = [key.split('_')[-1] for key in drone_status_keys if key.startswith('drone_status_')]
    return drones

@app.route('/list_ports', methods=['GET'])
def list_ports():
    ports = list(serial.tools.list_ports.comports())
    port_list = [{"device": port.device, "description": port.description} for port in ports]
    return jsonify(port_list)

@app.route('/')
def index():
    return render_template('test1.html', flight_modes=flight_modes)

@app.route('/set_com_port', methods=['POST'])
def set_com_port():
    data = request.json
    com_port = data.get('com_port')
    setup_connection(com_port)
    thread = Thread(target=listen_to_drones)
    thread.start()
    socketio.emit('log_message', {'data': f"COM port {com_port} set"})
    return jsonify(success=True, message="COM port set and listening started.")

@app.route('/set_rtk_port', methods=['POST'])
def set_rtk_port():
    data = request.json
    rtk_com_port = data.get('rtk_com_port')
    baudrate = int(data.get('baudrate'))
    print("1")
    if rtk_com_port:
        setup_rtk_connection(rtk_com_port, baudrate)
        rtk_thread = Thread(target=read_and_send_rtk_data)
        rtk_thread.start()
    socketio.emit('log_message', {'data': f"RTK port {rtk_com_port} set with baudrate {baudrate} and listening started."})
    return jsonify(success=True, message="RTK port set and listening started.")

@app.route('/motor_test', methods=['POST'])
def motor_test():
    # for d in get_drones():
    #     set_message_interval(int(d))
    # thread = Thread(target=esc_record)
    # thread.start()
    motor_id = request.form['motor_id']
    drone_id = request.form.get('drone_id')
    throttle = float(request.form['throttle'])
    print(f"Motor test command received: DroneID: {drone_id}  MotorID: {motor_id}  Throttle: {throttle}")
    if global_connection:
        if drone_id == 'all':
            socketio.emit('log_message', {'data': f"send to all drones start"})
            drones = get_drones()
            for d in drones:
                d = int(d)
                if motor_id == 'all':
                    for motor_instance in range(1, 5):
                        send_motor_test_command(d, motor_instance, throttle)
                        # time.sleep(0.2)
                else:
                    send_motor_test_command(d, int(motor_id), throttle)
        else:
            if motor_id == 'all':
                for motor_instance in range(1, 5):
                    send_motor_test_command(int(drone_id), motor_instance, throttle)
                    # time.sleep(0.1)
            else:
                send_motor_test_command(int(drone_id), int(motor_id), throttle)
        return jsonify(success=True, message="Motor test command sent.")

@app.route('/change_mode', methods=['POST'])
def change_mode():
    mode_id = int(request.form['mode'])
    drone_id = request.form.get('drone_id')
    if global_connection:
        try:
            if drone_id == 'all':
                socketio.emit('log_message', {'data': f"send to all start"})
                drones = get_drones()
                for d in drones:
                    d = int(d)
                    global_connection.mav.set_mode_send(
                        d,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        mode_id)
                    socketio.emit('log_message', {'data': f"Mode change command received: DroneID: {drone_id}  ModeID: {mode_id}  Mode: {flight_modes.get(mode_id, 'Unknown')}"})
            else:
                drone_id = int(drone_id)
                global_connection.mav.set_mode_send(
                    drone_id,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id)
                socketio.emit('log_message', {'data': f"Mode change command received: DroneID: {drone_id}  ModeID: {mode_id}  Mode: {flight_modes.get(mode_id, 'Unknown')}"})
            return jsonify(success=True, message="Mode change command sent.")
        except Exception as e:
            return jsonify(success=False, message=str(e))
    return jsonify(success=False, message="Connection not established.")

@app.route('/change_color', methods=['POST'])
def change_color():
    data = request.get_json()
    if not data:
        return jsonify({"success": False, "message": "Invalid JSON data"})
    color = data['rgb']
    duration = data['duration']
    flashes = data['flashes']
    drone_id = data.get('drone_id', '1')
    r = color['r']
    g = color['g']
    b = color['b']
    duration = int(duration)
    flashes = int(flashes)
    if drone_id == 'all':
        socketio.emit('log_message', {'data': f"send to all start"})
        drones = get_drones()
        for d in drones:
            d = int(d)
            send_led_control_message(d, r, g, b, duration, flashes)
            socketio.emit('log_message', {'data': f"Color change command received: DroneID: {d}  RGB: ({r}, {g}, {b}), Duration: {duration} ms, Flashes: {flashes}"})
    else:
        socketio.emit('log_message', {'data': f"send to one start"})
        drone_id = int(drone_id)
        send_led_control_message(drone_id, r, g, b, duration, flashes)
        socketio.emit('log_message', {'data': f"Color change command received: DroneID: {drone_id}  RGB: ({r}, {g}, {b}), Duration: {duration} ms, Flashes: {flashes}"})

    return jsonify(success=True, message="Color change command sent.")

@app.route('/get_battery_status', methods=['GET'])
def get_battery_status():
    drone_ids = get_drones()
    battery_statuses = {drone_id: cache.get(f'drone_status_{drone_id}') for drone_id in drone_ids}
    print(f"Battery statuses: {battery_statuses}")
    return jsonify(success=True, battery_statuses=battery_statuses)

@app.route('/get_rtk_status', methods=['GET'])
def get_rtk_status():
    return jsonify(success=True, rtk_status=rtk_status)

@app.route('/selected_block', methods=['POST'])
def handle_selected_block():
    data = request.get_json()
    drone_id = data['droneId']
    session['drone_id'] = drone_id
    return jsonify(message=f"Received and saved drone ID: {drone_id}")

if __name__ == '__main__':
    socketio.run(app, debug=True)
