from flask import Flask, render_template, request, jsonify, session
from pymavlink import mavutil
from flask_caching import Cache
from threading import Thread, Timer
from flask_socketio import SocketIO, emit
import subprocess
import logging
import serial
import struct
import serial.tools.list_ports
import time

app = Flask(__name__)
app.secret_key = 'AUAVDRONE'
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

log.addHandler(LogHandler())

# def read_terminal_output():
#     process = subprocess.Popen(['tail', '-f', '/path/to/your/logfile.log'], stdout=subprocess.PIPE)
#     while True:
#         output = process.stdout.readline().decode('utf-8')
#         if output:
#             socketio.emit('terminal_output', {'data': output})

def setup_connection(com_port):
    global global_connection
    baud_rate = 115200
    global_connection = mavutil.mavlink_connection(com_port, baud=baud_rate)

def setup_rtk_connection(com_port, baud_rate):
    global global_rtk_connection
    global_rtk_connection = serial.Serial(com_port, baudrate=baud_rate, timeout=1)

def listen_to_drones():
    while True:
        if global_connection:
            msg = global_connection.recv_match(type=['SYS_STATUS', 'GPS_RAW_INT', 'HEARTBEAT'], blocking=False)
            if msg:
                drone_id = msg.get_srcSystem()
                update_drone_status(drone_id, msg)
        time.sleep(0.1)

def check_mode_until_changed(drone, expected_mode, max_attempts=10):
    print("###################################################")
    expected_mode_name = flight_modes.get(expected_mode, "Unknown")
    attempts = 0
    while attempts < max_attempts:
        msg = drone.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg:
            current_mode = mavutil.mode_string_v10(msg)
            if current_mode == expected_mode_name:
                print("Mode change confirmed.")
                print("Current Mode:", current_mode)
                break
        else:
            print("No HEARTBEAT message received.")
        attempts += 1
        time.sleep(3)

def listen_to_rtk():
    global rtk_status
    while True:
        if global_rtk_connection:
            data = global_rtk_connection.read(1024)
            if data:
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

def send_rtk_data_to_drones(data):
    if global_connection:
        max_packet_size = 180
        sequence_id = 0
        for i in range(0, len(data), max_packet_size):
            chunk = data[i:i + max_packet_size]
            flags = (sequence_id << 3)
            sequence_id += 1

            if len(chunk) < max_packet_size:
                chunk += b'\x00' * (max_packet_size - len(chunk))

            global_connection.mav.gps_rtcm_data_send(
                flags,
                len(chunk),
                chunk
            )

def read_and_send_rtk_data():
    sequence_id = 0
    msglen = 180

    while True:
        data = global_rtk_connection.read(1024)
        for i in range(0, len(data), msglen * 4):
            chunk = data[i:i + msglen * 4]
            start_time = time.time()
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
    # print(f"Updated drone status: {drone_info}")
    Timer(30, check_drone_timeout, [drone_id]).start()

def check_drone_timeout(drone_id):
    current_time = time.time()
    if drone_id in last_update_time and (current_time - last_update_time[drone_id]) > 30:
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

# @socketio.on('connect')
# def handle_connect():
#     Thread(target=read_terminal_output).start()

@app.route('/set_com_port', methods=['POST'])
def set_com_port():
    data = request.json
    com_port = data.get('com_port')
    setup_connection(com_port)
    thread = Thread(target=listen_to_drones)
    thread.start()
    socketio.emit('log_message', {'data': f"COM port {com_port} set and listening started."})
    return jsonify(success=True, message="COM port set and listening started.")

@app.route('/set_rtk_port', methods=['POST'])
def set_rtk_port():
    data = request.json
    rtk_com_port = data.get('rtk_com_port')
    baudrate = int(request.form['baudrate'])
    
    if rtk_com_port:
        setup_rtk_connection(rtk_com_port, baudrate)
        rtk_thread = Thread(target=read_and_send_rtk_data)
        rtk_thread.start()
    socketio.emit('log_message', {'data': f"RTK port {rtk_com_port} set with baudrate {baudrate} and listening started."})
    return jsonify(success=True, message="RTK port set and listening started.")

@app.route('/rtl', methods=['POST'])
def rtl():
    if global_connection:
        try:
            global_connection.mav.command_long_send(
                global_connection.target_system,
                global_connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                0, 0, 0, 0, 0, 0, 0, 0)
            return jsonify(success=True, message="RTL command sent.")
        except Exception as e:
            return jsonify(success=False, message=str(e))
    return jsonify(success=False, message="Connection not established.")

@app.route('/takeOff', methods=['POST'])
def takeOff():
    if global_connection:
        try:
            global_connection.mav.command_long_send(
                global_connection.target_system,
                global_connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, 0)
            return jsonify(success=True, message="Take Off command sent.")
        except Exception as e:
            return jsonify(success=False, message=str(e))
    return jsonify(success=False, message="Connection not established.")

@app.route('/land', methods=['POST'])
def land():
    if global_connection:
        try:
            global_connection.mav.command_long_send(
                global_connection.target_system,
                global_connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0)
            return jsonify(success=True, message="Land command sent.")
        except Exception as e:
            return jsonify(success=False, message=str(e))
    return jsonify(success=False, message="Connection not established.")

@app.route('/change_mode', methods=['POST'])
def change_mode():
    mode_id = int(request.form['mode'])
    if global_connection:
        try:
            global_connection.mav.set_mode_send(
                global_connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id)
            check_mode_until_changed(global_connection, mode_id)
            return jsonify(success=True, message="Mode change command sent.")
        except Exception as e:
            return jsonify(success=False, message=str(e))
    return jsonify(success=False, message="Connection not established.")

@app.route('/set_color', methods=['POST'])
def set_color():
    data = request.get_json()
    if not data:
        return jsonify({"success": False, "message": "Invalid JSON data"})
    color_hex = data['color']
    duration = int(data['duration'])
    flashes = int(data['flashes'])
    drone_id = data.get('droneId', 'all')
    r, g, b = int(color_hex[1:3], 16), int(color_hex[3:5], 16), int(color_hex[5:7], 16)
    session['rgb'] = (r, g, b)
    session['duration'] = duration
    session['flashes'] = flashes
    session['drone_id'] = drone_id
    return jsonify({"success": True, "message": f"Color set to RGB: ({r}, {g}, {b}), Duration: {duration} ms, Flashes: {flashes}"})

@app.route('/change_color', methods=['POST'])
def change_color():
    r, g, b = session.get('rgb', (255, 0, 0))
    duration = session.get('duration', 500)
    flashes = session.get('flashes', 1)
    drone_id = session.get('drone_id', '1')
    if drone_id == 'all':
        drones = get_drones()
        for d in drones:
            send_led_control_message(d, r, g, b, duration, flashes)
    else:
        send_led_control_message(drone_id, r, g, b, duration, flashes)
    return jsonify(success=True, message="Color change command sent.")

@app.route('/turn_on_light', methods=['POST'])
def turn_on_light():
    drone_id = session.get('drone_id', '1')
    if drone_id == 'all':
        drones = get_drones()
        for d in drones:
            send_led_control_message(d, 0, 255, 0, 65534, 1)
    else:
        send_led_control_message(drone_id, 0, 255, 0, 65534, 1)
    return jsonify(success=True, message="Turn on light command sent.")

@app.route('/turn_off_light', methods=['POST'])
def turn_off_light():
    drone_id = session.get('drone_id', '1')
    if drone_id == 'all':
        drones = get_drones()
        for d in drones:
            send_led_control_message(d, 0, 0, 0, 65534, 1)
    else:
        send_led_control_message(drone_id, 0, 0, 0, 65534, 1)
    return jsonify(success=True, message="Turn off light command sent.")

@app.route('/get_battery_status', methods=['GET'])
def get_battery_status():
    drone_ids = get_drones()
    battery_statuses = {drone_id: cache.get(f'drone_status_{drone_id}') for drone_id in drone_ids}
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
