from datetime import datetime
from threading import Thread, Timer, Event

from flask import Flask, render_template, request, jsonify, session
from flask_caching import Cache
from flask_socketio import SocketIO, emit
from pymavlink import mavutil

import csv
import logging
import math
import os
import struct
import subprocess
import sys
import time

import serial
import serial.tools.list_ports

app = Flask(__name__)
app.secret_key = 'AUAVDRONE'
os.environ['MAVLINK20'] = '1'
flight_modes = {
    0: 'STABILIZE',
    1: 'ACRO',
    2: 'ALTHOLD',
    3: 'AUTO',
    4: 'GUIDED',
    5: 'LOITER',
    6: 'RTL',
    7: 'CIRCLE',
    9: 'LAND',
    11: 'DRIFT',
    13: 'SPORT',
    14: 'FLIP',
    15: 'AUTOTUNE',
    16: 'POSHOLD',
    17: 'BRAKE',
    18: 'THROW',
    19: 'AVOID_ADSB',
    20: 'GUIDED_NOGPS',
    21: 'SMART_RTL',
    22: 'FLOWHOLD',
    23: 'FOLLOW',
    24: 'ZIGZAG',
    25: 'SYSTEMID',
    26: 'HELI_AUTOROTATE',
    27: 'AUTO RTL'
}

app.config['CACHE_TYPE'] = 'simple'
baud_rate = 115200
drone_infos = {}
drone_timeout_timers = {}
testing_motors = {}
socketio = SocketIO(app)
global_connection = None
global_rtk_connection = None
motor_test_duration = 30
stop_battery_logging = Event()
rtk_status = {'satellites_visible': 0, 'survey_in_progress': True, 'rtk_precision': None}
log = logging.getLogger('werkzeug')
log.setLevel(logging.INFO)

class LogHandler(logging.Handler):
    def emit(self, record):
        log_entry = self.format(record)
        socketio.emit('log_message', {'data': log_entry})
        print(log_entry)

log.addHandler(LogHandler())

def startTimer(interval, function, *args, **kwargs):
    timer = Timer(interval, function, args, kwargs)
    timer.start()
    return timer

def setup_connection(com_port):
    return mavutil.mavlink_connection(com_port, baud=baud_rate)

def setup_rtk_connection(com_port, baud_rate):
    return serial.Serial(com_port, baudrate=baud_rate, timeout=1)
    
def listen_to_drones(connection):
    while True:
        msg = connection.recv_match(type=['SYS_STATUS', 'GPS_RAW_INT', 'HEARTBEAT'], blocking=False)
        if not msg:
            continue
        drone_id = msg.get_srcSystem()
        update_drone_status(drone_id, msg)

def read_and_send_rtk_data(connection, rtk_connection):
    sequence_id = 0
    msglen = 100
    while True:
        data = rtk_connection.read(1024)
        for i in range(0, len(data), msglen):
            chunk = data[i:i + msglen]
            start_time = time.time()
            print(f'Received {len(chunk)}, GOOD!')
            msgs = math.ceil(len(chunk) / msglen)
            for msg in range(msgs):
                flags = 0
                if msgs > 1:
                    flags = 1
                flags |= (msg & 0x3) << 1
                flags |= sequence_id << 3
                amount = min(len(chunk) - msg * msglen, msglen)
                datachunk = chunk[msg * msglen : msg * msglen + amount]
                
                connection.mav.gps_rtcm_data_send(
                    flags,
                    len(datachunk),
                    bytearray(datachunk.ljust(180, b'\0'))
                )
            if msgs < 4 and len(chunk) % msglen == 0 and len(chunk) > msglen:
                flags = 1 | (msgs & 0x3) << 1 | sequence_id << 3
                connection.mav.gps_rtcm_data_send(
                    flags,
                    0,
                    bytearray(''.ljust(180, b'\0'))
                )
            sequence_id = (sequence_id + 1) & 0x1f

def update_drone_status(drone_id, msg):
    drone_timeout_timer = drone_timeout_timers.pop(drone_id, None)
    if drone_timeout_timer:
        drone_timeout_timer.cancel()
    drone_info = drone_infos.get(drone_id, None)
    changed = False
    if not drone_info:
        changed = True
        drone_info = {
            'id': drone_id,
            'status': 'ok',
            'mode': '--',
            'voltage': '--',
            'current': '--',
            'gps_type': '--',
            'gps_coords': ('--', '--')
        }
        drone_infos[drone_id] = drone_info
    if msg.get_type() == 'HEARTBEAT':
        mode = flight_modes.get(msg.custom_mode, 'Unknown')
        if drone_info['mode'] != mode:
            changed = True
            drone_info['mode'] = mode
    elif msg.get_type() == 'SYS_STATUS':
        voltage = msg.voltage_battery / 1000.0
        current = msg.current_battery / 100.0
        if drone_info['voltage'] != voltage or drone_info['current'] != current:
            changed = True
            drone_info['voltage'] = voltage
            drone_info['current'] = current
    elif msg.get_type() == 'GPS_RAW_INT':
        gps_type = msg.fix_type
        gps_coords = (msg.lat / 1e7, msg.lon / 1e7)
        if drone_info['gps_type'] != gps_type or drone_info['gps_coords'] != gps_coords:
            changed = True
            drone_info['gps_type'] = gps_type
            drone_info['gps_coords'] = gps_coords

    if changed:
        socketio.emit('drone_info', drone_info)
    drone_timeout_timers[drone_id] = startTimer(3, mark_drone_disconnected, drone_id, drone_info, 'timeout')

def send_motor_test_command(connection, drone_id, motor_instance, throttle, duration):
    connection.mav.command_long_send(
        drone_id,
        1,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,
        motor_instance,
        mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,
        throttle,
        duration,
        0,
        0,
        0
    )

def mark_drone_disconnected(drone_id, drone_info, reason):
    drone_info['status'] = 'disconnected'
    drone_info['error'] = reason
    socketio.emit('drone_disconnected', drone_id, reason)

def send_led_control_message(drone_id, r, g, b, duration_msec, flashes):
    target_component = 1
    instance = 42
    pattern = 42
    custom_bytes = [r, g, b, duration_msec & 0xff, duration_msec >> 8]
    custom_len = len(custom_bytes)
    custom_bytes.extend([0] * (24 - custom_len))
    
    for _ in range(flashes):
        global_connection.mav.led_control_send(
            drone_id, target_component, instance, pattern, custom_len, custom_bytes
        )
        time.sleep(duration_msec / 1000 + 0.5)

def get_active_drones():
    return [drone_id for drone_id, drone_info in drone_infos.items() if drone_info['status'] == 'ok']

def clean_testing_motors():
    now = time.time()
    for d, motors in testing_motors.items():
        for m, deadline in motors.items():
            if deadline < now:
                del motors[m]

def stop_all_testing_motors():
    if not global_connection:
        return
    for d, motors in testing_motors.items():
        for m, deadline in motors.items():
            send_motor_test_command(global_connection, d, m, 0, 0)

@app.route('/list_ports', methods=['GET'])
def list_ports():
    ports = list(serial.tools.list_ports.comports())
    port_list = [{'device': port.device, 'description': port.description} for port in ports]
    return jsonify(port_list)

@app.route('/')
def index():
    return render_template('test1.html', flight_modes=flight_modes)

@app.route('/set_com_port', methods=['POST'])
def set_com_port():
    global global_connection
    data = request.json
    com_port = data.get('com_port')
    global_connection = setup_connection(com_port)
    thread = Thread(target=listen_to_drones, name='listen_to_drones', args=(global_connection,))
    thread.start()
    socketio.emit('log_message', {'data': f'COM port {com_port} set'})
    return jsonify(success=True, message='COM port set and listening started.')

@app.route('/set_rtk_port', methods=['POST'])
def set_rtk_port():
    global global_rtk_connection
    data = request.json
    rtk_com_port = data.get('rtk_com_port')
    baudrate = int(data.get('baudrate'))
    if rtk_com_port:
        global_rtk_connection = setup_rtk_connection(rtk_com_port, baudrate)
        rtk_thread = Thread(target=read_and_send_rtk_data, name='read_and_send_rtk_data', args=(global_connection, global_rtk_connection,))
        rtk_thread.start()
    socketio.emit('log_message', {'data': f'RTK port {rtk_com_port} set with baudrate {baudrate} and listening started.'})
    return jsonify(success=True, message='RTK port set and listening started.')

@app.route('/motor_test', methods=['POST'])
def motor_test():
    motor_id = request.form['motor_id']
    drone_id = request.form.get('drone_id')
    throttle = float(request.form['throttle'])

    if not global_connection:
        return jsonify(success=False, message='Not connected')
    drones = []
    motors = []
    if drone_id == 'all':
        socketio.emit('log_message', {'data': f'send to all drones start'})
        drones = get_active_drones()
    else:
        drones = [int(drone_id)]
    if motor_id == 'all':
        motors = range(1, 5)
    else:
        motors = [int(motor_id)]
    global motor_test_duration
    duration = motor_test_duration
    deadline = time.time() + duration
    for d in drones:
        for m in motors:
            if d in testing_motors:
                tsting_motors = testing_motors[d]
            else:
                tsting_motors = {}
                testing_motors[d] = tsting_motors
            tsting_motors[m] = deadline
            send_motor_test_command(global_connection, d, m, throttle, duration)
    startTimer(duration, clean_testing_motors)
    return jsonify(success=True, message='Motor test command sent.')


@app.route('/change_mode', methods=['POST'])
def change_mode():
    mode_id = int(request.form['mode'])
    drone_id = request.form.get('drone_id')
    if not global_connection:
        return jsonify(success=False, message='Connection not established.')
    drones = []
    if drone_id == 'all':
        socketio.emit('log_message', {'data': f'send to all start'})
        drones = get_active_drones()
    else:
        drones = [int(drone_id)]
    try:
        for d in drones:
            global_connection.mav.set_mode_send(
                d,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id)
            socketio.emit('log_message', {'data': f'Mode change command received: DroneID: {drone_id}  ModeID: {mode_id}  Mode: {flight_modes.get(mode_id, 'Unknown')}'})
        return jsonify(success=True, message='Mode change command sent.')
    except Exception as e:
        return jsonify(success=False, message=str(e))

@app.route('/change_color', methods=['POST'])
def change_color():
    data = request.get_json()
    if not data:
        return jsonify({'success': False, 'message': 'Invalid JSON data'})
    color = data['rgb']
    duration = data['duration']
    flashes = data['flashes']
    drone_id = data.get('drone_id', '1')
    r = color['r']
    g = color['g']
    b = color['b']
    duration = int(duration)
    flashes = int(flashes)
    drones = []
    if drone_id == 'all':
        socketio.emit('log_message', {'data': f'send to all start'})
        drones = get_active_drones()
    else:
        socketio.emit('log_message', {'data': f'send to one start'})
        drones = [int(drone_id)]
    for d in drones:
        send_led_control_message(d, r, g, b, duration, flashes)
        socketio.emit('log_message', {'data': f'Color change command received: DroneID: {d}  RGB: ({r}, {g}, {b}), Duration: {duration} ms, Flashes: {flashes}'})

    return jsonify(success=True, message='Color change command sent.')

@app.route('/get_rtk_status', methods=['GET'])
def get_rtk_status():
    return jsonify(success=True, rtk_status=rtk_status)

@app.route('/selected_block', methods=['POST'])
def handle_selected_block():
    data = request.get_json()
    drone_id = data['droneId']
    session['drone_id'] = drone_id
    return jsonify(message=f'Received and saved drone ID: {drone_id}')

if __name__ == '__main__':
    try:
        socketio.run(app, debug=True)
    finally:
        stop_all_testing_motors()
