import time
import math
from pymavlink import mavutil
import ctypes
import threading
import serial

class CustomThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs={}):
        threading.Thread.__init__(self, group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        return
    
    def run(self):
        self._target(*self.args, **self.kwargs)

    def get_id(self):
        if hasattr(self, '_thread_id'):
            return self._thread_id
        for id, thread in threading._active.items():
            if thread is self:
                return id
    
    def raise_exception(self):
        thread_id = self.get_id()
        resu = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), ctypes.py_object(SystemExit))
        if resu > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), 0)
            print('Failure in raising exception')

#座標情報出力
def check_gpi(master):
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        1,
        1
    )

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg is not None:
            print(msg)
        time.sleep(5)

#命令送信
def send_msg_rc(master, roll=0, pitch=0, throttle=0, yaw=0):
    while True:
        master.mav.rc_channels_override_send(
            master.target_system, 
            master.target_component, 
            roll, pitch, throttle, yaw, 0, 0, 0, 0) # R, P, Th, Y
        time.sleep(0.2)

#
def print_msg_AHRS2(master):
    msg = master.mav.command_long_encode(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,  # confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_AHRS2,
        0,0,0,0,0,0
    )
    master.mav.send(msg)
    msg = master.recv_match(type="AHRS2", blocking=True)
    print(msg)

#下向きLiDARの情報出力
def print_msg_DISTANCE_SENSOR(master):
    msg = master.mav.command_long_encode(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,  # confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
        0,0,0,0,0,0
    )

    master.mav.send(msg)
    msg = master.recv_match(type="DISTANCE_SENSOR", blocking=True)
    # print(msg)
    # print(msg.current_distance)
    distance = msg.current_distance / 100
    print(distance)
    return distance

def check_direction_pi(master):
    msg = master.mav.command_long_encode(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,  # confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_AHRS2,
        0,0,0,0,0,0
    )
    master.mav.send(msg)
    msg = master.recv_match(type="AHRS2", blocking=True)
    yaw = msg.yaw
    degrees = math.degrees(yaw)
    print(degrees)
    return degrees

#着陸
def land(vehicle):
    msg = vehicle.mav.command_long_encode(
        vehicle.target_system, vehicle.target_component,   # target system, target component
        mavutil.mavlink.MAV_CMD_NAV_LAND, # frame
        0,
        0,0,0,0,0,0,0
    )

    vehicle.mav.send(msg)
    msg = vehicle.recv_match(type="COMMAND_ACK", blocking=True)
    print(msg)

# def stab(vehicle):
#     msg = vehicle.mav.command_long_encode(
#         vehicle.target_system, vehicle.target_component,   # target system, target component
#         mavutil.mavlink.MAV_CMD_NAV_STABILIZE, # frame
#         0,
#         0,0,0,0,0,0,0
#     )

#     vehicle.mav.send(msg)
#     msg = vehicle.recv_match(type="COMMAND_ACK", blocking=True)
#     print(msg)

def command(c, th_send_msg_rc=None):
    flag_send_msg_rc = False
    if c == "a":
        c = "idle"
        print("ARM")
        master.arducopter_arm()
        msg = master.recv_match(type="COMMAND_ACK", blocking=True)
        print(f"result = {msg.result}")
    
    if c == "d":
        c = "idle"
        print("DISARM")
        master.arducopter_disarm()
        msg = master.recv_match(type="COMMAND_ACK", blocking=True)
        print(f"result = {msg.result}")

    if c == "up":
        throttle = 2000
        roll = 1500 # お試し
        pitch = 1500
        yaw = 1500
        flag_send_msg_rc = True
    if c == "hov":
        throttle = 1625
        roll = 1500 # お試し
        pitch = 1500
        yaw = 1500
        flag_send_msg_rc = True
    if c == "land":
        land(master)
        # throttle = 1600
        # roll = 1500 # お試し
        # pitch = 1500
        # yaw = 1500
        #flag_send_msg_rc = True
    if c == "idle":
        roll = 1500
        pitch = 1500
        throttle = 0
        yaw = 1500
        flag_send_msg_rc = True
    if c == "right":
        roll = 2000
        flag_send_msg_rc = True
    if c == "left":
        roll = 1000
        flag_send_msg_rc = True
    if c == "forward":
        pitch = 1000
        flag_send_msg_rc = True
    if c == "back":
        pitch = 2000    
        flag_send_msg_rc = True
    if c == "turnr":
        yaw = 1600
        flag_send_msg_rc = True
    if c == "turnl":
        yaw = 1400
        flag_send_msg_rc = True

    if c == "pa":
        print_msg_AHRS2(master)
    if c == "pd":
        print_msg_DISTANCE_SENSOR(master)

    if flag_send_msg_rc:
        if th_send_msg_rc is not None:
            th_send_msg_rc.raise_exception()
            th_send_msg_rc.join()
            th_send_msg_rc = None
        if roll + pitch + throttle + yaw > 0:
            th_send_msg_rc = CustomThread(
                name="send_msg_rc", target=send_msg_rc, 
                args=(master,),
                kwargs={"roll": roll, "pitch": pitch, "throttle": throttle, "yaw": yaw})
            th_send_msg_rc.start()
        else:
            master.mav.rc_channels_override_send(
                master.target_system, 
                master.target_component, 
                0, 0, 0, 0, 0, 0, 0, 0)
        
        flag_send_msg_rc = False

    return th_send_msg_rc

def commands(master, th_send_msg_rc, dict_params):
    if th_send_msg_rc is not None:
        th_send_msg_rc.raise_exception()
        th_send_msg_rc.join()
        th_send_msg_rc = None
    th_send_msg_rc = CustomThread(
        name="send_msg_rc", target=send_msg_rc, 
        args=(master,),
        kwargs=dict_params)
    th_send_msg_rc.start()
    return th_send_msg_rc

try:
    # SITLへの接続
    master = mavutil.mavlink_connection('/dev/ttyACM0')
    master.wait_heartbeat()

    flag_send_msg_rc = False
    th_send_msg_rc = None
    dict_params = {"roll": 1500, "pitch": 1500, "throttle": 0, "yaw": 1500}

    #起動arm
    th_send_msg_rc = command("a")
    time.sleep(1)

    #上昇takeoff
    dict_params["throttle"] = 1550
    th_send_msg_rc = commands(master, th_send_msg_rc, dict_params)

    time.sleep(3)

    dict_params["throttle"] = 1500
    th_send_msg_rc = commands(master, th_send_msg_rc, dict_params)

    time.sleep(3)

    th_send_msg_rc.raise_exception()
    th_send_msg_rc.join()

    exit(1)

    print("test done")

    if th_send_msg_rc is not None:
        th_send_msg_rc.raise_exception()
        th_send_msg_rc.join()
    th_send_msg_rc.raise_exception()
    th_send_msg_rc.join()
except KeyboardInterrupt:
    try:
        th_send_msg_rc.raise_exception()
        th_send_msg_rc.join()
    except:
        pass
