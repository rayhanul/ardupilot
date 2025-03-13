#!/usr/bin/env python

from pymavlink import mavutil
from pymavlink import mavwp
import time
import threading
import string
import sys

import pdb 

# Global variables
target_roll = 1500
target_pitch = 1500
target_yaw = 1500
target_throttle = 1500
target_rc5 = 1500

current_heading = 0
start_heading = 0
target_heading = 0

right_turn_phase = 0
left_turn_phase = 0

FORWARD_DURATION = 5 
THROTTLE_PWM = 1600   
TURN_DEGREES = 90     
TURN_RATE = 30      
YAW_PWM = 1600      
ROLL_LEFT_TURN=1400
ROLL_RIGHT_TURN=1600 



# python /usr/local/bin/mavproxy.py --mav=/dev/tty.usbserial-DN01WM7R --baudrate 57600 --out udp:127.0.0.1:14540 --out udp:127.0.0.1:14550
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
#------------------------------------------------------------------------------------
def set_rc_channel_pwm(id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if id < 1:
        print("Channel does not exist.")
        return

    # We only have 8 channels
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    if id < 9:
        # pdb.set_trace()
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[id - 1] = pwm

        # global master
        # print("Calling rc_channels override send value : {}", rc_channel_values)
        master.mav.rc_channels_override_send(
            master.target_system,  # target_system
            master.target_component,  # target_component
            *rc_channel_values)  # RC channel list, in microseconds.

#------------------------------------------------------------------------------------
def throttle_th():
    global target_roll
    global target_pitch
    global target_yaw
    global target_throttle

    while True:
        # print("inside throttle...")
        set_rc_channel_pwm(1, target_roll)
        set_rc_channel_pwm(2, target_pitch)
        set_rc_channel_pwm(3, target_throttle)
        set_rc_channel_pwm(4, target_yaw)
        time.sleep(0.1)

# ------------------------------------------------------------------------------------
def handle_hud(msg):

    hud_data = (msg.airspeed, msg.groundspeed, msg.heading,
                msg.throttle, msg.alt, msg.climb)
    # print "Aspd\tGspd\tHead\tThro\tAlt\tClimb"
    # print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f" % hud_data

    # print("Alt: %f" %msg.alt)

    global current_heading
    global start_heading
    global target_heading
    global target_roll
    global right_turn_phase
    global left_turn_phase
    global target_throttle
    global target


   

    # print("at handle hud thread")
    # [Implement your code at here]

    current_heading = msg.heading 

# ------------------------------------------------------------------------------------
def read_loop():
    while True:
        # grab a mavlink message
        msg = master.recv_match(blocking=True)

        # handle the message based on its type
        msg_type = msg.get_type()

        if msg_type == "VFR_HUD":
            handle_hud(msg)


def move_forward(duration, throttle_pwm):
    global target_throttle
    target_throttle = throttle_pwm
    time.sleep(duration)
    target_throttle = 1600  # Stop

def turn_right(degrees, turn_rate, yaw_pwm):
    global current_heading, target_yaw, target_roll
    # print("Turning right...")
    start_heading = current_heading
    target_heading = (start_heading + degrees) % 360
    turn_duration = abs(degrees / float(turn_rate))
    # print("Current Heading: {}, Target Heading: {}, Turn Duration: {}",start_heading, target_heading, turn_duration)

    target_roll = yaw_pwm
    time.sleep(turn_duration)
    target_roll = 1500
    # time.sleep(1) 

def perform_right_turn_sequence(forward_duration, turn_degrees, turn_rate, yaw_pwm):
    global right_turn_phase, target_throttle
    if right_turn_phase == 0:
        right_turn_phase = 1
        move_forward(forward_duration, THROTTLE_PWM)
        turn_right(turn_degrees, turn_rate, yaw_pwm)
        move_forward(forward_duration, THROTTLE_PWM)

        time.sleep(forward_duration)
        target_throttle = 1500
        right_turn_phase = 0


def turn_left(degrees, turn_rate, yaw_pwm):
    global current_heading, target_yaw, target_roll
    start_heading = current_heading
    target_heading = (start_heading + degrees) % 360
    turn_duration = abs(degrees / float(turn_rate))
    # print("Current Heading: {}, Target Heading: {}, Turn Duration: {}",start_heading, target_heading, turn_duration)

    target_roll = 1400
    time.sleep(turn_duration)
    target_roll = 1500

def perform_left_turn_sequence(forward_duration, turn_degrees, turn_rate, yaw_pwm):
    global left_turn_phase, target_throttle
    if left_turn_phase == 0:
        left_turn_phase = 1
        move_forward(forward_duration, THROTTLE_PWM)
        turn_left(turn_degrees, turn_rate, yaw_pwm)
        move_forward(forward_duration, THROTTLE_PWM)

        time.sleep(forward_duration)
        target_throttle = 1500

        left_turn_phase = 0






#------------------------------------------------------------------------------------
def main(argv):
    global target_roll
    global target_pitch
    global target_yaw
    global target_throttle
    global target_rc5

    global current_heading
    global start_heading
    global target_heading

    global right_turn_phase
    global left_turn_phase
    global target

    print(sys.version)

    master.wait_heartbeat()
    print("HEARTBEAT OK\n")

    goal_throttle = 1500


    t = threading.Thread(target=throttle_th, args=())
    t.daemon = True
    t.start()

    t2 = threading.Thread(target=read_loop, args=())
    t2.daemon = True
    t2.start()
    
    target = ""

    while True:
        # request data to be sent at the given rate
        for i in range(0, 3):
            master.mav.request_data_stream_send(master.target_system, master.target_component,
                                                mavutil.mavlink.MAV_DATA_STREAM_ALL, 30, 1)
            
        if target == "":
            target = raw_input("[forward/stop/right/left] ")
            print('%s' %target)

        if target == "forward":
            # [Implement your code at here]

            target_throttle =1600

            time.sleep(5)
            print("forwarding...")
            target = ""
            # target_throttle = 1500

        elif target == "stop":
            target_throttle=1500
            time.sleep(5)
            target = ""
        elif target == "right" and right_turn_phase != 1:
            # [Implement your code at here]

            perform_right_turn_sequence(FORWARD_DURATION, TURN_DEGREES, TURN_RATE, ROLL_RIGHT_TURN)
            target=""

            

        elif target == "left" and left_turn_phase != 1:
            # [Implement your code at here]
            perform_left_turn_sequence(FORWARD_DURATION, TURN_DEGREES, TURN_RATE, ROLL_LEFT_TURN)
            target=""

        else:
            print("Invalid command!, please provide the currect command.")

            
        

        time.sleep(0.1)

if __name__ == "__main__":
   main(sys.argv[1:])