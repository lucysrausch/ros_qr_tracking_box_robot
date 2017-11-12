#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist, Vector3
from sessel_otter.msg import MotorTicks
import serial
import struct

speed = 0
direction = 0
timeout = 0
ser = None

TIMEOUT      = 10
MAX_SPEED    = 1 # m/s
MAX_STEERING = 0.1 # rad/s

import threading


def receiveSerial ():
    r = rospy.Rate(250) # Hz
    global ser
    pub = rospy.Publisher('motor_ticks', MotorTicks, queue_size = 1)
    while not rospy.is_shutdown():
        try:
            response = ser.readline()
            ticks = map(int, response.split(';'))
            #print(ticks[0])
            #print(ticks[1])
            pub.publish(MotorTicks(ticks[0], ticks[1]))
        except Exception as ex:
            print ex
            pass
        r.sleep()


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def callback(data):
    global speed, direction, timeout
    speed = clamp(data.linear.x, -MAX_SPEED, MAX_SPEED)
    direction = clamp(data.angular.z, -MAX_STEERING, MAX_STEERING)
    timeout = 0
    #rospy.loginfo("Speed: %f", speed)

def main():
    global speed, direction, timeout, ser
    rospy.init_node('hardware_driver')

    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        rospy.logwarn("Using serial interface: %s", ser.name)
        connected = True
    except Exception:
        rospy.logerr("No Serial device found!")
        connected = False
        pass

    rospy.Subscriber("drive_command", Twist, callback)
    r = rospy.Rate(100) # Hz
    vel_tp = [0] * 50 # 50 sample low-pass for speed
    dir_tp = [0] * 10 # 10 sample low-pass for steering

    thread = threading.Thread(target=receiveSerial, args=[])
    thread.start()

    while not rospy.is_shutdown():
        vel_tp[len(vel_tp)-1] = speed if not timeout > TIMEOUT else 0
        vel_tp[:-1] = vel_tp[1:]

        dir_tp[len(dir_tp)-1] = direction
        dir_tp[:-1] = dir_tp[1:]

        tx_speed = sum(vel_tp)/len(vel_tp)
        tx_dir = sum(dir_tp)/len(dir_tp)

        #rospy.loginfo("Speed: %f", tx_speed)
        #rospy.loginfo("Steering: %f", tx_dir)

        motorR = tx_speed + tx_dir
        motorL= tx_speed - tx_dir

        binR = struct.pack('f', motorR)
        binL = struct.pack('f', motorL)

        if connected:
            for b in binR:
                ser.write(b)
            for b in binL:
                ser.write(b)

        timeout+=1
        r.sleep()

main()
