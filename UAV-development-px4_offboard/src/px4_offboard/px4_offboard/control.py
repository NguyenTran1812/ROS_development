#!/usr/bin/env python3
import sys,os

import geometry_msgs.msg
import rclpy
import std_msgs.msg

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. 
Using the arrow keys and WASD you have Mode 2 RC controls.
w: Up
x: Down
a: Yaw Left
d: Yaw Right
s: release Height/Yaw
i: Pitch Forward
,: Pitch Backward
j: Roll Left
l: Roll Right
k: release Pitch/Roll

Press SPACE to arm/disarm the drone
"""

moveBindings = {
    'w': (0, 0, 1, 0), #Z+
    'x': (0, 0, -1, 0),#Z-
    'a': (0, 0, 0, -1), #Yaw+
    'd': (0, 0, 0, 1),#Yaw-
    'i' : (0, 1, 0, 0),  #Up Arrow
    ',' : (0, -1, 0, 0), #Down Arrow
    'j' : (-1, 0, 0, 0), #Right Arrow
    'l' : (1, 0, 0, 0),  #Left Arrow
}


speedBindings = {
    # 'q': (1.1, 1.1),
    # 'z': (.9, .9),
    # 'w': (1.1, 1),
    # 'x': (.9, 1),
    # 'e': (1, 1.1),
    # 'c': (1, .9),
}

def getKey(settings):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        if key == '\x1b':  # possible arrow key
            key += sys.stdin.read(2)
        if settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    import os
    if not os.isatty(sys.stdin.fileno()):
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32' or old_settings is None: 
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )


    pub = node.create_publisher(geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos_profile)

    arm_toggle = False
    arm_pub = node.create_publisher(std_msgs.msg.Bool, '/arm_message', qos_profile)


    speed = 0.5
    turn = .2
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0
    yaw_val = 0.0

    try:
        print(msg)
        # print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            if key == ' ':  # ASCII value for space
                arm_toggle = not arm_toggle  # Flip the value of arm_toggle
                arm_msg = std_msgs.msg.Bool()
                arm_msg.data = arm_toggle
                arm_pub.publish(arm_msg)
                os.system('clear')
                print(msg)
                print(f"Arm toggle is now: {arm_toggle}")
            if key == 'k':
                x_val = 0.0
                y_val = 0.0
            if key == 's':
                z_val = 0.0
                yaw_val = 0.0
            twist = geometry_msgs.msg.Twist()
            
            x_val = (x * speed) + x_val
            y_val = (y * speed) + y_val
            z_val = (z * speed) + z_val
            yaw_val = (th * turn) + yaw_val
            twist.linear.x = x_val
            twist.linear.y = y_val
            twist.linear.z = z_val
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = yaw_val
            pub.publish(twist)
            os.system('clear')
            print(msg)
            print(f"Arm toggle is now: {arm_toggle}") 
            print("X:",twist.linear.x, "   Y:",twist.linear.y, "   Z:",twist.linear.z, "   Yaw:",twist.angular.z)
            

    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
