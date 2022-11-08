#!/usr/bin/env python3
import math
import time

import rospy
import serial
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def unsigned_short_2_signed_short(data: int):
    max_int = 2 ** (8 * 2 - 1) - 1
    signed_data = data - (2 * (max_int + 1) if data > max_int else 0)
    return signed_data


class GoldenTankCom:
    def __init__(self, port: str, baudrate: int):
        self.serial_port = serial.Serial(
            port=port,
            baudrate=baudrate,
        )

        # Wait serial communication establish
        time.sleep(1)

        # Make sure the wheel doesn't run at the beginning
        self.send_cmd(speed_l=0, speed_r=0)


    def send_cmd(self, speed_l: int, speed_r: int):
        mask = 0xffff  # 2 bytes
        msg_l = (speed_l & mask).to_bytes(2, byteorder="big")
        msg_r = (speed_r & mask).to_bytes(2, byteorder="big")
        msg = msg_l + msg_r
        print(f"Sending: {speed_l}(L) {speed_r}(R) {msg}")
        self.serial_port.write(data=msg)

    def recv_stat(self):
        res_raw = self.serial_port.readline()
        if len(res_raw) == 5:
            encoder_speed_l = int((res_raw[0] << 8) | res_raw[1])
            encoder_speed_r = int((res_raw[2] << 8) | res_raw[3])

            encoder_speed_l = unsigned_short_2_signed_short(encoder_speed_l)
            encoder_speed_r = unsigned_short_2_signed_short(encoder_speed_r)

            return encoder_speed_l, encoder_speed_r
        else:
            return None


class GoldenTankBringupNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('golden_tank_bringup_node')

        cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        odom_topic = rospy.get_param('~odom_topic', '/golden_tank/wheel_odom')
        self.tread_dist = rospy.get_param('~tread_distance', 0.2333863)
        self.pulse_per_rev = rospy.get_param('~pulse_per_rev', 50 * 127)
        self.wheel_diameter = rospy.get_param('~wheel_d', 0.042355)

        # Subscribers
        self.cmd_sub = rospy.Subscriber(cmd_vel_topic, Twist, self.receive_cmd)

        # Publishers
        self.odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=1)

        # Services

        # Node variables
        self.com = GoldenTankCom(
            port="/dev/serial/by-id/usb-Teensyduino_USB_Serial_11062550-if00",
            baudrate=115200
        )
        self._odom = Odometry()
        self._odom.header.frame_id = "odom"
        self._odom.child_frame_id = "base_link"
        self._odom.twist.covariance[0] = 0.01

        # Node main logics
        while not rospy.is_shutdown():
            tread_stat = self.com.recv_stat()
            if tread_stat:
                encoder_speed_l, encoder_speed_r = tread_stat
                self._odom.header.stamp = rospy.Time.now()
                self._odom.twist.twist = self.tread_speed_2_twist(encoder_speed_l, encoder_speed_r)
                self.odom_pub.publish(self._odom)

    def tread_speed_2_twist(self, speed_l, speed_r):
        out_twist = Twist()
        avg_speed = (speed_l + speed_r) / 2
        diff_speed = speed_r - speed_l
        wheel_perimeter = self.wheel_diameter * math.pi
        out_twist.linear.x = avg_speed / self.pulse_per_rev * wheel_perimeter
        out_twist.angular.z = diff_speed / self.pulse_per_rev * wheel_perimeter / self.tread_dist
        return out_twist

    def receive_cmd(self, data: Twist):
        # Calculate the tread speeds in m/s
        speed_l = data.linear.x
        speed_r = data.linear.x
        angular_adjustment = self.tread_dist * data.angular.z / 2
        speed_l -= angular_adjustment
        speed_r += angular_adjustment

        # Convert the tread speeds to pulse/s
        wheel_perimeter = (self.wheel_diameter * math.pi)
        speed_l = speed_l / wheel_perimeter  # revolution/sec
        speed_r = speed_r / wheel_perimeter  # revolution/sec
        speed_l = speed_l * self.pulse_per_rev  # pulse/s
        speed_r = speed_r * self.pulse_per_rev  # pulse/s

        self.com.send_cmd(speed_l=int(speed_l), speed_r=int(speed_r))


if __name__ == '__main__':
    try:
        GoldenTankBringupNode()
    except rospy.ROSInterruptException:
        pass
