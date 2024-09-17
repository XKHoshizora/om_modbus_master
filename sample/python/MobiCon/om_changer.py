#!/usr/bin/env python3

import rospy
from om_modbus_master.msg import om_query, om_response, om_state
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from math import sin, cos

# Global variables
gState_driver = 0
gState_mes = 0
gState_error = 0

x_spd = 0
y_spd = 0
z_ang = 0
odm_x = 0.0
odm_y = 0.0
odm_th = 0.0

def message_cb(twist):
    global x_spd, y_spd, z_ang
    x_spd = int(twist.linear.x * 1000.0)
    y_spd = int(twist.linear.y * 1000.0)
    z_ang = int(twist.angular.z * 1000000.0)

def res_callback(msg):
    global odm_x, odm_y, odm_th
    if msg.slave_id == 1:
        odm_x = msg.data[0] / 1000.0
        odm_y = msg.data[1] / 1000.0
        odm_th = msg.data[2] / 1000000.0

def state_callback(msg):
    global gState_driver, gState_mes, gState_error
    gState_driver = msg.state_driver
    gState_mes = msg.state_mes
    gState_error = msg.state_error

def wait():
    rospy.sleep(0.03)
    start_time = rospy.get_time()
    while gState_driver == 1 and not rospy.is_shutdown():
        rospy.sleep(0.01)  # Short sleep to prevent CPU hogging
        if rospy.get_time() - start_time > 5.0:  # 5 seconds timeout
            rospy.logwarn("Timeout waiting for driver state to change")
            break

def init(pub):
    msg = om_query()
    msg.slave_id = 0x00
    msg.func_code = 1
    msg.write_addr = 124
    msg.write_num = 1
    msg.data = [0] * 64  # Initialize with 64 zeros
    msg.data[0] = 0  # Set the first element to 0
    pub.publish(msg)
    wait()

def main_loop(pub, odom_pub, odom_broadcaster):
    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        msg = om_query()
        msg.slave_id = 0x01
        msg.func_code = 2
        msg.read_addr = 4928
        msg.read_num = 3
        msg.write_addr = 4960
        msg.write_num = 4
        msg.data = [0] * 64  # Initialize with 64 zeros
        msg.data[:4] = [1, int(x_spd), int(z_ang), int(y_spd)]  # Ensure all values are integers
        pub.publish(msg)

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, odm_th)

        odom_broadcaster.sendTransform(
            (odm_x, odm_y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = odm_x
        odom.pose.pose.position.y = odm_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = x_spd / 1000.0
        odom.twist.twist.linear.y = y_spd / 1000.0
        odom.twist.twist.angular.z = z_ang / 1000000.0

        odom_pub.publish(odom)

        wait()
        rate.sleep()

def main():
    rospy.init_node('om_ros_node', anonymous=True)
    pub = rospy.Publisher('om_query1', om_query, queue_size=1)
    rospy.Subscriber('om_response1', om_response, res_callback)
    rospy.Subscriber('om_state1', om_state, state_callback)

    odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()
    rospy.Subscriber('cmd_vel', Twist, message_cb)

    rospy.sleep(1.0)
    init(pub)

    print("START")

    # Initial setup
    msg = om_query()
    msg.slave_id = 0x01
    msg.func_code = 1
    msg.write_addr = 4864
    msg.write_num = 32
    msg.data = [0] * 64  # Initialize with 64 zeros
    msg.data[:32] = [1069, 1070, 1071] + [0] * 13 + [993, 994, 995, 996] + [0] * 12
    pub.publish(msg)
    wait()

    # Start the main loop in a separate thread
    import threading
    threading.Thread(target=main_loop, args=(pub, odom_pub, odom_broadcaster), daemon=True).start()

    # Use rospy.spin() to keep the main thread alive
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass