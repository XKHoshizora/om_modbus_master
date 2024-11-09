/** @file    amr_ros_bridge.cpp
 *  @brief   ROS节点，用于处理机器人的里程计数据并发布TF变换
 *
 *  @details 该节点主要完成以下任务：
 *           1. 接收cmd_vel命令并将其写入到Modbus设备
 *           2. 从Modbus设备读取里程计数据
 *           3. 发布里程计数据（odom话题和TF变换）
 */

#include "ros/ros.h"
#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_state.h"

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mutex>

/* 全局变量 */
int gState_driver = 0; // 通信状态标志（0:可通信，1:通信中）
int gState_mes = 0;    // 消息状态（0:无消息，1:消息到达，2:消息错误）
int gState_error = 0;  // 错误状态（0:无错误，1:无响应，2:异常响应）

std::mutex odom_mutex;

// 里程计数据
double x_spd = 0.0; // X方向速度 [mm/s]
double y_spd = 0.0; // Y方向速度 [mm/s]
double z_ang = 0.0; // 角速度 [rad/s]
double odm_x = 0.0; // X方向位置 [m]
double odm_y = 0.0; // Y方向位置 [m]
double odm_th = 0.0; // 偏航角 [rad]

void messageCb(const geometry_msgs::Twist& twist) {
    std::lock_guard<std::mutex> lock(odom_mutex);
    x_spd = int(twist.linear.x * 1000.0);  // 转换为mm/s
    y_spd = int(twist.linear.y * 1000.0);  // 转换为mm/s
    z_ang = int(twist.angular.z * 1000000.0);  // 转换为μrad/s

    // 添加调试输出
    ROS_INFO("Received velocity command - x: %f, y: %f, z: %f", twist.linear.x, twist.linear.y, twist.angular.z);
}

void resCallback(const om_modbus_master::om_response msg) {
    if (msg.slave_id == 1) {
        std::lock_guard<std::mutex> lock(odom_mutex);
        odm_x = msg.data[0] / 1000.0;  // 转换为m
        odm_y = msg.data[1] / 1000.0;  // 转换为m
        odm_th = msg.data[2] / 1000000.0;  // 转换为rad
    }
}

void stateCallback(const om_modbus_master::om_state msg) {
    gState_driver = msg.state_driver;
    gState_mes = msg.state_mes;
    gState_error = msg.state_error;
}

bool waitForResponse(void) {
    ros::Duration(0.03).sleep();
    ros::spinOnce();

    ros::Time start_time = ros::Time::now();
    while (gState_driver == 1) {
        if ((ros::Time::now() - start_time).toSec() > 1.0) {
            ROS_WARN_THROTTLE(1.0, "Communication timeout");
            return false;
        }
        ros::spinOnce();
    }
    return true;
}

void init(om_modbus_master::om_query msg, ros::Publisher pub) {
    int init_retry = 0;
    const int MAX_INIT_RETRY = 5;

    while (init_retry < MAX_INIT_RETRY) {
        msg.slave_id = 0x00;   // 广播
        msg.func_code = 1;     // 写入功能
        msg.write_addr = 124;  // 驱动器输入命令地址
        msg.write_num = 1;     // 写入1个32位数据
        msg.data[0] = 0;       // 所有位置为OFF
        pub.publish(msg);

        if (waitForResponse()) {
            ROS_INFO("Device initialized successfully");
            return;
        }

        ROS_WARN("Init failed, retrying... (%d/%d)", init_retry + 1, MAX_INIT_RETRY);
        ros::Duration(1.0).sleep();
        init_retry++;
    }

    ROS_ERROR("Failed to initialize device after %d attempts", MAX_INIT_RETRY);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "om_ros_node");
    ros::NodeHandle n;

    double update_rate;
    n.param("update_rate", update_rate, 20.0);  // 默认20Hz

    // 创建发布者和订阅者
    ros::Publisher pub = n.advertise<om_modbus_master::om_query>("om_query1", 1);
    ros::Subscriber sub1 = n.subscribe("om_response1", 1, resCallback);
    ros::Subscriber sub2 = n.subscribe("om_state1", 1, stateCallback);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf2_ros::TransformBroadcaster odom_broadcaster;
    ros::Subscriber sub = n.subscribe("cmd_vel", 1, &messageCb);

    om_modbus_master::om_query msg;

    ros::Duration(1.0).sleep();
    init(msg, pub);
    ros::Rate loop_rate(update_rate);

    ROS_INFO("AMR ROS Bridge Start");

    // 设置间接引用地址
    msg.slave_id = 0x01;
    msg.func_code = 1;
    msg.write_addr = 4864;
    msg.write_num = 32;
    msg.data[0] = 1069;  // 里程计X
    msg.data[1] = 1070;  // 里程计Y
    msg.data[2] = 1071;  // 里程计Theta
    msg.data[16] = 993;  // 直接数据运行模式
    msg.data[17] = 994;  // 前后平移速度(Vx)
    msg.data[18] = 995;  // 角速度(ω)
    msg.data[19] = 996;  // 左右平移速度(Vy)

    pub.publish(msg);
    if (!waitForResponse()) {
        ROS_ERROR("Failed to set initial parameters");
    }

    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();

        // 写入速度命令并读取数据
        msg.slave_id = 0x01;
        msg.func_code = 2;
        msg.read_addr = 4928;
        msg.read_num = 3;  // 只读取里程计数据
        msg.write_addr = 4960;
        msg.write_num = 4;
        {
            std::lock_guard<std::mutex> lock(odom_mutex);
            msg.data[0] = 1;
            msg.data[1] = x_spd;
            msg.data[2] = z_ang;
            msg.data[3] = y_spd;

            // 添加调试输出
            ROS_DEBUG("Sending command - x_spd: %f, y_spd: %f, z_ang: %f", x_spd, y_spd, z_ang);
        }

        pub.publish(msg);
        if (!waitForResponse()) {
            ROS_WARN_THROTTLE(1.0, "Communication timeout, retrying...");
            ros::Duration(0.1).sleep();
            continue;
        }

        // 创建并发布TF变换
        tf2::Transform odom_tf;
        tf2::Quaternion q;
        {
            std::lock_guard<std::mutex> lock(odom_mutex);
            odom_tf.setOrigin(tf2::Vector3(odm_x, odm_y, 0.0));
            q.setRPY(0, 0, odm_th);
            odom_tf.setRotation(q);
        }

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        tf2::convert(odom_tf, odom_trans.transform);
        odom_broadcaster.sendTransform(odom_trans);

        // 创建并发布Odometry消息
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = odom_tf.getOrigin().x();
        odom.pose.pose.position.y = odom_tf.getOrigin().y();
        odom.pose.pose.position.z = odom_tf.getOrigin().z();
        tf2::convert(odom_tf.getRotation(), odom.pose.pose.orientation);

        {
            std::lock_guard<std::mutex> lock(odom_mutex);
            odom.twist.twist.linear.x = x_spd / 1000.0;
            odom.twist.twist.linear.y = y_spd / 1000.0;
            odom.twist.twist.angular.z = z_ang / 1000000.0;
        }

        odom_pub.publish(odom);

        ROS_INFO_THROTTLE(1, "Current position: x=%f, y=%f, theta=%f, speeds: x=%f, y=%f, z=%f",
                         odm_x, odm_y, odm_th, x_spd/1000.0, y_spd/1000.0, z_ang/1000000.0);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // 退出前发送停止命令
    msg.slave_id = 0x01;
    msg.func_code = 1;
    msg.write_addr = 4960;
    msg.write_num = 4;
    msg.data[0] = 0;
    msg.data[1] = 0;
    msg.data[2] = 0;
    msg.data[3] = 0;
    pub.publish(msg);
    waitForResponse();

    return 0;
}