/** @file    amr_ros_bridge.cpp
 *  @brief   ROS节点，用于处理机器人的里程计和IMU数据并发布TF变换
 *
 *  @details 该节点主要完成以下任务：
 *           1. 接收cmd_vel命令并将其写入到Modbus设备
 *           2. 从Modbus设备读取里程计和IMU数据
 *           3. 发布里程计数据（odom话题和TF变换）
 *           4. 发布IMU数据（imu话题）
 */

#include "ros/ros.h"
#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_state.h"

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
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
const int MAX_RETRY_COUNT = 3;  // 最大重试次数
const double RETRY_DELAY = 0.1; // 重试延时(秒)

std::mutex odom_mutex;
std::mutex imu_mutex;

// 里程计数据
double x_spd = 0.0; // X方向速度 [mm/s]
double y_spd = 0.0; // Y方向速度 [mm/s]
double z_ang = 0.0; // 角速度 [rad/s]
double odm_x = 0.0; // X方向位置 [m]
double odm_y = 0.0; // Y方向位置 [m]
double odm_th = 0.0; // 偏航角 [rad]

// IMU数据
double imu_acc_x = 0.0;  // X方向加速度 [m/s^2]
double imu_acc_y = 0.0;  // Y方向加速度 [m/s^2]
double imu_acc_z = 0.0;  // Z方向加速度 [m/s^2]
double imu_gyro_x = 0.0; // Roll角速度 [rad/s]
double imu_gyro_y = 0.0; // Pitch角速度 [rad/s]
double imu_gyro_z = 0.0; // Yaw角速度 [rad/s]

/**
 * @brief 处理接收到的速度命令
 * @param twist 接收到的Twist消息
 */
void messageCb(const geometry_msgs::Twist& twist) {
    std::lock_guard<std::mutex> lock(odom_mutex);
    x_spd = int(twist.linear.x * 1000.0);  // 转换为mm/s
    y_spd = int(twist.linear.y * 1000.0);  // 转换为mm/s
    z_ang = int(twist.angular.z * 1000000.0);  // 转换为μrad/s
}

/**
 * @brief 处理从Modbus设备接收到的响应
 * @param msg 接收到的响应消息
 */
void resCallback(const om_modbus_master::om_response msg) {
    if (msg.slave_id == 1) {
        {
            std::lock_guard<std::mutex> lock(odom_mutex);
            odm_x = msg.data[0] / 1000.0;  // 转换为m
            odm_y = msg.data[1] / 1000.0;  // 转换为m
            odm_th = msg.data[2] / 1000000.0;  // 转换为rad
        }
        {
            std::lock_guard<std::mutex> lock(imu_mutex);
            // 转换IMU数据到正确的单位
            imu_acc_x = msg.data[3] * 0.001;  // 转换为m/s^2
            imu_acc_y = msg.data[4] * 0.001;  // 转换为m/s^2
            imu_acc_z = msg.data[5] * 0.001;  // 转换为m/s^2
            imu_gyro_x = msg.data[6] * 0.000001;  // 转换为rad/s
            imu_gyro_y = msg.data[7] * 0.000001;  // 转换为rad/s
            imu_gyro_z = msg.data[8] * 0.000001;  // 转换为rad/s
        }
    }
}

/**
 * @brief 处理Modbus通信状态
 * @param msg 接收到的状态消息
 */
void stateCallback(const om_modbus_master::om_state msg) {
    gState_driver = msg.state_driver;
    gState_mes = msg.state_mes;
    gState_error = msg.state_error;
}

/**
 * @brief 等待通信完成，带有超时和重试机制
 * @return bool 通信是否成功
 */
bool waitForResponse(void) {
    int retry_count = 0;
    while (retry_count < MAX_RETRY_COUNT) {
        ros::Duration(0.05).sleep();  // 增加基础延时
        ros::spinOnce();

        if (gState_driver == 0) {
            return true;  // 通信成功
        }

        // 如果驱动忙，等待后重试
        ROS_DEBUG("Driver busy, retrying... (%d/%d)", retry_count + 1, MAX_RETRY_COUNT);
        ros::Duration(RETRY_DELAY).sleep();
        retry_count++;
    }
    return false;  // 通信失败
}

void init(om_modbus_master::om_query msg, ros::Publisher pub) {
    int init_retry = 0;
    const int MAX_INIT_RETRY = 5;  // 初始化最大重试次数

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
        ros::Duration(1.0).sleep();  // 初始化失败后等待较长时间
        init_retry++;
    }

    ROS_ERROR("Failed to initialize device after %d attempts", MAX_INIT_RETRY);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "om_ros_node");
    ros::NodeHandle n;

    // 从参数服务器获取参数
    double update_rate;
    n.param("update_rate", update_rate, 50.0);  // 默认更新率为20Hz

    // 创建发布者和订阅者
    // om_modbus 相关
    ros::Publisher pub = n.advertise<om_modbus_master::om_query>("om_query1", 1);
    ros::Subscriber sub1 = n.subscribe("om_response1", 1, resCallback);
    ros::Subscriber sub2 = n.subscribe("om_state1", 1, stateCallback);
    // 里程计相关
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    // IMU 相关
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 50);
    // cmd_vel 相关
    ros::Subscriber sub = n.subscribe("cmd_vel", 1, &messageCb);

    // 创建里程计 TF 转换广播器
    tf2_ros::TransformBroadcaster odom_broadcaster;

    om_modbus_master::om_query msg;

    // 等待系统稳定
    ros::Duration(1.0).sleep();

    // 初始化设备
    init(msg, pub);
    ros::Rate loop_rate(update_rate);

    ROS_INFO("AMR ROS Bridge Start");

    // 设置间接引用地址
    msg.slave_id = 0x01;
    msg.func_code = 1;
    msg.write_addr = 4864;
    msg.write_num = 32;
    // ---------里程计数据读取---------
    msg.data[0] = 1069;  // 里程计X
    msg.data[1] = 1070;  // 里程计Y
    msg.data[2] = 1071;  // 里程计Theta
    // ---------IMU数据读取---------
    msg.data[3] = 1038;  // 加速度X
    msg.data[4] = 1039;  // 加速度Y
    msg.data[5] = 1040;  // 加速度Z
    msg.data[6] = 1041;  // 角速度ROLL
    msg.data[7] = 1042;  // 角速度PITCH
    msg.data[8] = 1043;  // 角速度YAW
    // ---------里程计IMU融合数据读取---------
    // msg.data[9] = 1032;  // 陀螺仪融合里程计X
    // msg.data[10] = 1033;  // 陀螺仪融合里程计Y
    // msg.data[11] = 1037;  // 陀螺仪融合里程计YAW
    // ... 其他数据保持不变 ...
    msg.data[16] = 993;  // 直接数据运行模式
    msg.data[17] = 994;  // 前后平移速度(Vx)
    msg.data[18] = 995;  // 角速度(ω)
    msg.data[19] = 996;  // 左右平移速度(Vy)
    // ... 其他数据保持为0 ...

    pub.publish(msg);
    if (!waitForResponse()) {
        ROS_ERROR("Failed to set initial parameters");
    }

    // 主循环
    bool first_read = true;
    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();

        // 如果是第一次读取或者上次通信成功，才发送新的请求
        if (first_read || gState_driver == 0) {
            // 写入速度命令
            msg.slave_id = 0x01;
            msg.func_code = 2;
            msg.read_addr = 4928;
            msg.read_num = 9;  // 3个里程计数据 + 6个IMU数据
            msg.write_addr = 4960;
            msg.write_num = 4;
            {
                std::lock_guard<std::mutex> lock(odom_mutex);
                msg.data[0] = 1;
                msg.data[1] = x_spd;
                msg.data[2] = z_ang;
                msg.data[3] = y_spd;
            }
            pub.publish(msg);
            first_read = false;

            // 等待响应，但不阻塞太久
            if (!waitForResponse()) {
                ROS_WARN_THROTTLE(1.0, "Communication timeout");
                ros::Duration(0.1).sleep();
                continue;
            }
        }

        // 创建并发布odom到base_footprint的TF变换
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

        // 创建并发布IMU消息
        sensor_msgs::Imu imu;
        imu.header.stamp = current_time;
        imu.header.frame_id = "imu_link";

        {
            std::lock_guard<std::mutex> lock(imu_mutex);
            // 设置线性加速度
            imu.linear_acceleration.x = imu_acc_x;
            imu.linear_acceleration.y = imu_acc_y;
            imu.linear_acceleration.z = imu_acc_z;

            // 设置角速度
            imu.angular_velocity.x = imu_gyro_x;
            imu.angular_velocity.y = imu_gyro_y;
            imu.angular_velocity.z = imu_gyro_z;
        }

        // 设置协方差矩阵(如果不确定,可以设置为未知)
        for(int i=0; i<9; i++) {
            imu.orientation_covariance[i] = -1;  // 姿态数据无法确定，设置为-1(方向协方差未知)
            imu.angular_velocity_covariance[i] = 0.01;  // 角速度协方差
            imu.linear_acceleration_covariance[i] = 0.01;  // 线性加速度协方差
        }

        imu_pub.publish(imu);

        // 输出当前里程计和IMU数据
        ROS_INFO_THROTTLE(1, "Current position: x=%f, y=%f, theta=%f", odm_x, odm_y, odm_th);
        ROS_INFO_THROTTLE(1, "IMU data - acceleration: x=%f, y=%f, z=%f, gyro: x=%f, y=%f, z=%f",
                        imu_acc_x, imu_acc_y, imu_acc_z, imu_gyro_x, imu_gyro_y, imu_gyro_z);

        waitForResponse();

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