/** @file    amr_ros_bridge.cpp
 *  @brief   ROS节点，用于处理机器人的里程计和IMU数据并发布TF变换
 *
 *  @details 该节点主要完成以下任务：
 *           1. 接收cmd_vel命令并将其写入到Modbus设备来控制AMR
 *           2. 从Modbus设备读取里程计和IMU数据
 *           3. 发布里程计数据（odom话题和TF变换）
 *           4. 发布IMU数据（imu话题）
 */
/** @file    amr_ros_bridge.cpp
 *  @brief   ROS节点，用于处理机器人的里程计和IMU数据并发布TF变换
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mutex>
#include <atomic>

#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_state.h"

// 全局常量定义
const double ACC_SCALE = 0.001;      // 加速度标定系数
const double GYRO_SCALE = 0.000001;  // 角速度标定系数
const double MAX_LINEAR_VEL = 2.0;   // 最大线速度(m/s)
const double MAX_ANGULAR_VEL = 6.2;  // 最大角速度(rad/s)

// Modbus寄存器地址
const uint16_t REG_MAP_START = 4864;
const uint16_t REG_ODOM_START = 4928;
const uint16_t REG_VEL_START = 4960;

// 全局发布器和订阅器
ros::Publisher query_pub;
ros::Publisher odom_pub;
ros::Publisher imu_pub;
tf2_ros::TransformBroadcaster* tf_broadcaster_ptr = nullptr;

// 坐标系参数
std::string odom_frame = "odom";
std::string base_frame = "base_footprint";
std::string imu_frame = "imu_link";

// 机器人状态结构体
struct RobotState {
    double odom_x = 0.0;
    double odom_y = 0.0;
    double odom_yaw = 0.0;
    double acc_x = 0.0;
    double acc_y = 0.0;
    double acc_z = 0.0;
    double gyro_x = 0.0;
    double gyro_y = 0.0;
    double gyro_z = 0.0;
} robot_state;

// 速度指令结构体
struct VelocityCommand {
    double linear_x = 0.0;
    double linear_y = 0.0;
    double angular_z = 0.0;
    bool updated = false;
} latest_cmd;

// 安全控制结构体
struct SafetyControl {
    ros::Time last_cmd_time;
    ros::Time last_odom_time;
    std::atomic<bool> odom_healthy{false};
    const double CMD_TIMEOUT = 0.2;
    const double ODOM_TIMEOUT = 0.2;
} safety_control;

// 互斥锁
std::mutex state_mutex;
std::mutex cmd_mutex;

// 全局消息
om_modbus_master::om_query query_msg;

// 处理从Modbus设备接收到的响应
void responseCallback(const om_modbus_master::om_response::ConstPtr& msg) {
    if (!msg || msg->data.size() < 9) {
        safety_control.odom_healthy = false;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex);
        robot_state.odom_x = msg->data[0] / 1000.0;      // mm -> m
        robot_state.odom_y = msg->data[1] / 1000.0;
        robot_state.odom_yaw = msg->data[2] / 1000000.0; // μrad -> rad

        // 打印里程计数据
        ROS_DEBUG_THROTTLE(1.0, "Odom: x=%.3f, y=%.3f, yaw=%.3f",
            robot_state.odom_x, robot_state.odom_y, robot_state.odom_yaw);

        robot_state.acc_x = msg->data[3] * ACC_SCALE;    // 转换为m/s²
        robot_state.acc_y = msg->data[4] * ACC_SCALE;
        robot_state.acc_z = msg->data[5] * ACC_SCALE;
        robot_state.gyro_x = msg->data[6] * GYRO_SCALE;  // 转换为rad/s
        robot_state.gyro_y = msg->data[7] * GYRO_SCALE;
        robot_state.gyro_z = msg->data[8] * GYRO_SCALE;
    }

    safety_control.last_odom_time = ros::Time::now();
    safety_control.odom_healthy = true;
}

// 处理接收到的速度指令
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex);
    latest_cmd.linear_x = std::clamp(msg->linear.x, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    latest_cmd.linear_y = std::clamp(msg->linear.y, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    latest_cmd.angular_z = std::clamp(msg->angular.z, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
    latest_cmd.updated = true;
    safety_control.last_cmd_time = ros::Time::now();

    // 打印速度命令
    ROS_DEBUG("Cmd_vel received: linear=(%.3f, %.3f), angular=%.3f",
        latest_cmd.linear_x, latest_cmd.linear_y, latest_cmd.angular_z);
}

// 发布里程计和TF数据
void publishOdomAndTf(const ros::Time& time) {
    std::lock_guard<std::mutex> lock(state_mutex);

    // 发布TF变换
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = time;
    tf_msg.header.frame_id = odom_frame;
    tf_msg.child_frame_id = base_frame;
    tf_msg.transform.translation.x = robot_state.odom_x;
    tf_msg.transform.translation.y = robot_state.odom_y;
    tf_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, robot_state.odom_yaw);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    if(tf_broadcaster_ptr) {
        tf_broadcaster_ptr->sendTransform(tf_msg);
    }

    // 创建并发布里程计消息
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = time;
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id = base_frame;

    // 设置位姿
    odom_msg.pose.pose.position.x = robot_state.odom_x;
    odom_msg.pose.pose.position.y = robot_state.odom_y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf_msg.transform.rotation;

    // 设置位姿协方差
    for(int i = 0; i < 36; ++i) {
        odom_msg.pose.covariance[i] = 0.0;
    }
    odom_msg.pose.covariance[0] = 0.01;   // x
    odom_msg.pose.covariance[7] = 0.01;   // y
    odom_msg.pose.covariance[35] = 0.01;  // yaw

    // 设置速度 - 这里应该使用实际的速度值，而不是命令值
    {
        std::lock_guard<std::mutex> cmd_lock(cmd_mutex);
        odom_msg.twist.twist.linear.x = query_msg.data[1] / 1000.0;    // mm/s -> m/s
        odom_msg.twist.twist.linear.y = query_msg.data[3] / 1000.0;    // mm/s -> m/s
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = query_msg.data[2] / 1000000.0; // μrad/s -> rad/s
    }

    // 设置速度协方差
    for(int i = 0; i < 36; ++i) {
        odom_msg.twist.covariance[i] = 0.0;
    }
    odom_msg.twist.covariance[0] = 0.01;   // vx
    odom_msg.twist.covariance[7] = 0.01;   // vy
    odom_msg.twist.covariance[35] = 0.01;  // vth

    odom_pub.publish(odom_msg);
}

// 发布IMU数据
void publishImu(const ros::Time& time) {
    std::lock_guard<std::mutex> lock(state_mutex);

    sensor_msgs::Imu imu_msg;

    // 设置header
    imu_msg.header.stamp = time;
    imu_msg.header.frame_id = imu_frame;

    // 设置方向数据（没有方向数据）
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 0.0;

    // 标记没有方向估计
    imu_msg.orientation_covariance[0] = -1;
    for(int i = 1; i < 9; ++i) {
        imu_msg.orientation_covariance[i] = 0.0;
    }

    // 设置角速度
    imu_msg.angular_velocity.x = robot_state.gyro_x;
    imu_msg.angular_velocity.y = robot_state.gyro_y;
    imu_msg.angular_velocity.z = robot_state.gyro_z;

    // 设置角速度协方差
    for(int i = 0; i < 9; ++i) {
        imu_msg.angular_velocity_covariance[i] = 0.0;
    }

    // 设置线性加速度
    imu_msg.linear_acceleration.x = robot_state.acc_x;
    imu_msg.linear_acceleration.y = robot_state.acc_y;
    imu_msg.linear_acceleration.z = robot_state.acc_z;

    // 设置加速度协方差
    for(int i = 0; i < 9; ++i) {
        imu_msg.linear_acceleration_covariance[i] = 0.0;
    }

    imu_pub.publish(imu_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "amr_ros_bridge");
    ros::NodeHandle nh;

    // 创建TF广播器
    tf2_ros::TransformBroadcaster tf_broadcaster;
    tf_broadcaster_ptr = &tf_broadcaster;

    // 创建发布器和订阅器
    query_pub = nh.advertise<om_modbus_master::om_query>("om_query1", 1);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 50);
    ros::Subscriber response_sub = nh.subscribe("om_response1", 1, responseCallback);
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1, cmdVelCallback);

    ROS_INFO("Initializing AMR ROS Bridge...");
    ros::Duration(1.0).sleep();

    // 设置寄存器映射
    om_modbus_master::om_query init_msg;
    init_msg.slave_id = 0x01;
    init_msg.func_code = 1;
    init_msg.write_addr = REG_MAP_START;
    init_msg.write_num = 32;

    for(int i = 0; i < 64; i++) {
        init_msg.data[i] = 0;
    }

    init_msg.data[0] = 1069;  // 里程计X
    init_msg.data[1] = 1070;  // 里程计Y
    init_msg.data[2] = 1071;  // 里程计Theta
    init_msg.data[3] = 1038;  // IMU加速度X
    init_msg.data[4] = 1039;  // IMU加速度Y
    init_msg.data[5] = 1040;  // IMU加速度Z
    init_msg.data[6] = 1041;  // IMU角速度X
    init_msg.data[7] = 1042;  // IMU角速度Y
    init_msg.data[8] = 1043;  // IMU角速度Z
    init_msg.data[16] = 993;  // 直接数据运行模式
    init_msg.data[17] = 994;  // Vx
    init_msg.data[18] = 995;  // ω
    init_msg.data[19] = 996;  // Vy

    query_pub.publish(init_msg);
    ros::Duration(1.0).sleep();
    ROS_INFO("AMR ROS Bridge initialized successfully");

    // 初始化安全控制时间戳
    safety_control.last_cmd_time = ros::Time::now();
    safety_control.last_odom_time = ros::Time::now();

    // 配置查询消息
    query_msg.slave_id = 0x01;
    query_msg.func_code = 2;
    query_msg.read_addr = REG_ODOM_START;
    query_msg.read_num = 9;
    query_msg.write_addr = REG_VEL_START;
    query_msg.write_num = 4;

    for(int i = 0; i < 64; i++) {
        query_msg.data[i] = 0;
    }
    query_msg.data[0] = 1;  // 使能

    // 主循环
    ros::Rate rate(20);  // 20Hz
    while(ros::ok()) {
        ros::Time current_time = ros::Time::now();
        bool need_stop = false;

        // 安全检查
        if ((current_time - safety_control.last_cmd_time).toSec() > safety_control.CMD_TIMEOUT) {
            need_stop = true;
            ROS_DEBUG_THROTTLE(1.0, "Velocity command timeout, stopping robot");
        }

        if ((current_time - safety_control.last_odom_time).toSec() > safety_control.ODOM_TIMEOUT) {
            need_stop = true;
            safety_control.odom_healthy = false;
            ROS_DEBUG_THROTTLE(1.0, "Odometry timeout, stopping robot");
        }

        // 更新速度命令
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            if (need_stop) {
                query_msg.data[1] = 0;
                query_msg.data[2] = 0;
                query_msg.data[3] = 0;
            } else if (latest_cmd.updated) {
                query_msg.data[1] = static_cast<int32_t>(latest_cmd.linear_x * 1000);    // m/s -> mm/s
                query_msg.data[2] = static_cast<int32_t>(latest_cmd.angular_z * 1000000); // rad/s -> μrad/s
                query_msg.data[3] = static_cast<int32_t>(latest_cmd.linear_y * 1000);    // m/s -> mm/s
                latest_cmd.updated = false;
            }
        }

        // 发送查询消息
        query_pub.publish(query_msg);

        // 发布数据
        publishOdomAndTf(current_time);
        publishImu(current_time);

        ros::spinOnce();
        rate.sleep();
    }

    // 程序退出前确保机器人停止
    query_msg.data[1] = 0;
    query_msg.data[2] = 0;
    query_msg.data[3] = 0;
    query_pub.publish(query_msg);

    ROS_INFO("AMR ROS Bridge shutdown completed");

    return 0;
}