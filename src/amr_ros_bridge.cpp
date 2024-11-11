/** @file    amr_ros_bridge.cpp
 *  @brief   ROS节点，用于处理机器人的里程计和IMU数据并发布TF变换
 *
 *  @details 该节点主要完成以下任务：
 *           1. 接收cmd_vel命令并将其写入到Modbus设备来控制AMR
 *           2. 从Modbus设备读取里程计和IMU数据
 *           3. 发布里程计数据（odom话题和TF变换）
 *           4. 发布IMU数据（imu话题）
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

// 全局变量定义
double ACC_SCALE = 0.001;      // 加速度标定系数: raw * 0.001 = m/s²
double GYRO_SCALE = 0.000001;  // 角速度标定系数: raw * 0.000001 = rad/s
double MAX_LINEAR_VEL = 2.0;   // 最大线速度限制(m/s)
double MAX_ANGULAR_VEL = 6.2;  // 最大角速度限制(rad/s)

// Modbus寄存器地址
const uint16_t REG_MAP_START = 4864;  // 寄存器映射起始地址
const uint16_t REG_ODOM_START = 4928; // 里程计数据起始地址
const uint16_t REG_VEL_START = 4960;  // 速度控制寄存器起始地址

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
    // 里程计位置 (全局坐标系)
    double odom_x = 0.0;        // X位置 (meters)
    double odom_y = 0.0;        // Y位置 (meters)
    double odom_yaw = 0.0;      // 偏航角 (radians)

    // IMU数据
    double acc_x = 0.0;         // X轴加速度 (m/s²)
    double acc_y = 0.0;         // Y轴加速度 (m/s²)
    double acc_z = 0.0;         // Z轴加速度 (m/s²)
    double gyro_x = 0.0;        // Roll角速度 (rad/s)
    double gyro_y = 0.0;        // Pitch角速度 (rad/s)
    double gyro_z = 0.0;        // Yaw角速度 (rad/s)

    // 实际速度 (底盘坐标系)
    double vel_x_actual = 0.0;      // 实际X方向速度 (m/s)
    double vel_y_actual = 0.0;      // 实际Y方向速度 (m/s)
    double vel_angular_actual = 0.0; // 实际角速度 (rad/s)

    // 指令速度 (底盘坐标系)
    double vel_x_cmd = 0.0;         // 指令X方向速度 (m/s)
    double vel_y_cmd = 0.0;         // 指令Y方向速度 (m/s)
    double vel_angular_cmd = 0.0;    // 指令角速度 (rad/s)
} robot_state;

// 速度指令结构体
struct VelocityCommand {
    double linear_x = 0.0;      // X方向线速度 (m/s)
    double linear_y = 0.0;      // Y方向线速度 (m/s)
    double angular_z = 0.0;     // Z轴角速度 (rad/s)
    bool updated = false;       // 更新标志
} latest_cmd;

// 安全控制结构体
struct SafetyControl {
    ros::Time last_cmd_time;    // 最后一次接收到速度命令的时间
    ros::Time last_odom_time;   // 最后一次接收到里程计数据的时间
    std::atomic<bool> odom_healthy{false};
    double cmd_timeout = 0.2;    // 速度命令超时时间(seconds)
    double odom_timeout = 0.2;   // 里程计数据超时时间(seconds)
} safety_control;

// 互斥锁
std::mutex state_mutex;  // 用于保护robot_state的互斥锁
std::mutex cmd_mutex;    // 用于保护latest_cmd的互斥锁

// 全局消息
om_modbus_master::om_query query_msg;

// 处理从Modbus设备接收到的响应
void responseCallback(const om_modbus_master::om_response::ConstPtr& msg) {
    if (!msg || msg->data.size() < 15) {  // 确保数据完整性
        safety_control.odom_healthy = false;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex);

        // 处理里程计位置数据
        robot_state.odom_x = msg->data[0] / 1000.0;      // mm -> m
        robot_state.odom_y = msg->data[1] / 1000.0;      // mm -> m
        robot_state.odom_yaw = msg->data[2] / 1000000.0; // μrad -> rad

        // 处理IMU数据
        robot_state.acc_x = msg->data[3] * ACC_SCALE;    // raw -> m/s²  (1=0.001[m/s²])
        robot_state.acc_y = msg->data[4] * ACC_SCALE;    // raw -> m/s²
        robot_state.acc_z = msg->data[5] * ACC_SCALE;    // raw -> m/s²
        robot_state.gyro_x = msg->data[6] * GYRO_SCALE;  // raw -> rad/s (1=0.000001[rad/s])
        robot_state.gyro_y = msg->data[7] * GYRO_SCALE;  // raw -> rad/s
        robot_state.gyro_z = msg->data[8] * GYRO_SCALE;  // raw -> rad/s

        // 处理实际速度数据
        robot_state.vel_x_actual = msg->data[9] / 1000.0;       // mm/s -> m/s
        robot_state.vel_angular_actual = msg->data[10] / 1000000.0; // μrad/s -> rad/s
        robot_state.vel_y_actual = msg->data[11] / 1000.0;      // mm/s -> m/s

        // 处理指令速度数据
        robot_state.vel_x_cmd = msg->data[12] / 1000.0;       // mm/s -> m/s
        robot_state.vel_angular_cmd = msg->data[13] / 1000000.0; // μrad/s -> rad/s
        robot_state.vel_y_cmd = msg->data[14] / 1000.0;      // mm/s -> m/s

        // 打印调试信息
        ROS_DEBUG_THROTTLE(1.0, "Odom: x=%.3f, y=%.3f, yaw=%.3f",
            robot_state.odom_x, robot_state.odom_y, robot_state.odom_yaw);
        ROS_DEBUG_THROTTLE(1.0, "Velocity(actual): vx=%.3f, vy=%.3f, w=%.3f",
            robot_state.vel_x_actual, robot_state.vel_y_actual, robot_state.vel_angular_actual);
    }

    safety_control.last_odom_time = ros::Time::now();
    safety_control.odom_healthy = true;
}

// 处理接收到的速度指令
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex);
    // 速度限制和更新
    latest_cmd.linear_x = std::clamp(msg->linear.x, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    latest_cmd.linear_y = std::clamp(msg->linear.y, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    latest_cmd.angular_z = std::clamp(msg->angular.z, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
    latest_cmd.updated = true;
    safety_control.last_cmd_time = ros::Time::now();

    ROS_DEBUG("Cmd_vel received: linear=(%.3f, %.3f), angular=%.3f",
        latest_cmd.linear_x, latest_cmd.linear_y, latest_cmd.angular_z);
}

// 发布里程计和TF数据
void publishOdomAndTf(const ros::Time& time) {
    std::lock_guard<std::mutex> lock(state_mutex);

    // 创建并发布TF变换
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = time;
    tf_msg.header.frame_id = odom_frame;
    tf_msg.child_frame_id = base_frame;

    // 设置平移部分
    tf_msg.transform.translation.x = robot_state.odom_x;
    tf_msg.transform.translation.y = robot_state.odom_y;
    tf_msg.transform.translation.z = 0.0;

    // 设置旋转部分(将偏航角转换为四元数)
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
    odom_msg.pose.covariance[0] = 0.01;   // x方向位置协方差
    odom_msg.pose.covariance[7] = 0.01;   // y方向位置协方差
    odom_msg.pose.covariance[35] = 0.01;  // yaw角度协方差

    // 设置速度 - 使用实际速度值，而非指令速度
    odom_msg.twist.twist.linear.x = robot_state.vel_x_actual;
    odom_msg.twist.twist.linear.y = robot_state.vel_y_actual;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = robot_state.vel_angular_actual;

    // 设置速度协方差
    for(int i = 0; i < 36; ++i) {
        odom_msg.twist.covariance[i] = 0.0;
    }
    odom_msg.twist.covariance[0] = 0.01;   // vx协方差
    odom_msg.twist.covariance[7] = 0.01;   // vy协方差
    odom_msg.twist.covariance[35] = 0.01;  // 角速度协方差

    odom_pub.publish(odom_msg);
}

// 发布IMU数据
void publishImu(const ros::Time& time) {
    std::lock_guard<std::mutex> lock(state_mutex);

    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = time;
    imu_msg.header.frame_id = imu_frame;

    // IMU方向数据（当前未提供）
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 0.0;
    imu_msg.orientation_covariance[0] = -1;  // 表示没有方向数据

    // 设置角速度数据
    imu_msg.angular_velocity.x = robot_state.gyro_x;    // rad/s
    imu_msg.angular_velocity.y = robot_state.gyro_y;    // rad/s
    imu_msg.angular_velocity.z = robot_state.gyro_z;    // rad/s

    // 设置线性加速度数据
    imu_msg.linear_acceleration.x = robot_state.acc_x;  // m/s²
    imu_msg.linear_acceleration.y = robot_state.acc_y;  // m/s²
    imu_msg.linear_acceleration.z = robot_state.acc_z;  // m/s²

    // 设置协方差矩阵
    for(int i = 1; i < 9; ++i) {
        imu_msg.orientation_covariance[i] = 0.0;
    }
    for(int i = 0; i < 9; ++i) {
        imu_msg.angular_velocity_covariance[i] = 0.0;
        imu_msg.linear_acceleration_covariance[i] = 0.0;
    }

    imu_pub.publish(imu_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "amr_ros_bridge");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");  // 添加私有节点句柄

    // 从参数服务器读取参数
    private_nh.param<double>("max_linear_vel", MAX_LINEAR_VEL);
    private_nh.param<double>("max_angular_vel", MAX_ANGULAR_VEL);

    double cmd_timeout = 0.2;
    double odom_timeout = 0.2;
    private_nh.getParam("cmd_timeout", cmd_timeout);
    private_nh.getParam("odom_timeout", odom_timeout);
    safety_control.cmd_timeout = cmd_timeout;
    safety_control.odom_timeout = odom_timeout;

    // 读取坐标系参数
    private_nh.param<std::string>("base_frame", base_frame, "base_footprint");
    private_nh.param<std::string>("odom_frame", odom_frame, "odom");
    private_nh.param<std::string>("imu_frame", imu_frame, "imu_link");

    // 读取协方差参数
    double pose_covariance = 0.01;
    double twist_covariance = 0.01;
    private_nh.getParam("pose_covariance", pose_covariance);
    private_nh.getParam("twist_covariance", twist_covariance);

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

    // 里程计相关寄存器映射
    init_msg.data[0] = 1069;  // 里程计X（1=0.001[m]），显示通过编码器反馈计算得到的里程计X方向的结果
    init_msg.data[1] = 1070;  // 里程计Y（1=0.001[m]），显示通过编码器反馈计算得到的里程计Y方向的结果
    init_msg.data[2] = 1071;  // 里程计Theta（1=0.000001[rad]），显示通过编码器反馈计算得到的里程计角度（θ）结果

    // IMU传感器数据寄存器映射
    init_msg.data[3] = 1038;  // IMU加速度X（1=0.001[m/s²]），显示IMU传感器的加速度X值
    init_msg.data[4] = 1039;  // IMU加速度Y（1=0.001[m/s²]），显示IMU传感器的加速度Y值
    init_msg.data[5] = 1040;  // IMU加速度Z（1=0.001[m/s²]），显示IMU传感器的加速度Z值
    init_msg.data[6] = 1041;  // IMU角速度Roll（1=0.000001[rad/s]），显示IMU传感器的角速度Roll值
    init_msg.data[7] = 1042;  // IMU角速度Pitch（1=0.000001[rad/s]），显示IMU传感器的角速度Pitch值
    init_msg.data[8] = 1043;  // IMU角速度Yaw（1=0.000001[rad/s]），显示IMU传感器的角速度Yaw值

    // 实际速度数据寄存器映射
    init_msg.data[9] = 1246;  // 当前平移速度（检测）Vx（1=0.001[m/s]），显示通过编码器反馈计算的当前机器人平移速度
    init_msg.data[10] = 1247; // 当前角速度（检测）ω（1=0.000001[rad/s]），显示通过编码器反馈计算的当前机器人角速度
    init_msg.data[11] = 1248; // 当前平移速度（检测）Vy（1=0.001[m/s]），显示通过编码器反馈计算的当前机器人平移速度

    // 指令速度数据寄存器映射
    init_msg.data[12] = 1251; // 当前平移速度（指令）Vx（1=0.001[m/s]），显示驱动器内部当前机器人平移速度的指令值
    init_msg.data[13] = 1252; // 当前角速度（指令）ω（1=0.000001[rad/s]），显示驱动器内部当前机器人角速度的指令值
    init_msg.data[14] = 1253; // 当前平移速度（指令）Vy（1=0.001[m/s]），显示驱动器内部当前机器人平移速度的指令值

    // 驱动控制寄存器映射
    init_msg.data[16] = 993;  // 驾驶模式，设置直接数据驱动的模式（0：无效，1：Vω控制）
    init_msg.data[17] = 994;  // 前后平移速度Vx（1=0.001[m/s]），设置范围：-2000～2000
    init_msg.data[18] = 995;  // 角速度ω（1=0.000001[rad/s]），设置范围：-6283186～6283186
    init_msg.data[19] = 996;  // 左右平移速度Vy（1=0.001[m/s]），设置范围：-2000～2000

    // 发布初始化消息并等待
    query_pub.publish(init_msg);
    ros::Duration(1.0).sleep();
    ROS_INFO("AMR ROS Bridge initialized successfully");

    // 初始化安全控制时间戳
    safety_control.last_cmd_time = ros::Time::now();
    safety_control.last_odom_time = ros::Time::now();

    // 配置周期查询消息
    query_msg.slave_id = 0x01;
    query_msg.func_code = 2;
    query_msg.read_addr = REG_ODOM_START;
    query_msg.read_num = 15;  // 读取15个寄存器数据
    query_msg.write_addr = REG_VEL_START;
    query_msg.write_num = 4;  // 写入4个数据（使能+三个速度分量）

    // 初始化查询消息数据
    for(int i = 0; i < 64; i++) {
        query_msg.data[i] = 0;
    }
    query_msg.data[0] = 1;  // 使能控制

    // 主循环
    ros::Rate rate(20);  // 20Hz的更新频率
    while(ros::ok()) {
        ros::Time current_time = ros::Time::now();
        bool need_stop = false;

        // 安全检查：超时保护
        if ((current_time - safety_control.last_cmd_time).toSec() > safety_control.cmd_timeout) {
            need_stop = true;
            ROS_DEBUG_THROTTLE(1.0, "Velocity command timeout, stopping robot");
        }

        if ((current_time - safety_control.last_odom_time).toSec() > safety_control.odom_timeout) {
            need_stop = true;
            safety_control.odom_healthy = false;
            ROS_DEBUG_THROTTLE(1.0, "Odometry timeout, stopping robot");
        }

        // 更新速度命令
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            if (need_stop) {
                // 超时情况下停止机器人
                query_msg.data[1] = 0;  // Vx = 0
                query_msg.data[2] = 0;  // ω = 0
                query_msg.data[3] = 0;  // Vy = 0
            } else if (latest_cmd.updated) {
                // 将速度命令转换为控制器单位并写入
                query_msg.data[1] = static_cast<int32_t>(latest_cmd.linear_x * 1000);     // m/s -> mm/s
                query_msg.data[2] = static_cast<int32_t>(latest_cmd.angular_z * 1000000); // rad/s -> μrad/s
                query_msg.data[3] = static_cast<int32_t>(latest_cmd.linear_y * 1000);     // m/s -> mm/s
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

    // 程序退出前确保机器人安全停止
    query_msg.data[1] = 0;
    query_msg.data[2] = 0;
    query_msg.data[3] = 0;
    query_pub.publish(query_msg);

    ROS_INFO("AMR ROS Bridge shutdown completed");

    return 0;
}