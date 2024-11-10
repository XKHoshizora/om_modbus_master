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
#include <thread>
#include <atomic>
#include <mutex>

#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_state.h"

// 全局变量
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
    ros::Time last_cmd_time;            // 最后收到速度指令的时间
    ros::Time last_odom_time;           // 最后收到里程计数据的时间
    std::atomic<bool> odom_healthy{false};  // 里程计健康状态
    const double CMD_TIMEOUT = 0.2;     // 速度指令超时时间(200ms)
    const double ODOM_TIMEOUT = 0.2;    // 里程计超时时间(200ms)
} safety_control;

// 互斥锁和同步变量
std::mutex state_mutex;
std::mutex cmd_mutex;
std::mutex query_mutex;
std::atomic<bool> is_running{true};

// 常量定义
const double ACC_SCALE = 0.001;
const double GYRO_SCALE = 0.000001;
const double MAX_LINEAR_VEL = 2.0;
const double MAX_ANGULAR_VEL = 6.2;

// Modbus寄存器地址
const uint16_t REG_MAP_START = 4864;
const uint16_t REG_ODOM_START = 4928;
const uint16_t REG_VEL_START = 4960;

// 预配置的消息
om_modbus_master::om_query state_query_msg;
om_modbus_master::om_query vel_msg;

// 处理从Modbus设备接收到的响应
void responseCallback(const om_modbus_master::om_response::ConstPtr& msg) {
    if (!msg || msg->data.size() < 9) {
        ROS_WARN_THROTTLE(1.0, "Invalid response data");
        safety_control.odom_healthy = false;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex);
        robot_state.odom_x = msg->data[0] / 1000.0;      // mm -> m
        robot_state.odom_y = msg->data[1] / 1000.0;
        robot_state.odom_yaw = msg->data[2] / 1000000.0; // μrad -> rad
        robot_state.acc_x = msg->data[3] * ACC_SCALE;
        robot_state.acc_y = msg->data[4] * ACC_SCALE;
        robot_state.acc_z = msg->data[5] * ACC_SCALE;
        robot_state.gyro_x = msg->data[6] * GYRO_SCALE;
        robot_state.gyro_y = msg->data[7] * GYRO_SCALE;
        robot_state.gyro_z = msg->data[8] * GYRO_SCALE;
    }

    // 更新里程计健康状态
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
}

// 发布里程计和TF数据
void publishOdomAndTf(const ros::Time& time) {
    std::lock_guard<std::mutex> lock(state_mutex);

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

    nav_msgs::Odometry odom_msg;
    odom_msg.header = tf_msg.header;
    odom_msg.child_frame_id = tf_msg.child_frame_id;
    odom_msg.pose.pose.position.x = robot_state.odom_x;
    odom_msg.pose.pose.position.y = robot_state.odom_y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf_msg.transform.rotation;

    odom_pub.publish(odom_msg);
}

// 发布IMU数据
void publishImu(const ros::Time& time) {
    std::lock_guard<std::mutex> lock(state_mutex);

    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = time;
    imu_msg.header.frame_id = imu_frame;
    imu_msg.linear_acceleration.x = robot_state.acc_x;
    imu_msg.linear_acceleration.y = robot_state.acc_y;
    imu_msg.linear_acceleration.z = robot_state.acc_z;
    imu_msg.angular_velocity.x = robot_state.gyro_x;
    imu_msg.angular_velocity.y = robot_state.gyro_y;
    imu_msg.angular_velocity.z = robot_state.gyro_z;

    imu_pub.publish(imu_msg);
}

// 状态查询线程（主线程）
void stateQueryThread() {
    ros::Rate rate(20);

    // 预先配置状态查询消息 - 只读取，不写入
    state_query_msg.slave_id = 0x01;
    state_query_msg.func_code = 2;  // 读取功能码
    state_query_msg.read_addr = REG_ODOM_START;
    state_query_msg.read_num = 9;
    state_query_msg.write_addr = 0;  // 不需要写入
    state_query_msg.write_num = 0;   // 不需要写入

    for(int i = 0; i < 64; i++) {
        state_query_msg.data[i] = 0;
    }

    while(ros::ok() && is_running) {
        {
            std::lock_guard<std::mutex> lock(query_mutex);
            query_pub.publish(state_query_msg);
        }

        ros::Time current_time = ros::Time::now();
        publishOdomAndTf(current_time);
        publishImu(current_time);

        ros::spinOnce();
        rate.sleep();
    }
}

// 速度控制线程
void velocityControlThread() {
    ros::Rate rate(20);  // 速度控制使用20Hz

    // 预先配置速度控制消息
    vel_msg.slave_id = 0x01;
    vel_msg.func_code = 1;  // 写入功能码
    vel_msg.write_addr = REG_VEL_START;
    vel_msg.write_num = 4;

    for(int i = 0; i < 64; i++) {
        vel_msg.data[i] = 0;
    }
    vel_msg.data[0] = 1;  // 使能

    // 确保初始状态为停止
    vel_msg.data[1] = 0;  // Vx = 0
    vel_msg.data[2] = 0;  // ω = 0
    vel_msg.data[3] = 0;  // Vy = 0

    while(ros::ok() && is_running) {
        ros::Time current_time = ros::Time::now();
        bool need_stop = false;

        // 安全检查
        if ((current_time - safety_control.last_cmd_time).toSec() > safety_control.CMD_TIMEOUT) {
            need_stop = true;
            ROS_WARN_THROTTLE(1.0, "Velocity command timeout, stopping robot");
        }

        if ((current_time - safety_control.last_odom_time).toSec() > safety_control.ODOM_TIMEOUT) {
            need_stop = true;
            safety_control.odom_healthy = false;
            ROS_ERROR_THROTTLE(1.0, "Odometry timeout, stopping robot");
        }

        if (!safety_control.odom_healthy) {
            need_stop = true;
            ROS_ERROR_THROTTLE(1.0, "Odometry unhealthy, stopping robot");
        }

        // 更新速度命令
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            if (need_stop) {
                // 安全停止
                vel_msg.data[1] = 0;
                vel_msg.data[2] = 0;
                vel_msg.data[3] = 0;
            } else if (latest_cmd.updated) {
                // 只在安全的情况下更新新的速度命令
                vel_msg.data[1] = static_cast<int32_t>(latest_cmd.linear_x * 1000);
                vel_msg.data[2] = static_cast<int32_t>(latest_cmd.angular_z * 1000000);
                vel_msg.data[3] = static_cast<int32_t>(latest_cmd.linear_y * 1000);
                latest_cmd.updated = false;
            }
        }

        // 发送速度命令
        {
            std::lock_guard<std::mutex> lock(query_mutex);
            query_pub.publish(vel_msg);
        }

        rate.sleep();
    }
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
    ros::Duration(0.5).sleep();

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

    {
        std::lock_guard<std::mutex> lock(query_mutex);
        query_pub.publish(init_msg);
    }

    ros::Duration(0.5).sleep();
    ROS_INFO("AMR ROS Bridge initialized successfully");

    // 初始化安全控制时间戳
    safety_control.last_cmd_time = ros::Time::now();
    safety_control.last_odom_time = ros::Time::now();

    // 启动速度控制线程
    std::thread vel_thread(velocityControlThread);

    // 在主线程中运行状态查询
    stateQueryThread();

    // 清理程序退出
    is_running = false;
    if(vel_thread.joinable()) {
        vel_thread.join();
    }

    // 确保机器人停止
    om_modbus_master::om_query stop_msg;
    stop_msg.slave_id = 0x01;
    stop_msg.func_code = 1;
    stop_msg.write_addr = REG_VEL_START;
    stop_msg.write_num = 4;

    for(int i = 0; i < 64; i++) {
        stop_msg.data[i] = 0;
    }
    stop_msg.data[0] = 1;  // 使能

    {
        std::lock_guard<std::mutex> lock(query_mutex);
        query_pub.publish(stop_msg);
    }

    ROS_INFO("AMR ROS Bridge shutdown complete");

    return 0;
}