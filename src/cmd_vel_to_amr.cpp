/**
 * @file    cmd_vel_to_amr.cpp
 * @brief   ROS节点,用于将cmd_vel命令转换并发送到AMR(自主移动机器人)
 *
 * @details 该节点主要完成以下任务:
 *          1. 订阅cmd_vel话题,接收速度命令
 *          2. 将速度命令转换为适合AMR的格式
 *          3. 通过Modbus协议将转换后的命令发送到AMR
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_state.h"

#include <mutex>

/* 全局变量 */
int gState_driver = 0; // 通信状态标志(0:可通信,1:通信中)
int gState_mes = 0;    // 消息状态(0:无消息,1:消息到达,2:消息错误)
int gState_error = 0;  // 错误状态(0:无错误,1:无响应,2:异常响应)

std::mutex cmd_vel_mutex;
double x_spd = 0.0; // X方向速度 [mm/s]
double y_spd = 0.0; // Y方向速度 [mm/s]
double z_ang = 0.0; // 角速度 [rad/s]

/**
 * @brief 处理接收到的速度命令
 * @param twist 接收到的Twist消息
 */
void cmdVelCallback(const geometry_msgs::Twist& twist) {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex);
    x_spd = int(twist.linear.x * 1000.0);  // 转换为mm/s
    y_spd = int(twist.linear.y * 1000.0);  // 转换为mm/s
    z_ang = int(twist.angular.z * 1000000.0);  // 转换为μrad/s
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
 * @brief 等待通信完成
 */
void wait(void) {
    ros::Duration(0.03).sleep();
    ros::spinOnce();
    while (gState_driver == 1) {
        ros::spinOnce();
    }
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "cmd_vel_to_amr");
    ros::NodeHandle nh;

    // 从参数服务器获取更新频率参数
    double update_rate;
    nh.param("update_rate", update_rate, 10.0);  // 默认更新率为10Hz

    // 创建发布者和订阅者
    ros::Publisher modbus_pub = nh.advertise<om_modbus_master::om_query>("om_query1", 1);
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1, cmdVelCallback);
    ros::Subscriber state_sub = nh.subscribe("om_state1", 1, stateCallback);

    om_modbus_master::om_query msg;

    ros::Rate loop_rate(update_rate);

    ROS_INFO("cmd_vel to AMR Node Start");

    while (ros::ok()) {
        // 将速度命令写入Modbus设备
        msg.slave_id = 0x01;
        msg.func_code = 2;
        msg.read_addr = 4928;
        msg.read_num = 3;
        msg.write_addr = 4960;
        msg.write_num = 4;
        {
            std::lock_guard<std::mutex> lock(cmd_vel_mutex);
            msg.data[0] = 1;
            msg.data[1] = x_spd;
            msg.data[2] = z_ang;
            msg.data[3] = y_spd;
        }
        modbus_pub.publish(msg);

        wait();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}