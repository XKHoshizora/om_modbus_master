/** @file    amr_imu_node.cpp

@attention  对象设备 IMU
@details    处理内容: 读取IMU数据并发布到/imu话题
*/

#include "ros/ros.h"
#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_state.h"
#include "sensor_msgs/Imu.h"
#include <mutex>

// 全局变量
int gState_driver = 0;
int gState_mes = 0;
int gState_error = 0;

std::mutex imu_mutex;
double accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
double gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;

// 响应回调函数
void resCallback(const om_modbus_master::om_response& msg) {
    if (msg.slave_id == 1) {
        std::lock_guard<std::mutex> lock(imu_mutex);
        // 根据寄存器地址读取IMU数据
        accel_x = static_cast<int16_t>(msg.data[0]) / 16384.0 * 9.80665; // 转换为 m/s^2
        accel_y = static_cast<int16_t>(msg.data[1]) / 16384.0 * 9.80665;
        accel_z = static_cast<int16_t>(msg.data[2]) / 16384.0 * 9.80665;
        gyro_x = static_cast<int16_t>(msg.data[3]) / 131.0 * (M_PI / 180.0); // 转换为 rad/s
        gyro_y = static_cast<int16_t>(msg.data[4]) / 131.0 * (M_PI / 180.0);
        gyro_z = static_cast<int16_t>(msg.data[5]) / 131.0 * (M_PI / 180.0);
    }
}

// 状态回调函数
void stateCallback(const om_modbus_master::om_state& msg) {
    gState_driver = msg.state_driver;
    gState_mes = msg.state_mes;
    gState_error = msg.state_error;
}

// 等待函数
void wait() {
    ros::Duration(0.03).sleep();
    ros::spinOnce();
    while (gState_driver == 1) {
        ros::spinOnce();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "amr_imu_node");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<om_modbus_master::om_query>("om_query1", 1);
    ros::Subscriber sub1 = n.subscribe("om_response1", 1, resCallback);
    ros::Subscriber sub2 = n.subscribe("om_state1", 1, stateCallback);

    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 50);

    om_modbus_master::om_query msg;

    double update_rate;
    n.param("update_rate", update_rate, 100.0); // IMU数据的更新和发布频率，默认为100Hz

    ros::Rate loop_rate(update_rate);

    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();

        // 读取IMU数据
        msg.slave_id = 0x01;
        msg.func_code = 3; // 读取保持寄存器
        msg.read_addr = 0x3B; // IMU数据起始地址
        msg.read_num = 6; // 读取6个寄存器（加速度x,y,z和角速度x,y,z）

        pub.publish(msg);

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = current_time;
        imu_msg.header.frame_id = "imu_link";

        {
            std::lock_guard<std::mutex> lock(imu_mutex);
            imu_msg.linear_acceleration.x = accel_x;
            imu_msg.linear_acceleration.y = accel_y;
            imu_msg.linear_acceleration.z = accel_z;
            imu_msg.angular_velocity.x = gyro_x;
            imu_msg.angular_velocity.y = gyro_y;
            imu_msg.angular_velocity.z = gyro_z;
        }

        // 设置协方差（这里使用示例值，实际使用时应根据IMU的具体特性设置）
        for (int i = 0; i < 9; ++i) {
            imu_msg.linear_acceleration_covariance[i] = 0.0;
            imu_msg.angular_velocity_covariance[i] = 0.0;
            imu_msg.orientation_covariance[i] = 0.0;
        }
        imu_msg.linear_acceleration_covariance[0] = 
        imu_msg.linear_acceleration_covariance[4] = 
        imu_msg.linear_acceleration_covariance[8] = 0.01;
        imu_msg.angular_velocity_covariance[0] = 
        imu_msg.angular_velocity_covariance[4] = 
        imu_msg.angular_velocity_covariance[8] = 0.01;
        imu_msg.orientation_covariance[0] = -1; // 表示没有方向数据

        imu_pub.publish(imu_msg);

        wait();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}