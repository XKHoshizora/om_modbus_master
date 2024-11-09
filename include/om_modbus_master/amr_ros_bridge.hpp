/** @file    amr_ros_bridge.hpp
 *  @brief   AMR ROS Bridge 类定义
 */

#ifndef AMR_ROS_BRIDGE_HPP
#define AMR_ROS_BRIDGE_HPP

#include "ros/ros.h"
#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_state.h"

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mutex>
#include <atomic>

class AmrRosBridge {
public:
    explicit AmrRosBridge(ros::NodeHandle& nh);
    ~AmrRosBridge();

    bool init();
    void run();
    void shutdown();

private:
    // 内部类：Modbus通信处理
    class ModbusComm {
    public:
        ModbusComm(ros::NodeHandle& nh);
        bool init();
        bool sendAndReceive(om_modbus_master::om_query& msg);
        bool waitForResponse();
        void setState(const om_modbus_master::om_state& state);

    private:
        ros::Publisher query_pub_;
        std::atomic<int> state_driver_{0};
        std::atomic<int> state_mes_{0};
        std::atomic<int> state_error_{0};

        static constexpr double SPIN_SLEEP_TIME = 0.001;
        static constexpr double COMM_TIMEOUT = 0.5;
        static constexpr int MAX_CONSECUTIVE_RETRIES = 3;
        ros::Time last_spin_time_;
    };

    // 内部类：里程计数据处理
    class OdometryHandler {
    public:
        OdometryHandler(ros::NodeHandle& nh);
        void updateCommand(const geometry_msgs::Twist& twist);
        void updateData(const om_modbus_master::om_response& msg);
        void publish(const ros::Time& current_time);
        int32_t getVelX() const;
        int32_t getVelY() const;
        int32_t getVelTh() const;

    private:
        ros::Publisher odom_pub_;
        tf2_ros::TransformBroadcaster tf_broadcaster_;

        std::mutex mutex_;
        // 速度命令 (实际值，单位：m/s 和 rad/s)
        double cmd_vel_x_{0.0};
        double cmd_vel_y_{0.0};
        double cmd_vel_th_{0.0};
        // 位置数据 (单位：m 和 rad)
        double odom_pos_x_{0.0};
        double odom_pos_y_{0.0};
        double odom_pos_th_{0.0};
    };

    // 内部类：IMU数据处理
    class ImuHandler {
    public:
        ImuHandler(ros::NodeHandle& nh);
        void updateData(const om_modbus_master::om_response& msg);
        void publish(const ros::Time& current_time);

    private:
        ros::Publisher imu_pub_;
        std::mutex mutex_;

        double acc_x_{0.0};
        double acc_y_{0.0};
        double acc_z_{0.0};
        double gyro_x_{0.0};
        double gyro_y_{0.0};
        double gyro_z_{0.0};

        static constexpr double ACC_SCALE = 0.001;   // 加速度转换因子
        static constexpr double GYRO_SCALE = 0.000001; // 角速度转换因子
    };

    // 主类成员
    ros::NodeHandle& nh_;
    double update_rate_;
    bool running_;

    std::unique_ptr<ModbusComm> modbus_;
    std::unique_ptr<OdometryHandler> odom_;
    std::unique_ptr<ImuHandler> imu_;

    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber modbus_response_sub_;
    ros::Subscriber modbus_state_sub_;

    // 回调函数
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void modbusResponseCallback(const om_modbus_master::om_response::ConstPtr& msg);
    void modbusStateCallback(const om_modbus_master::om_state::ConstPtr& msg);

    // 工具函数
    bool setupModbusRegisters(om_modbus_master::om_query& msg);
    void loadParameters();
};

#endif // AMR_ROS_BRIDGE_HPP