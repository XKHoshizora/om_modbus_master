#ifndef AMR_ROS_BRIDGE_HPP
#define AMR_ROS_BRIDGE_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mutex>

#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_state.h"

class AmrRosBridge {
public:
    explicit AmrRosBridge(ros::NodeHandle& nh);
    ~AmrRosBridge();

    bool init();
    void run();
    void shutdown();

private:
    // 基本配置
    ros::NodeHandle& nh_;
    std::string odom_frame_{"odom"};
    std::string base_frame_{"base_footprint"};
    std::string imu_frame_{"imu_link"};
    double update_rate_{20.0};

    // ROS通信
    ros::Publisher query_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher imu_pub_;
    ros::Subscriber response_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber cmd_vel_sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // 状态变量
    std::mutex data_mutex_;
    bool running_{false};
    bool busy_{false};
    int error_{0};

    // 里程计数据
    struct {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
        double vx{0.0};
        double vy{0.0};
        double vth{0.0};
    } odom_data_;

    // IMU数据
    struct {
        double acc_x{0.0};
        double acc_y{0.0};
        double acc_z{0.0};
        double gyro_x{0.0};
        double gyro_y{0.0};
        double gyro_z{0.0};
    } imu_data_;

    // 回调函数
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void responseCallback(const om_modbus_master::om_response::ConstPtr& msg);
    void stateCallback(const om_modbus_master::om_state::ConstPtr& msg);

    // 内部函数
    bool loadParameters();
    void publishOdomAndTf(const ros::Time& time);
    void publishImu(const ros::Time& time);
    bool sendModbusCommand(uint8_t slave_id, uint8_t func_code,
                          uint16_t read_addr, uint16_t read_num,
                          uint16_t write_addr, uint16_t write_num,
                          const std::vector<int32_t>& data = std::vector<int32_t>());

    static constexpr double ACC_SCALE = 0.001;      // 加速度比例
    static constexpr double GYRO_SCALE = 0.000001;  // 角速度比例
    static constexpr double MAX_LINEAR_VEL = 1.0;   // 最大线速度(m/s)
    static constexpr double MAX_ANGULAR_VEL = 1.57; // 最大角速度(rad/s)
    static constexpr double TIMEOUT = 0.1;          // 超时时间(s)
};

#endif // AMR_ROS_BRIDGE_HPP