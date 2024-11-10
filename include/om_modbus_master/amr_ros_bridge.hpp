/**
 * @file amr_ros_bridge.hpp
 * @brief AMR ROS桥接头文件，包含AMR ROS Bridge节点的接口和实现
 */

#ifndef AMR_ROS_BRIDGE_HPP
#define AMR_ROS_BRIDGE_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mutex>
#include <atomic>
#include <queue>

#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_state.h"

#define DEBUG_MODBUS 1 // 调试标志

// 配置参数结构体
struct BridgeConfig {
    double update_rate{50.0};    // 主循环频率(Hz)
    double odom_rate{20.0};      // 里程计更新频率(Hz)
    double imu_rate{100.0};      // IMU更新频率(Hz)
    double cmd_timeout{0.1};     // 命令超时时间(s)

    double max_linear_vel{2.0};  // 最大线速度(m/s)
    double max_angular_vel{3.14}; // 最大角速度(rad/s)
    double min_valid_distance{0.001}; // 最小有效位移(m)
    double min_valid_rotation{0.001}; // 最小有效旋转(rad)

    double odom_alpha{0.8};      // 里程计数据平滑系数
    double imu_alpha{0.8};       // IMU数据平滑系数
};

// 主类AmrRosBridge，负责管理Modbus通信和ROS数据接口
class AmrRosBridge {
public:
    explicit AmrRosBridge(ros::NodeHandle& nh); // 构造函数
    ~AmrRosBridge();  // 析构函数

    bool init();  // 初始化
    void run();   // 主循环
    void shutdown();  // 关闭清理

    // Modbus通信处理类
    class ModbusHandler {
    public:
        enum class CmdType {
            READ,    // 读取命令
            WRITE,   // 写入命令
            MONITOR  // 状态监控
        };

        struct Command {  // 命令结构体
            uint8_t slave_id{0x01};      // 从站ID
            uint8_t func_code{0};        // 功能码
            uint32_t addr{0};            // 地址(用于单独读写)
            uint32_t read_addr{0};       // 读取地址
            uint32_t write_addr{0};      // 写入地址
            uint8_t data_num{0};         // 数据数量(用于单独读写)
            uint8_t read_num{0};         // 读取数量
            uint8_t write_num{0};        // 写入数量
            int32_t data[64]{};          // 数据
            ros::Time timestamp;         // 时间戳
            CmdType type{CmdType::READ}; // 命令类型
        };

        explicit ModbusHandler(ros::NodeHandle& nh,
                          OdometryHandler* odom,
                          ImuHandler* imu);

        bool init(); // 初始化
        bool sendCommand(const Command& cmd); // 发送命令

    private:
        OdometryHandler* odom_;  // 里程计处理器
        ImuHandler* imu_;        // IMU处理器

        void handleResponse(const om_modbus_master::om_response::ConstPtr& msg); // 处理Modbus响应
        void handleState(const om_modbus_master::om_state::ConstPtr& msg);       // 处理Modbus状态

        ros::Publisher query_pub_;      // 查询发布器
        ros::Subscriber response_sub_;  // 响应订阅器
        ros::Subscriber state_sub_;     // 状态订阅器

        volatile bool busy_{false};     // 通信忙状态
        volatile int error_{0};         // 错误状态
        std::queue<Command> cmd_queue_; // 命令队列
        mutable std::mutex queue_mutex_; // 队列互斥锁

        static constexpr int MAX_RETRY = 3;  // 最大重试次数
        static constexpr double TIMEOUT = 0.1; // 超时时间(s)

        ros::Time last_busy_time_;  // 记录最后一次忙状态的开始时间
        static constexpr double MIN_CMD_INTERVAL = 0.05;  // 最小命令间隔(20Hz)
    };

    // 里程计处理类
    class OdometryHandler {
    public:
        OdometryHandler(ros::NodeHandle& nh,
                       const std::string& odom_frame,
                       const std::string& base_frame,
                       const BridgeConfig& config);

        void updateOdometry(const std::vector<int32_t>& data); // 更新里程计数据
        void updateCommand(const geometry_msgs::Twist::ConstPtr& cmd); // 更新速度指令
        ModbusHandler::Command getReadCommand(); // 获取读取命令
        ModbusHandler::Command getWriteCommand(); // 获取写入命令
        void publish(const ros::Time& time); // 发布数据

    private:
        ros::Publisher odom_pub_; // 里程计发布器
        tf2_ros::TransformBroadcaster tf_broadcaster_; // TF广播器

        const std::string odom_frame_id_; // 里程计坐标系
        const std::string base_frame_id_; // 基座坐标系
        const BridgeConfig& config_;      // 配置参数引用

        mutable std::mutex data_mutex_;     // 数据互斥锁

        // 位置状态
        struct {
            double x{0.0};    // X位置(m)
            double y{0.0};    // Y位置(m)
            double yaw{0.0};  // 偏航角(rad)
        } pose_;

        // 速度状态
        struct {
            double linear_x{0.0};  // X速度(m/s)
            double linear_y{0.0};  // Y速度(m/s)
            double angular_z{0.0}; // 角速度(rad/s)
        } velocity_;

        // Modbus寄存器地址配置
        static constexpr uint32_t ODOM_REG_ADDR = 4928;  // 里程计数据起始地址
        static constexpr uint32_t VEL_REG_ADDR = 4960;   // 速度指令起始地址
    };

    // IMU处理类
    class ImuHandler {
    public:
        ImuHandler(ros::NodeHandle& nh,
                  const std::string& imu_frame,
                  const BridgeConfig& config);

        void updateImu(const std::vector<int32_t>& data); // 更新IMU数据
        ModbusHandler::Command getReadCommand(); // 获取读取命令
        void publish(const ros::Time& time); // 发布IMU数据

    private:
        ros::Publisher imu_pub_;    // IMU发布器
        const std::string frame_id_; // 坐标系ID
        const BridgeConfig& config_; // 配置参数引用
        mutable std::mutex data_mutex_;     // 数据互斥锁

        // IMU原始数据
        struct {
            double acc_x{0.0};   // X加速度(m/s²)
            double acc_y{0.0};   // Y加速度(m/s²)
            double acc_z{0.0};   // Z加速度(m/s²)
            double gyro_x{0.0};  // X角速度(rad/s)
            double gyro_y{0.0};  // Y角速度(rad/s)
            double gyro_z{0.0};  // Z角速度(rad/s)
        } imu_data_;

        // 缩放系数
        static constexpr double ACC_SCALE = 0.001;      // 加速度比例: 1 = 0.001m/s²
        static constexpr double GYRO_SCALE = 0.000001;  // 角速度比例: 1 = 0.000001rad/s

        // Modbus寄存器地址
        static constexpr uint32_t IMU_REG_ADDR = 4928;  // IMU数据起始地址
    };

private:
    ros::NodeHandle& nh_;             // ROS节点句柄
    std::unique_ptr<ModbusHandler> modbus_;    // Modbus处理器
    std::unique_ptr<OdometryHandler> odom_;    // 里程计处理器
    std::unique_ptr<ImuHandler> imu_;          // IMU处理器

    ros::Subscriber cmd_vel_sub_;     // 速度指令订阅器
    std::atomic<bool> running_{false}; // 运行状态
    BridgeConfig config_;             // 配置参数

    // 坐标系参数
    std::string odom_frame_id_{"odom"};
    std::string base_frame_id_{"base_footprint"};
    std::string imu_frame_id_{"imu_link"};

    bool loadParameters(); // 加载参数
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg); // 速度指令回调
};

#endif // AMR_ROS_BRIDGE_HPP
