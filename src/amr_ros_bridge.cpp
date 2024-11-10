/**
 * @file    amr_ros_bridge.cpp
 * @brief   AMR ROS桥接节点实现
 */

#include "om_modbus_master/amr_ros_bridge.hpp"

// ModbusHandler实现
AmrRosBridge::ModbusHandler::ModbusHandler(ros::NodeHandle& nh) {
    // 初始化发布器和订阅器
    query_pub_ = nh.advertise<om_modbus_master::om_query>("om_query1", 10);
    response_sub_ = nh.subscribe("om_response1", 1,
                                &ModbusHandler::handleResponse, this);
    state_sub_ = nh.subscribe("om_state1", 1,
                             &ModbusHandler::handleState, this);
}

bool AmrRosBridge::ModbusHandler::init() {
    int retry_count = 0;
    const int MAX_INIT_RETRY = 5;
    const double RETRY_DELAY = 1.0;  // 秒

    ROS_INFO("Initializing Modbus communication...");

    // 等待节点和话题准备就绪
    ros::Duration(0.5).sleep();  // 给ROS节点一些启动时间

    // 检查发布器和订阅器是否正确创建
    if (!query_pub_ || query_pub_.getNumSubscribers() == 0) {
        ROS_WARN("Waiting for Modbus master node to connect...");
        ros::Duration(1.0).sleep();  // 再给一些时间等待连接
    }

    // 发送初始化查询
    om_modbus_master::om_query init_msg;
    init_msg.slave_id = 0x01;    // 目标从站ID
    init_msg.func_code = 0;      // 读取功能
    init_msg.read_addr = 4928;   // 状态寄存器地址
    init_msg.read_num = 1;       // 读取一个寄存器
    init_msg.write_num = 0;

    while (retry_count < MAX_INIT_RETRY && ros::ok()) {
        ROS_INFO("Attempting to initialize Modbus communication (%d/%d)...",
                 retry_count + 1, MAX_INIT_RETRY);

        busy_ = true;
        error_ = 0;

        // 发送查询
        query_pub_.publish(init_msg);

        // 等待响应
        ros::Time start_time = ros::Time::now();
        bool got_response = false;

        while (ros::Time::now() - start_time < ros::Duration(TIMEOUT)) {
            ros::spinOnce();  // 处理回调
            if (!busy_ && error_ == 0) {
                got_response = true;
                break;
            }
            ros::Duration(0.01).sleep();
        }

        if (got_response) {
            ROS_INFO("Modbus communication initialized successfully");
            return true;
        }

        retry_count++;
        if (retry_count < MAX_INIT_RETRY) {
            ROS_WARN("Modbus initialization failed, retrying in %d seconds...",
                    static_cast<int>(RETRY_DELAY));
            ros::Duration(RETRY_DELAY).sleep();
        }
    }

    ROS_ERROR("Modbus communication initialization failed");
    return false;
}

bool AmrRosBridge::ModbusHandler::sendCommand(const Command& cmd) {
    if (busy_) {
        return false;
    }

    om_modbus_master::om_query msg;
    msg.slave_id = cmd.slave_id;
    msg.func_code = cmd.func_code;

    switch (cmd.type) {
        case CmdType::READ:
            msg.read_addr = cmd.addr;
            msg.read_num = cmd.data_num;
            msg.write_num = 0;
            break;

        case CmdType::WRITE:
            msg.write_addr = cmd.addr;
            msg.write_num = cmd.data_num;
            for (int i = 0; i < cmd.data_num; i++) {
                msg.data[i] = cmd.data[i];
            }
            break;

        case CmdType::MONITOR:
            msg.read_addr = cmd.addr;
            msg.read_num = cmd.data_num;
            msg.write_num = 0;
            break;
    }

    busy_ = true;
    query_pub_.publish(msg);

    // 等待响应
    ros::Time start = ros::Time::now();
    while (busy_ && (ros::Time::now() - start).toSec() < TIMEOUT) {
        ros::Duration(0.001).sleep();  // 1ms
    }

    return !busy_ && error_ == 0;
}

void AmrRosBridge::ModbusHandler::handleResponse(
    const om_modbus_master::om_response::ConstPtr& msg) {

    if (!msg) {
        ROS_ERROR("Received empty Modbus response");
        error_ = 1;
        busy_ = false;
        return;
    }

    // 检查响应的从站ID
    if (msg->slave_id != 0x01) {
        // 这可能是其他设备的响应，忽略它
        return;
    }

    // 检查功能码（如果有错误，功能码会有最高位置1）
    if (msg->func_code & 0x80) {
        ROS_ERROR("Modbus device returned an error, function code: 0x%02X", msg->func_code);
        error_ = 2;
        busy_ = false;
        return;
    }

    // 处理成功的响应
    error_ = 0;
    busy_ = false;
}

void AmrRosBridge::ModbusHandler::handleState(
    const om_modbus_master::om_state::ConstPtr& msg) {

    if (!msg) return;

    if (msg->state_error != 0) {
        ROS_WARN("Modbus communication state error: %d", msg->state_error);
    }

    error_ = msg->state_error;
}

// OdometryHandler实现
AmrRosBridge::OdometryHandler::OdometryHandler(
    ros::NodeHandle& nh,
    const std::string& odom_frame,
    const std::string& base_frame,
    const BridgeConfig& config)
    : odom_frame_id_(odom_frame),
      base_frame_id_(base_frame),
      config_(config) {

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);
}

void AmrRosBridge::OdometryHandler::updateOdometry(
    const std::vector<int32_t>& data) {

    std::lock_guard<std::mutex> lock(data_mutex_);

    // 检查数据有效性
    if (data.size() < 3) {
        ROS_WARN_THROTTLE(1.0, "Incomplete odometry data");
        return;
    }

    // 转换单位：mm -> m, μrad -> rad
    double new_x = data[0] / 1000.0;
    double new_y = data[1] / 1000.0;
    double new_yaw = data[2] / 1000000.0;

    // 检查数据变化是否超过最小阈值
    if (std::abs(new_x - pose_.x) < config_.min_valid_distance &&
        std::abs(new_y - pose_.y) < config_.min_valid_distance &&
        std::abs(new_yaw - pose_.yaw) < config_.min_valid_rotation) {
        return;  // 变化太小，忽略
    }

    // 应用平滑滤波
    pose_.x = config_.odom_alpha * new_x + (1 - config_.odom_alpha) * pose_.x;
    pose_.y = config_.odom_alpha * new_y + (1 - config_.odom_alpha) * pose_.y;
    pose_.yaw = config_.odom_alpha * new_yaw + (1 - config_.odom_alpha) * pose_.yaw;
}

void AmrRosBridge::OdometryHandler::updateCommand(
    const geometry_msgs::Twist::ConstPtr& cmd) {

    std::lock_guard<std::mutex> lock(data_mutex_);

    // 使用clamp限制速度范围
    velocity_.linear_x = std::clamp(cmd->linear.x,
                                   -config_.max_linear_vel,
                                   config_.max_linear_vel);
    velocity_.linear_y = std::clamp(cmd->linear.y,
                                   -config_.max_linear_vel,
                                   config_.max_linear_vel);
    velocity_.angular_z = std::clamp(cmd->angular.z,
                                    -config_.max_angular_vel,
                                    config_.max_angular_vel);
}

AmrRosBridge::ModbusHandler::Command
AmrRosBridge::OdometryHandler::getReadCommand() {
    ModbusHandler::Command cmd;
    cmd.slave_id = 0x01;
    cmd.func_code = 0;  // 读取功能
    cmd.addr = ODOM_REG_ADDR;
    cmd.data_num = 3;   // x, y, yaw
    cmd.type = ModbusHandler::CmdType::READ;
    return cmd;
}

AmrRosBridge::ModbusHandler::Command
AmrRosBridge::OdometryHandler::getWriteCommand() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    ModbusHandler::Command cmd;
    cmd.slave_id = 0x01;
    cmd.func_code = 1;  // 写入功能
    cmd.addr = VEL_REG_ADDR;
    cmd.data_num = 4;

    // 转换单位：m/s -> mm/s, rad/s -> μrad/s
    cmd.data[0] = 1;    // 运行模式
    cmd.data[1] = static_cast<int32_t>(velocity_.linear_x * 1000.0);
    cmd.data[2] = static_cast<int32_t>(velocity_.angular_z * 1000000.0);
    cmd.data[3] = static_cast<int32_t>(velocity_.linear_y * 1000.0);

    cmd.type = ModbusHandler::CmdType::WRITE;
    return cmd;
}

void AmrRosBridge::OdometryHandler::publish(const ros::Time& time) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 发布TF转换
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = time;
    tf_msg.header.frame_id = odom_frame_id_;
    tf_msg.child_frame_id = base_frame_id_;

    tf_msg.transform.translation.x = pose_.x;
    tf_msg.transform.translation.y = pose_.y;
    tf_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, pose_.yaw);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(tf_msg);

    // 发布里程计消息
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = time;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;

    odom_msg.pose.pose.position.x = pose_.x;
    odom_msg.pose.pose.position.y = pose_.y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf_msg.transform.rotation;

    odom_msg.twist.twist.linear.x = velocity_.linear_x;
    odom_msg.twist.twist.linear.y = velocity_.linear_y;
    odom_msg.twist.twist.angular.z = velocity_.angular_z;

    // 设置协方差
    for(int i = 0; i < 36; i++) {
        odom_msg.pose.covariance[i] = 0.0;
        odom_msg.twist.covariance[i] = 0.0;
    }
    odom_msg.pose.covariance[0] = 0.01;   // x
    odom_msg.pose.covariance[7] = 0.01;   // y
    odom_msg.pose.covariance[35] = 0.01;  // yaw
    odom_msg.twist.covariance[0] = 0.01;  // vx
    odom_msg.twist.covariance[7] = 0.01;  // vy
    odom_msg.twist.covariance[35] = 0.01; // vyaw

    odom_pub_.publish(odom_msg);
}

// ImuHandler实现
AmrRosBridge::ImuHandler::ImuHandler(
    ros::NodeHandle& nh,
    const std::string& imu_frame,
    const BridgeConfig& config)
    : frame_id_(imu_frame), config_(config) {

    imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu", 50);
}

void AmrRosBridge::ImuHandler::updateImu(
    const std::vector<int32_t>& data) {

    std::lock_guard<std::mutex> lock(data_mutex_);

    if (data.size() < 6) {
        ROS_WARN_THROTTLE(1.0, "Incomplete IMU data");
        return;
    }

    // 转换并应用平滑滤波
    double new_acc_x = data[0] * ACC_SCALE;
    double new_acc_y = data[1] * ACC_SCALE;
    double new_acc_z = data[2] * ACC_SCALE;
    double new_gyro_x = data[3] * GYRO_SCALE;
    double new_gyro_y = data[4] * GYRO_SCALE;
    double new_gyro_z = data[5] * GYRO_SCALE;

    imu_data_.acc_x = config_.imu_alpha * new_acc_x +
                      (1 - config_.imu_alpha) * imu_data_.acc_x;
    imu_data_.acc_y = config_.imu_alpha * new_acc_y +
                      (1 - config_.imu_alpha) * imu_data_.acc_y;
    imu_data_.acc_z = config_.imu_alpha * new_acc_z +
                      (1 - config_.imu_alpha) * imu_data_.acc_z;
    imu_data_.gyro_x = config_.imu_alpha * new_gyro_x +
                       (1 - config_.imu_alpha) * imu_data_.gyro_x;
    imu_data_.gyro_y = config_.imu_alpha * new_gyro_y +
                       (1 - config_.imu_alpha) * imu_data_.gyro_y;
    imu_data_.gyro_z = config_.imu_alpha * new_gyro_z +
                       (1 - config_.imu_alpha) * imu_data_.gyro_z;
}

AmrRosBridge::ModbusHandler::Command
AmrRosBridge::ImuHandler::getReadCommand() {
    ModbusHandler::Command cmd;
    cmd.slave_id = 0x01;
    cmd.func_code = 0;
    cmd.addr = IMU_REG_ADDR;
    cmd.data_num = 6;  // ax, ay, az, gx, gy, gz
    cmd.type = ModbusHandler::CmdType::READ;
    return cmd;
}

void AmrRosBridge::ImuHandler::publish(const ros::Time& time) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = time;
    imu_msg.header.frame_id = frame_id_;

    // 设置加速度数据
    imu_msg.linear_acceleration.x = imu_data_.acc_x;
    imu_msg.linear_acceleration.y = imu_data_.acc_y;
    imu_msg.linear_acceleration.z = imu_data_.acc_z;

    // 设置角速度数据
    imu_msg.angular_velocity.x = imu_data_.gyro_x;
    imu_msg.angular_velocity.y = imu_data_.gyro_y;
    imu_msg.angular_velocity.z = imu_data_.gyro_z;

    // 姿态未知，使用单位四元数
    imu_msg.orientation.w = 1.0;
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;

    // 设置协方差
    for(int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i] = -1;  // 姿态未知
        imu_msg.angular_velocity_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
        imu_msg.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
    }

    imu_pub_.publish(imu_msg);
}

// AmrRosBridge主类实现
AmrRosBridge::AmrRosBridge(ros::NodeHandle& nh) : nh_(nh) {
    // 加载参数
    if (!loadParameters()) {
        throw std::runtime_error("Failed to load parameters");
    }

    // 创建各个处理器
    modbus_ = std::make_unique<ModbusHandler>(nh_);
    odom_ = std::make_unique<OdometryHandler>(
        nh_, odom_frame_id_, base_frame_id_, config_);
    imu_ = std::make_unique<ImuHandler>(
        nh_, imu_frame_id_, config_);

    // 创建速度指令订阅器
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1,
                                &AmrRosBridge::cmdVelCallback, this);

    ROS_INFO("AMR ROS Bridge created");
}

AmrRosBridge::~AmrRosBridge() {
    shutdown();
}

bool AmrRosBridge::loadParameters() {
    // 加载更新频率
    nh_.param<double>("update_rate", config_.update_rate, 50.0);
    if (config_.update_rate <= 0 || config_.update_rate > 200) {
        ROS_WARN("Invalid update rate (%.1f), using default: %.1f Hz",
                 config_.update_rate, 50.0);
        config_.update_rate = 50.0;
    }

    // 加载数据更新频率
    nh_.param<double>("odom_rate", config_.odom_rate, 20.0);
    nh_.param<double>("imu_rate", config_.imu_rate, 100.0);
    nh_.param<double>("cmd_timeout", config_.cmd_timeout, 0.1);

    // 加载运动参数
    nh_.param<double>("max_linear_vel", config_.max_linear_vel, 2.0);
    nh_.param<double>("max_angular_vel", config_.max_angular_vel, 3.14);
    nh_.param<double>("min_valid_distance", config_.min_valid_distance, 0.001);
    nh_.param<double>("min_valid_rotation", config_.min_valid_rotation, 0.001);

    // 加载滤波参数
    nh_.param<double>("odom_alpha", config_.odom_alpha, 0.8);
    nh_.param<double>("imu_alpha", config_.imu_alpha, 0.8);

    // 加载坐标系参数
    nh_.param<std::string>("base_frame", base_frame_id_, "base_footprint");
    nh_.param<std::string>("odom_frame", odom_frame_id_, "odom");
    nh_.param<std::string>("imu_frame", imu_frame_id_, "imu_link");

    ROS_INFO("Parameters loaded:");
    ROS_INFO("- Main loop rate: %.1f Hz", config_.update_rate);
    ROS_INFO("- Odometry rate: %.1f Hz", config_.odom_rate);
    ROS_INFO("- IMU rate: %.1f Hz", config_.imu_rate);
    ROS_INFO("- Max linear velocity: %.2f m/s", config_.max_linear_vel);
    ROS_INFO("- Max angular velocity: %.2f rad/s", config_.max_angular_vel);
    ROS_INFO("- Frame configuration:");
    ROS_INFO("  - Base: %s", base_frame_id_.c_str());
    ROS_INFO("  - Odometry: %s", odom_frame_id_.c_str());
    ROS_INFO("  - IMU: %s", imu_frame_id_.c_str());

    return true;
}

bool AmrRosBridge::init() {
    ROS_INFO("Initializing AMR ROS Bridge...");

    // 1. 首先加载并验证参数
    if (!loadParameters()) {
        ROS_ERROR("Failed to load parameters");
        return false;
    }

    // 2. 等待ROS系统就绪
    ros::Duration(0.5).sleep();

    // 3. 初始化Modbus通信
    if (!modbus_ || !modbus_->init()) {
        ROS_ERROR("Modbus communication initialization failed");
        return false;
    }

    // 4. 验证通信
    ModbusHandler::Command test_cmd;
    test_cmd.slave_id = 0x01;
    test_cmd.func_code = 0;      // 读取功能
    test_cmd.addr = 4928;        // 状态寄存器地址
    test_cmd.data_num = 1;       // 读取一个寄存器
    test_cmd.type = ModbusHandler::CmdType::READ;

    int verify_retry = 0;
    const int MAX_VERIFY_RETRY = 3;

    while (verify_retry < MAX_VERIFY_RETRY) {
        if (modbus_->sendCommand(test_cmd)) {
            ROS_INFO("AMR ROS Bridge initialized successfully");
            running_ = true;
            return true;
        }
        ROS_WARN("Communication verification failed, retrying... (%d/%d)",
                 verify_retry + 1, MAX_VERIFY_RETRY);
        ros::Duration(1.0).sleep();
        verify_retry++;
    }

    ROS_ERROR("AMR ROS Bridge initialization failed: Unable to verify Modbus communication");
    return false;
}

void AmrRosBridge::cmdVelCallback(
    const geometry_msgs::Twist::ConstPtr& msg) {

    if (!running_) return;

    // 更新里程计命令
    odom_->updateCommand(msg);

    // 立即发送速度命令
    auto cmd = odom_->getWriteCommand();
    if (!modbus_->sendCommand(cmd)) {
        ROS_WARN_THROTTLE(1.0, "Failed to send velocity command");
    }
}

void AmrRosBridge::run() {
    if (!running_) {
        ROS_WARN("AMR ROS Bridge not initialized");
        return;
    }

    ros::Rate rate(config_.update_rate);
    double odom_period = 1.0 / config_.odom_rate;
    double imu_period = 1.0 / config_.imu_rate;

    ros::Time last_odom_time = ros::Time::now();
    ros::Time last_imu_time = ros::Time::now();

    ROS_INFO("AMR ROS Bridge running, rate: %.1f Hz", config_.update_rate);

    while (ros::ok() && running_) {
        ros::Time current = ros::Time::now();

        // 读取里程计数据
        if ((current - last_odom_time).toSec() >= odom_period) {
            auto cmd = odom_->getReadCommand();
            if (modbus_->sendCommand(cmd)) {
                // TODO: 从响应中获取数据
                std::vector<int32_t> odom_data(3, 0);
                odom_->updateOdometry(odom_data);
                odom_->publish(current);
                last_odom_time = current;
            }
        }

        // 读取IMU数据
        if ((current - last_imu_time).toSec() >= imu_period) {
            auto cmd = imu_->getReadCommand();
            if (modbus_->sendCommand(cmd)) {
                // TODO: 从响应中获取数据
                std::vector<int32_t> imu_data(6, 0);
                imu_->updateImu(imu_data);
                imu_->publish(current);
                last_imu_time = current;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
}

void AmrRosBridge::shutdown() {
    if (!running_) return;

    ROS_INFO("Shutting down AMR ROS Bridge...");
    running_ = false;

    // 发送停止命令
    ModbusHandler::Command stop_cmd;
    stop_cmd.slave_id = 0x01;
    stop_cmd.func_code = 1;
    stop_cmd.addr = 4960;
    stop_cmd.data_num = 4;
    std::fill(stop_cmd.data, stop_cmd.data + 4, 0);
    stop_cmd.type = ModbusHandler::CmdType::WRITE;

    if (!modbus_->sendCommand(stop_cmd)) {
        ROS_WARN("Failed to send stop command");
    }

    ros::Duration(0.1).sleep();  // 等待命令执行
    ROS_INFO("AMR ROS Bridge shutdown complete");
}

// 主函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "amr_ros_bridge");
    ros::NodeHandle nh;

    try {
        AmrRosBridge bridge(nh);

        if (!bridge.init()) {
            ROS_ERROR("AMR ROS Bridge initialization failed");
            return 1;
        }

        bridge.run();
    }
    catch (const std::exception& e) {
        ROS_ERROR("AMR ROS Bridge exception: %s", e.what());
        return 1;
    }

    return 0;
}
