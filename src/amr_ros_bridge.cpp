/**
 * @file    amr_ros_bridge.cpp
 * @brief   AMR ROS桥接节点实现，负责与机器人进行Modbus通信，并将数据发布到ROS话题
 */

#include "om_modbus_master/amr_ros_bridge.hpp"

// ImuHandler构造函数
AmrRosBridge::ImuHandler::ImuHandler(ros::NodeHandle& nh, const std::string& imu_frame, const BridgeConfig& config)
    : frame_id_(imu_frame), config_(config) {
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu", 50);
}

// updateImu方法实现
void AmrRosBridge::ImuHandler::updateImu(const std::vector<int32_t>& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 数据解析示例（假设data包含加速度和角速度数据）
    if (data.size() >= 6) {
        imu_data_.acc_x = data[0] * ACC_SCALE;
        imu_data_.acc_y = data[1] * ACC_SCALE;
        imu_data_.acc_z = data[2] * ACC_SCALE;
        imu_data_.gyro_x = data[3] * GYRO_SCALE;
        imu_data_.gyro_y = data[4] * GYRO_SCALE;
        imu_data_.gyro_z = data[5] * GYRO_SCALE;
    }
}

// publish方法实现
void AmrRosBridge::ImuHandler::publish(const ros::Time& time) {
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = time;
    imu_msg.header.frame_id = frame_id_;
    imu_msg.linear_acceleration.x = imu_data_.acc_x;
    imu_msg.linear_acceleration.y = imu_data_.acc_y;
    imu_msg.linear_acceleration.z = imu_data_.acc_z;
    imu_msg.angular_velocity.x = imu_data_.gyro_x;
    imu_msg.angular_velocity.y = imu_data_.gyro_y;
    imu_msg.angular_velocity.z = imu_data_.gyro_z;

    imu_pub_.publish(imu_msg);
}

// OdometryHandler::getWriteCommand方法实现
AmrRosBridge::ModbusHandler::Command AmrRosBridge::OdometryHandler::getWriteCommand() {
    AmrRosBridge::ModbusHandler::Command cmd;
    cmd.slave_id = 0x01;
    cmd.func_code = 16; // 示例功能码
    cmd.addr = VEL_REG_ADDR;
    cmd.data_num = 4;
    cmd.data[0] = static_cast<int32_t>(velocity_.linear_x * 1000); // 假设单位转换
    cmd.data[1] = static_cast<int32_t>(velocity_.linear_y * 1000);
    cmd.data[2] = static_cast<int32_t>(velocity_.angular_z * 1000);
    return cmd;
}

// ModbusHandler类实现 - 处理Modbus协议的通信，包括发送命令和处理响应
AmrRosBridge::ModbusHandler::ModbusHandler(ros::NodeHandle& nh,
                                           OdometryHandler* odom,
                                           ImuHandler* imu)
    : odom_(odom), imu_(imu) {
    // 初始化Modbus通信的发布器和订阅器
    query_pub_ = nh.advertise<om_modbus_master::om_query>("om_query1", 10);
    response_sub_ = nh.subscribe("om_response1", 1,
                                &ModbusHandler::handleResponse, this);
    state_sub_ = nh.subscribe("om_state1", 1,
                             &ModbusHandler::handleState, this);
}

/**
 * @brief 初始化Modbus通信，等待ROS话题准备完毕并验证通信状态
 * @return true 如果初始化成功；false 如果初始化失败
 */
bool AmrRosBridge::ModbusHandler::init() {
    int retry_count = 0;               // 当前重试次数
    const int MAX_INIT_RETRY = 5;      // 最大初始化重试次数
    const double RETRY_DELAY = 1.0;    // 初始化失败后重试等待的秒数

    ROS_INFO("Initializing Modbus communication...");

    // 等待ROS节点和话题的准备就绪
    ros::Duration(0.5).sleep();

    // 检查Modbus主节点是否已连接，如果未连接则等待
    if (!query_pub_ || query_pub_.getNumSubscribers() == 0) {
        ROS_WARN("Waiting for Modbus master node to connect...");
        ros::Duration(1.0).sleep();
    }

    // 初始化Modbus通信的查询消息
    om_modbus_master::om_query init_msg;
    init_msg.slave_id = 0x01;       // 从站ID
    init_msg.func_code = 0;         // 读取功能码
    init_msg.read_addr = 4928;      // 起始读取地址
    init_msg.read_num = 1;          // 读取1个寄存器
    init_msg.write_num = 0;         // 不涉及写入操作

    // 循环尝试发送初始化指令并等待响应
    while (retry_count < MAX_INIT_RETRY && ros::ok()) {
        ROS_INFO("Attempting to initialize Modbus communication (%d/%d)...",
                 retry_count + 1, MAX_INIT_RETRY);

        busy_ = true;
        error_ = 0;
        query_pub_.publish(init_msg);

        // 等待响应，直到超时或收到响应
        ros::Time start_time = ros::Time::now();
        bool got_response = false;
        while (ros::Time::now() - start_time < ros::Duration(TIMEOUT)) {
            ros::spinOnce();
            if (!busy_ && error_ == 0) {
                got_response = true;
                break;
            }
            ros::Duration(0.01).sleep();
        }

        // 如果收到有效响应，则初始化成功
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

    // 如果超过最大重试次数，报告初始化失败
    ROS_ERROR("Modbus communication initialization failed");
    return false;
}

/**
 * @brief 发送Modbus命令并等待响应
 * @param cmd 要发送的命令数据
 * @return true 如果命令发送成功；false 如果发送失败
 */
bool AmrRosBridge::ModbusHandler::sendCommand(const Command& cmd) {
    std::lock_guard<std::mutex> lock(queue_mutex_);  // 保护命令队列的互斥锁

    // 检查通信是否在忙碌状态
    if (busy_) {
        ros::Time current = ros::Time::now();
        if ((current - last_busy_time_).toSec() > TIMEOUT) {
            ROS_DEBUG("Reset busy flag due to timeout");
            busy_ = false;
            error_ = 0;
        } else {
            return false;
        }
    }

    busy_ = true; // 标记通信为忙碌状态
    last_busy_time_ = ros::Time::now();
    error_ = 0;

    // 构建Modbus查询消息
    om_modbus_master::om_query msg;
    msg.slave_id = cmd.slave_id;
    msg.func_code = cmd.func_code;
    if (cmd.type == CmdType::READ) {
        msg.read_addr = cmd.addr;
        msg.read_num = cmd.data_num;
        msg.write_num = 0;
    } else if (cmd.type == CmdType::WRITE) {
        if (cmd.func_code == 2) {  // 读写功能
            msg.read_addr = cmd.read_addr;
            msg.read_num = cmd.read_num;
            msg.write_addr = cmd.write_addr;
            msg.write_num = cmd.write_num;
            for (int i = 0; i < cmd.write_num; i++) {
                msg.data[i] = cmd.data[i];
            }
        } else {
            msg.write_addr = cmd.addr;
            msg.write_num = cmd.data_num;
            for (int i = 0; i < cmd.data_num; i++) {
                msg.data[i] = cmd.data[i];
            }
        }
    } else if (cmd.type == CmdType::MONITOR) {
        msg.read_addr = cmd.addr;
        msg.read_num = cmd.data_num;
        msg.write_num = 0;
    }

    // 发布查询消息到Modbus主节点
    query_pub_.publish(msg);

    // 等待响应，若超时则返回失败
    ros::Time start = ros::Time::now();
    while (busy_ && (ros::Time::now() - start).toSec() < TIMEOUT) {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
    }

    if (busy_) {
        busy_ = false;
        error_ = 3;
        return false;
    }

    return error_ == 0;
}

/**
 * @brief 处理Modbus响应消息，根据响应数据更新里程计和IMU数据
 * @param msg 从Modbus主节点接收到的响应消息
 */
void AmrRosBridge::ModbusHandler::handleResponse(
    const om_modbus_master::om_response::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("Received empty Modbus response");
        error_ = 1;
        busy_ = false;
        return;
    }

    // 检查数据长度是否满足要求
    if (msg->func_code == 2 && msg->data.size() < 9) {
        ROS_WARN("Incomplete response data: got %d, expected 9", static_cast<int>(msg->data.size()));
        error_ = 4;
        busy_ = false;
        return;
    }

    // 更新里程计和IMU数据
    std::vector<int32_t> odom_data(msg->data.begin(), msg->data.begin() + 3);
    odom_->updateOdometry(odom_data);

    std::vector<int32_t> imu_data(msg->data.begin() + 3, msg->data.begin() + 9);
    imu_->updateImu(imu_data);

    error_ = 0;     // 重置错误状态
    busy_ = false;  // 标记通信完成
}

/**
 * @brief 处理Modbus状态消息，监控通信状态
 * @param msg Modbus主节点发送的状态消息
 */
void AmrRosBridge::ModbusHandler::handleState(
    const om_modbus_master::om_state::ConstPtr& msg) {

    if (!msg) return;

    // 检查状态错误
    if (msg->state_error != 0) {
        ROS_WARN("Modbus communication state error: %d", msg->state_error);
    }

    error_ = msg->state_error;
}

// OdometryHandler类实现 - 处理里程计数据
AmrRosBridge::OdometryHandler::OdometryHandler(
    ros::NodeHandle& nh,
    const std::string& odom_frame,
    const std::string& base_frame,
    const BridgeConfig& config)
    : odom_frame_id_(odom_frame),
      base_frame_id_(base_frame),
      config_(config) {

    // 初始化里程计话题的发布器
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);
}

/**
 * @brief 更新里程计数据
 * @param data 从Modbus响应中解析的里程计数据
 */
void AmrRosBridge::OdometryHandler::updateOdometry(
    const std::vector<int32_t>& data) {

    std::lock_guard<std::mutex> lock(data_mutex_); // 锁定数据互斥锁以确保线程安全

    // 检查数据是否完整
    if (data.size() < 3) {
        ROS_WARN_THROTTLE(1.0, "Incomplete odometry data");
        return;
    }

    // 解析里程计数据，将单位转换为米和弧度
    double new_x = data[0] / 1000.0;
    double new_y = data[1] / 1000.0;
    double new_yaw = data[2] / 1000000.0;

    // 检查位移和旋转是否显著，过滤掉微小的变化
    if (std::abs(new_x - pose_.x) < config_.min_valid_distance &&
        std::abs(new_y - pose_.y) < config_.min_valid_distance &&
        std::abs(new_yaw - pose_.yaw) < config_.min_valid_rotation) {
        return;
    }

    // 平滑更新里程计位置和偏航角，减小瞬时波动
    pose_.x = config_.odom_alpha * new_x + (1 - config_.odom_alpha) * pose_.x;
    pose_.y = config_.odom_alpha * new_y + (1 - config_.odom_alpha) * pose_.y;
    pose_.yaw = config_.odom_alpha * new_yaw + (1 - config_.odom_alpha) * pose_.yaw;
}

/**
 * @brief 根据ROS发布的速度指令更新速度
 * @param cmd 速度指令消息
 */
void AmrRosBridge::OdometryHandler::updateCommand(
    const geometry_msgs::Twist::ConstPtr& cmd) {

    std::lock_guard<std::mutex> lock(data_mutex_); // 锁定数据互斥锁

    // 限制线速度和角速度在配置的最大值范围内
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

/**
 * @brief 发布TF变换和里程计数据
 * @param time 当前时间戳
 */
void AmrRosBridge::OdometryHandler::publish(const ros::Time& time) {
    std::lock_guard<std::mutex> lock(data_mutex_);  // 确保数据的线程安全

    // 发布TF变换，描述里程计坐标和机器人底座坐标的关系
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

    // 创建并发布里程计消息
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

    // 设置里程计和速度的协方差矩阵
    for(int i = 0; i < 36; i++) {
        odom_msg.pose.covariance[i] = 0.0;
        odom_msg.twist.covariance[i] = 0.0;
    }
    odom_msg.pose.covariance[0] = 0.01;
    odom_msg.pose.covariance[7] = 0.01;
    odom_msg.pose.covariance[35] = 0.01;
    odom_msg.twist.covariance[0] = 0.01;
    odom_msg.twist.covariance[7] = 0.01;
    odom_msg.twist.covariance[35] = 0.01;

    odom_pub_.publish(odom_msg);
}

/**
 * @brief AmrRosBridge主类构造函数，加载参数并初始化Modbus、里程计和IMU处理器
 * @param nh ROS节点句柄
 */
AmrRosBridge::AmrRosBridge(ros::NodeHandle& nh) : nh_(nh) {
    // 加载ROS参数配置
    if (!loadParameters()) {
        throw std::runtime_error("Failed to load parameters");
    }

    // 创建处理器对象
    odom_ = std::make_unique<OdometryHandler>(
        nh_, odom_frame_id_, base_frame_id_, config_);
    imu_ = std::make_unique<ImuHandler>(
        nh_, imu_frame_id_, config_);

    modbus_ = std::make_unique<ModbusHandler>(
        nh_, odom_.get(), imu_.get());

    // 初始化速度指令订阅器
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1,
                                &AmrRosBridge::cmdVelCallback, this);

    ROS_INFO("AMR ROS Bridge created");
}

AmrRosBridge::~AmrRosBridge() {
    shutdown();
}

bool AmrRosBridge::loadParameters() {
    // 从ROS参数服务器加载参数
    nh_.param<double>("update_rate", config_.update_rate, 50.0);
    if (config_.update_rate <= 0 || config_.update_rate > 200) {
        ROS_WARN("Invalid update rate (%.1f), using default: %.1f Hz",
                 config_.update_rate, 50.0);
        config_.update_rate = 50.0;
    }

    nh_.param<double>("odom_rate", config_.odom_rate, 20.0);
    nh_.param<double>("imu_rate", config_.imu_rate, 100.0);
    nh_.param<double>("cmd_timeout", config_.cmd_timeout, 0.1);

    nh_.param<double>("max_linear_vel", config_.max_linear_vel, 2.0);
    nh_.param<double>("max_angular_vel", config_.max_angular_vel, 3.14);
    nh_.param<double>("min_valid_distance", config_.min_valid_distance, 0.001);
    nh_.param<double>("min_valid_rotation", config_.min_valid_rotation, 0.001);

    nh_.param<double>("odom_alpha", config_.odom_alpha, 0.8);
    nh_.param<double>("imu_alpha", config_.imu_alpha, 0.8);

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

// 初始化函数，配置Modbus通信等
bool AmrRosBridge::init() {
    ROS_INFO("Initializing AMR ROS Bridge...");

    if (!loadParameters()) {
        ROS_ERROR("Failed to load parameters");
        return false;
    }

    ros::Duration(0.5).sleep();

    if (!modbus_ || !modbus_->init()) {
        ROS_ERROR("Modbus communication initialization failed");
        return false;
    }

    ModbusHandler::Command addr_map_cmd;
    addr_map_cmd.slave_id = 0x01;
    addr_map_cmd.func_code = 1;
    addr_map_cmd.addr = 4864;
    addr_map_cmd.data_num = 32;

    addr_map_cmd.data[0] = 1069;
    addr_map_cmd.data[1] = 1070;
    addr_map_cmd.data[2] = 1071;
    addr_map_cmd.data[3] = 1038;
    addr_map_cmd.data[4] = 1039;
    addr_map_cmd.data[5] = 1040;
    addr_map_cmd.data[6] = 1041;
    addr_map_cmd.data[7] = 1042;
    addr_map_cmd.data[8] = 1043;
    addr_map_cmd.data[16] = 993;
    addr_map_cmd.data[17] = 994;
    addr_map_cmd.data[18] = 995;
    addr_map_cmd.data[19] = 996;

    addr_map_cmd.type = ModbusHandler::CmdType::WRITE;

    if (!modbus_->sendCommand(addr_map_cmd)) {
        ROS_ERROR("Failed to configure address mapping");
        return false;
    }

    ros::Duration(0.1).sleep();

    ModbusHandler::Command test_cmd;
    test_cmd.slave_id = 0x01;
    test_cmd.func_code = 2;
    test_cmd.read_addr = 4928;
    test_cmd.read_num = 9;
    test_cmd.write_addr = 4960;
    test_cmd.write_num = 4;
    test_cmd.data[0] = 0;
    test_cmd.data[1] = 0;
    test_cmd.data[2] = 0;
    test_cmd.data[3] = 0;
    test_cmd.type = ModbusHandler::CmdType::WRITE;

    int verify_retry = 0;
    const int MAX_VERIFY_RETRY = 5;

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

// cmd_vel回调函数，用于接收速度指令
void AmrRosBridge::cmdVelCallback(
    const geometry_msgs::Twist::ConstPtr& msg) {
    if (!running_) return;

    static ros::Time last_cmd_time = ros::Time::now();
    ros::Time current = ros::Time::now();

    if ((current - last_cmd_time).toSec() < 0.05) {
        return;
    }

    odom_->updateCommand(msg);

    auto cmd = odom_->getWriteCommand();
    int retry_count = 0;
    const int MAX_RETRY = 3;

    while (retry_count < MAX_RETRY) {
        if (modbus_->sendCommand(cmd)) {
            last_cmd_time = current;
            return;
        }
        retry_count++;
        ros::Duration(0.001).sleep();
    }

    ROS_WARN_THROTTLE(1.0, "Failed to send velocity command after %d retries", MAX_RETRY);
}

// 主循环
void AmrRosBridge::run() {
    if (!running_) {
        ROS_WARN("AMR ROS Bridge not initialized");
        return;
    }

    ros::Rate rate(config_.update_rate);
    ros::Time last_update_time = ros::Time::now();

    ROS_INFO("AMR ROS Bridge running, rate: %.1f Hz", config_.update_rate);

    while (ros::ok() && running_) {
        ros::Time current = ros::Time::now();

        auto cmd = odom_->getWriteCommand();
        if (modbus_->sendCommand(cmd)) {
            odom_->publish(current);
            imu_->publish(current);
            last_update_time = current;
        }

        ros::spinOnce();
        rate.sleep();
    }
}

// 关闭函数
void AmrRosBridge::shutdown() {
    if (!running_) return;

    ROS_INFO("Shutting down AMR ROS Bridge...");
    running_ = false;

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

    ros::Duration(0.1).sleep();
    ROS_INFO("AMR ROS Bridge shutdown complete");
}

// 主函数，初始化ROS节点并运行主循环
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
