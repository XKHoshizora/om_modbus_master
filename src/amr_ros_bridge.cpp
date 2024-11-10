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
    // 初始化从站
    om_modbus_master::om_query init_msg;
    init_msg.slave_id = 0x01;
    init_msg.func_code = 1;     // 写入功能
    init_msg.write_addr = 124;  // 驱动器输入命令地址
    init_msg.write_num = 1;     // 写入1个32位数据
    init_msg.data[0] = 0;       // 初始化为0

    int retry = 0;
    while (retry < MAX_RETRY) {
        query_pub_.publish(init_msg);
        ros::Duration(0.1).sleep();  // 等待响应

        if (!busy_ && error_ == 0) {
            ROS_INFO("Modbus通信初始化成功");
            return true;
        }

        ROS_WARN("Modbus初始化重试 %d/%d", retry + 1, MAX_RETRY);
        retry++;
        ros::Duration(1.0).sleep();
    }

    ROS_ERROR("Modbus通信初始化失败");
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

    if (msg->slave_id != 0x01) {
        return;  // 不是目标从站的响应
    }

    busy_ = false;
    error_ = 0;
}

void AmrRosBridge::ModbusHandler::handleState(
    const om_modbus_master::om_state::ConstPtr& msg) {

    error_ = msg->state_error;
    if (error_ != 0) {
        ROS_WARN("Modbus通信错误: %d", error_);
    }
}

// OdometryHandler实现
AmrRosBridge::OdometryHandler::OdometryHandler(
    ros::NodeHandle& nh,
    const std::string& odom_frame,
    const std::string& base_frame)
    : odom_frame_id_(odom_frame), base_frame_id_(base_frame) {

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);
}

void AmrRosBridge::OdometryHandler::updateOdometry(
    const std::vector<int32_t>& data) {

    std::lock_guard<std::mutex> lock(data_mutex_);

    // 检查数据有效性
    if (data.size() < 3) {
        ROS_WARN_THROTTLE(1.0, "里程计数据不完整");
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

    // 速度限制
    velocity_.linear_x = std::clamp(cmd->linear.x, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    velocity_.linear_y = std::clamp(cmd->linear.y, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    velocity_.angular_z = std::clamp(cmd->angular.z, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
}

AmrRosBridge::ModbusHandler::Command
AmrRosBridge::OdometryHandler::getReadCommand() const {
    Command cmd;
    cmd.slave_id = 0x01;
    cmd.func_code = 0;  // 读取功能
    cmd.addr = ODOM_REG_ADDR;
    cmd.data_num = 3;   // x, y, yaw
    cmd.type = CmdType::READ;
    return cmd;
}

AmrRosBridge::ModbusHandler::Command
AmrRosBridge::OdometryHandler::getWriteCommand() const {
    std::lock_guard<std::mutex> lock(data_mutex_);

    Command cmd;
    cmd.slave_id = 0x01;
    cmd.func_code = 1;  // 写入功能
    cmd.addr = VEL_REG_ADDR;
    cmd.data_num = 4;

    // 转换单位：m/s -> mm/s, rad/s -> μrad/s
    cmd.data[0] = 1;    // 运行模式
    cmd.data[1] = static_cast<int32_t>(velocity_.linear_x * 1000.0);
    cmd.data[2] = static_cast<int32_t>(velocity_.angular_z * 1000000.0);
    cmd.data[3] = static_cast<int32_t>(velocity_.linear_y * 1000.0);

    cmd.type = CmdType::WRITE;
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
    ros::NodeHandle& nh, const std::string& imu_frame)
    : frame_id_(imu_frame) {

    imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu", 50);
}

void AmrRosBridge::ImuHandler::updateImu(
    const std::vector<int32_t>& data) {

    std::lock_guard<std::mutex> lock(data_mutex_);

    if (data.size() < 6) {
        ROS_WARN_THROTTLE(1.0, "IMU数据不完整");
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
AmrRosBridge::ImuHandler::getReadCommand() const {
    Command cmd;
    cmd.slave_id = 0x01;
    cmd.func_code = 0;
    cmd.addr = IMU_REG_ADDR;
    cmd.data_num = 6;  // ax, ay, az, gx, gy, gz
    cmd.type = CmdType::READ;
    return cmd;
}

void AmrRosBridge::ImuHandler::publish(const ros::Time& time) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = time;
    imu_msg.header.frame_id = frame_id_;

    // 设置加速度
    imu_msg.linear_acceleration.x = imu_data_.acc_x;
    imu_msg.linear_acceleration.y = imu_data_.acc_y;
    imu_msg.linear_acceleration.z = imu_data_.acc_z;

    // 设置角速度
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
        throw std::runtime_error("参数加载失败");
    }

    // 创建各个处理器
    modbus_ = std::make_unique<ModbusHandler>(nh_);
    odom_ = std::make_unique<OdometryHandler>(
        nh_, odom_frame_id_, base_frame_id_);
    imu_ = std::make_unique<ImuHandler>(nh_, imu_frame_id_);

    // 创建速度指令订阅器
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1,
                                &AmrRosBridge::cmdVelCallback, this);

    ROS_INFO("AMR ROS Bridge已创建");
}

AmrRosBridge::~AmrRosBridge() {
    shutdown();
}

bool AmrRosBridge::loadParameters() {
    // 加载主循环频率
    nh_.param<double>("update_rate", config_.update_rate, 50.0);
    if (config_.update_rate <= 0 || config_.update_rate > 200) {
        ROS_WARN("无效的更新频率(%.1f), 使用默认值: %.1f Hz",
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

    // 设置更新周期
    odom_rate_.expected_period = 1.0 / config_.odom_rate;
    imu_rate_.expected_period = 1.0 / config_.imu_rate;

    ROS_INFO("参数加载完成:");
    ROS_INFO("- 主循环频率: %.1f Hz", config_.update_rate);
    ROS_INFO("- 里程计频率: %.1f Hz", config_.odom_rate);
    ROS_INFO("- IMU频率: %.1f Hz", config_.imu_rate);
    ROS_INFO("- 最大线速度: %.2f m/s", config_.max_linear_vel);
    ROS_INFO("- 最大角速度: %.2f rad/s", config_.max_angular_vel);

    return true;
}

bool AmrRosBridge::init() {
    ROS_INFO("正在初始化AMR ROS Bridge...");

    // 初始化Modbus通信
    if (!modbus_->init()) {
        ROS_ERROR("Modbus通信初始化失败");
        return false;
    }

    // 验证通信
    ModbusHandler::Command test_cmd;
    test_cmd.slave_id = 0x01;
    test_cmd.func_code = 0;
    test_cmd.addr = 4928;
    test_cmd.data_num = 1;
    test_cmd.type = ModbusHandler::CmdType::READ;

    if (!modbus_->sendCommand(test_cmd)) {
        ROS_ERROR("Modbus通信测试失败");
        return false;
    }

    running_ = true;
    ROS_INFO("AMR ROS Bridge初始化成功");
    return true;
}

void AmrRosBridge::cmdVelCallback(
    const geometry_msgs::Twist::ConstPtr& msg) {

    if (!running_) return;

    // 更新里程计命令
    odom_->updateCommand(msg);

    // 立即发送速度命令
    auto cmd = odom_->getWriteCommand();
    if (!modbus_->sendCommand(cmd)) {
        ROS_WARN_THROTTLE(1.0, "速度指令发送失败");
    }
}

void AmrRosBridge::run() {
    if (!running_) {
        ROS_WARN("AMR ROS Bridge未初始化");
        return;
    }

    ros::Rate rate(config_.update_rate);
    ROS_INFO("AMR ROS Bridge开始运行, 频率: %.1f Hz", config_.update_rate);

    while (ros::ok() && running_) {
        ros::Time current = ros::Time::now();

        // 里程计数据更新(使用频率控制)
        if (odom_rate_.need_update()) {
            auto cmd = odom_->getReadCommand();
            if (modbus_->sendCommand(cmd)) {
                std::vector<int32_t> odom_data(3);
                // ... 处理数据 ...
                odom_->updateOdometry(odom_data);
                odom_->publish(current);
            }
        }

        // IMU数据更新(使用频率控制)
        if (imu_rate_.need_update()) {
            auto cmd = imu_->getReadCommand();
            if (modbus_->sendCommand(cmd)) {
                std::vector<int32_t> imu_data(6);
                // ... 处理数据 ...
                imu_->updateImu(imu_data);
                imu_->publish(current);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
}

void AmrRosBridge::shutdown() {
    if (!running_) return;

    ROS_INFO("正在关闭AMR ROS Bridge...");
    running_ = false;

    // 发送停止命令
    ModbusHandler::Command stop_cmd;
    stop_cmd.slave_id = 0x01;
    stop_cmd.func_code = 1;
    stop_cmd.addr = 4960;
    stop_cmd.data_num = 4;
    std::fill(stop_cmd.data, stop_cmd.data + 4, 0);

    if (!modbus_->sendCommand(stop_cmd)) {
        ROS_WARN("停止命令发送失败");
    }

    ros::Duration(0.1).sleep();  // 等待命令执行
    ROS_INFO("AMR ROS Bridge已关闭");
}

// 主函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "amr_ros_bridge");
    ros::NodeHandle nh;

    try {
        AmrRosBridge bridge(nh);

        if (!bridge.init()) {
            ROS_ERROR("AMR ROS Bridge初始化失败");
            return 1;
        }

        bridge.run();
    }
    catch (const std::exception& e) {
        ROS_ERROR("AMR ROS Bridge异常: %s", e.what());
        return 1;
    }

    return 0;
}
