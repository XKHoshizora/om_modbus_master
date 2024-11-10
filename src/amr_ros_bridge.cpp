/** @file    amr_ros_bridge.cpp
 *  @brief   AMR ROS Bridge节点，用于处理机器人的里程计和IMU数据并发布TF变换
 */

#include "om_modbus_master/amr_ros_bridge.hpp"

// ModbusComm 实现
AmrRosBridge::ModbusComm::ModbusComm(ros::NodeHandle& nh) {
    query_pub_ = nh.advertise<om_modbus_master::om_query>("om_query1", 1);
    last_spin_time_ = ros::Time::now();
}

bool AmrRosBridge::ModbusComm::init() {
    ROS_INFO("Initializing Modbus communication...");

    // 初始化设备
    om_modbus_master::om_query msg;
    msg.slave_id = 0x00;   // 广播地址
    msg.func_code = 1;     // 写入功能
    msg.write_addr = 124;  // 驱动器输入命令地址
    msg.write_num = 1;     // 写入1个32位数据
    msg.data[0] = 0;       // 所有位置为OFF

    // 尝试与设备建立通信
    int init_retry = 0;
    const int MAX_INIT_RETRY = 5;

    while (init_retry < MAX_INIT_RETRY) {
        query_pub_.publish(msg);

        if (waitForResponse()) {
            ROS_INFO("Modbus communication initialized successfully");
            return true;
        }

        ROS_WARN("Modbus initialization attempt %d/%d failed", init_retry + 1,
                 MAX_INIT_RETRY);
        ros::Duration(1.0).sleep();
        init_retry++;
    }

    ROS_ERROR("Failed to initialize Modbus communication after %d attempts",
              MAX_INIT_RETRY);
    return false;
}

bool AmrRosBridge::ModbusComm::waitForResponse() {
    ros::Duration(0.03).sleep();

    ros::Time start_time = ros::Time::now();
    while (state_driver_ == 1) {
        if ((ros::Time::now() - start_time).toSec() > COMM_TIMEOUT) {
            ROS_WARN_THROTTLE(1.0, "Communication timeout");
            return false;
        }

        if ((ros::Time::now() - last_spin_time_).toSec() >= 0.01) {
            ros::spinOnce();
            last_spin_time_ = ros::Time::now();
        }
        ros::Duration(SPIN_SLEEP_TIME).sleep();
    }

    if (state_error_ != 0) {
        ROS_WARN_THROTTLE(1.0, "Communication error: %d", state_error_.load());
        return false;
    }

    return true;
}

bool AmrRosBridge::ModbusComm::sendAndReceive(om_modbus_master::om_query& msg) {
    static int retry_count = 0;

    if (retry_count >= MAX_CONSECUTIVE_RETRIES) {
        ros::Duration(0.1).sleep();
    }

    query_pub_.publish(msg);

    if (!waitForResponse()) {
        retry_count++;
        if (retry_count > MAX_CONSECUTIVE_RETRIES) {
            ROS_ERROR_THROTTLE(1.0, "Multiple communication failures");
        }
        return false;
    }

    retry_count = 0;
    return true;
}

void AmrRosBridge::ModbusComm::setState(
    const om_modbus_master::om_state& state) {
    state_driver_ = state.state_driver;
    state_mes_ = state.state_mes;
    state_error_ = state.state_error;
}

// OdometryHandler 实现
AmrRosBridge::OdometryHandler::OdometryHandler(ros::NodeHandle& nh,
                                               const std::string& odom_frame_id,
                                               const std::string& base_frame_id)
    : odom_frame_id_(odom_frame_id), base_frame_id_(base_frame_id) {
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);
}

void AmrRosBridge::OdometryHandler::updateCommand(
    const geometry_msgs::Twist& twist) {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_vel_x_ = twist.linear.x;
    cmd_vel_y_ = twist.linear.y;
    cmd_vel_th_ = twist.angular.z;
}

void AmrRosBridge::OdometryHandler::updateData(
    const om_modbus_master::om_response& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    odom_pos_x_ = msg.data[0] / 1000.0;
    odom_pos_y_ = msg.data[1] / 1000.0;
    odom_pos_th_ = msg.data[2] / 1000000.0;
}

int32_t AmrRosBridge::OdometryHandler::getVelX() const {
    return static_cast<int32_t>(std::round(cmd_vel_x_ * 1000.0));
}

int32_t AmrRosBridge::OdometryHandler::getVelY() const {
    return static_cast<int32_t>(std::round(cmd_vel_y_ * 1000.0));
}

int32_t AmrRosBridge::OdometryHandler::getVelTh() const {
    return static_cast<int32_t>(std::round(cmd_vel_th_ * 1000000.0));
}

void AmrRosBridge::OdometryHandler::publish(const ros::Time& current_time) {
    std::lock_guard<std::mutex> lock(mutex_);

    tf2::Transform odom_tf;
    tf2::Quaternion q;
    odom_tf.setOrigin(tf2::Vector3(odom_pos_x_, odom_pos_y_, 0.0));
    q.setRPY(0, 0, odom_pos_th_);
    odom_tf.setRotation(q);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom_frame_id_;
    odom_trans.child_frame_id = base_frame_id_;
    tf2::convert(odom_tf, odom_trans.transform);
    tf_broadcaster_.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.position.x = odom_pos_x_;
    odom.pose.pose.position.y = odom_pos_y_;
    odom.pose.pose.position.z = 0.0;
    tf2::convert(q, odom.pose.pose.orientation);

    odom.twist.twist.linear.x = cmd_vel_x_;
    odom.twist.twist.linear.y = cmd_vel_y_;
    odom.twist.twist.angular.z = cmd_vel_th_;

    odom_pub_.publish(odom);
}

// ImuHandler 实现
AmrRosBridge::ImuHandler::ImuHandler(ros::NodeHandle& nh,
                                     const std::string& imu_frame_id)
    : imu_frame_id_(imu_frame_id) {
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data", 50);
}

void AmrRosBridge::ImuHandler::updateData(
    const om_modbus_master::om_response& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    acc_x_ = msg.data[3] * ACC_SCALE;
    acc_y_ = msg.data[4] * ACC_SCALE;
    acc_z_ = msg.data[5] * ACC_SCALE;
    gyro_x_ = msg.data[6] * GYRO_SCALE;
    gyro_y_ = msg.data[7] * GYRO_SCALE;
    gyro_z_ = msg.data[8] * GYRO_SCALE;
}

void AmrRosBridge::ImuHandler::publish(const ros::Time& current_time) {
    std::lock_guard<std::mutex> lock(mutex_);

    sensor_msgs::Imu imu;
    imu.header.stamp = current_time;
    imu.header.frame_id = imu_frame_id_;

    imu.linear_acceleration.x = acc_x_;
    imu.linear_acceleration.y = acc_y_;
    imu.linear_acceleration.z = acc_z_;

    imu.angular_velocity.x = gyro_x_;
    imu.angular_velocity.y = gyro_y_;
    imu.angular_velocity.z = gyro_z_;

    for (int i = 0; i < 9; i++) {
        imu.orientation_covariance[i] = -1;
        imu.angular_velocity_covariance[i] = 0.01;
        imu.linear_acceleration_covariance[i] = 0.01;
    }

    imu_pub_.publish(imu);
}

// 主类实现
AmrRosBridge::AmrRosBridge(ros::NodeHandle& nh) : nh_(nh), running_(false) {
    loadParameters();

    modbus_ = std::make_unique<ModbusComm>(nh_);
    odom_ =
        std::make_unique<OdometryHandler>(nh_, odom_frame_id_, base_frame_id_);
    imu_ = std::make_unique<ImuHandler>(nh_, imu_frame_id_);

    cmd_vel_sub_ =
        nh_.subscribe("cmd_vel", 1, &AmrRosBridge::cmdVelCallback, this);
    modbus_response_sub_ = nh_.subscribe(
        "om_response1", 1, &AmrRosBridge::modbusResponseCallback, this);
    modbus_state_sub_ =
        nh_.subscribe("om_state1", 1, &AmrRosBridge::modbusStateCallback, this);
}

AmrRosBridge::~AmrRosBridge() { shutdown(); }

void AmrRosBridge::loadParameters() {
    double device_rate;
    nh_.param<double>("modbus_device_rate", device_rate,
                      20.0);  // 默认值改为20.0以匹配launch文件
    nh_.param<double>("update_rate", update_rate_,
                      20.0);  // 默认值改为20.0以匹配launch文件

    // 加载TF frame参数
    nh_.param<std::string>("base_frame", base_frame_id_, "base_footprint");
    nh_.param<std::string>("odom_frame", odom_frame_id_, "odom");
    nh_.param<std::string>("imu_frame", imu_frame_id_, "imu_link");
}

void AmrRosBridge::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    odom_->updateCommand(*msg);
}

void AmrRosBridge::modbusResponseCallback(
    const om_modbus_master::om_response::ConstPtr& msg) {
    if (msg->slave_id == 1) {
        odom_->updateData(*msg);
        imu_->updateData(*msg);
    }
}

void AmrRosBridge::modbusStateCallback(
    const om_modbus_master::om_state::ConstPtr& msg) {
    modbus_->setState(*msg);
}

bool AmrRosBridge::setupModbusRegisters(om_modbus_master::om_query& msg) {
    msg.slave_id = 0x01;
    msg.func_code = 1;
    msg.write_addr = 4864;
    msg.write_num = 32;

    // 设置寄存器地址
    msg.data[0] = 1069;  // 里程计X
    msg.data[1] = 1070;  // 里程计Y
    msg.data[2] = 1071;  // 里程计Theta
    msg.data[3] = 1038;  // 加速度X
    msg.data[4] = 1039;  // 加速度Y
    msg.data[5] = 1040;  // 加速度Z
    msg.data[6] = 1041;  // 角速度ROLL
    msg.data[7] = 1042;  // 角速度PITCH
    msg.data[8] = 1043;  // 角速度YAW
    msg.data[16] = 993;  // 直接数据运行模式
    msg.data[17] = 994;  // 前后平移速度(Vx)
    msg.data[18] = 995;  // 角速度(ω)
    msg.data[19] = 996;  // 左右平移速度(Vy)

    // 发送并等待响应
    if (!modbus_->sendAndReceive(msg)) {
        ROS_ERROR("Failed to setup Modbus registers");
        return false;
    }

    return true;
}

bool AmrRosBridge::init() {
    ROS_INFO("Initializing AMR ROS Bridge...");

    // 等待系统稳定
    ros::Duration(1.0).sleep();

    // 初始化Modbus通信
    if (!modbus_->init()) {
        ROS_ERROR("Failed to initialize Modbus communication");
        return false;
    }

    // 设置Modbus寄存器
    om_modbus_master::om_query init_msg;
    if (!setupModbusRegisters(init_msg)) {
        return false;
    }

    running_ = true;
    ROS_INFO("AMR ROS Bridge initialized successfully");
    return true;
}

void AmrRosBridge::run() {
    if (!running_) {
        ROS_WARN("AMR ROS Bridge not initialized!");
        return;
    }

    ros::Rate loop_rate(update_rate_);
    om_modbus_master::om_query msg;

    ROS_INFO("AMR ROS Bridge running at %.1f Hz", update_rate_);

    while (ros::ok() && running_) {
        ros::Time current_time = ros::Time::now();

        // 准备并发送Modbus命令
        msg.slave_id = 0x01;
        msg.func_code = 2;
        msg.read_addr = 4928;
        msg.read_num = 9;
        msg.write_addr = 4960;
        msg.write_num = 4;

        // 获取当前速度命令
        msg.data[0] = 1;
        msg.data[1] = odom_->getVelX();
        msg.data[2] = odom_->getVelTh();
        msg.data[3] = odom_->getVelY();

        // 发送命令并等待响应
        if (!modbus_->sendAndReceive(msg)) {
            ROS_WARN_THROTTLE(1.0, "Failed to communicate with device");
            ros::Duration(0.1).sleep();
            continue;
        }

        // 发布里程计和IMU数据
        odom_->publish(current_time);
        imu_->publish(current_time);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void AmrRosBridge::shutdown() {
    if (!running_) {
        return;
    }

    ROS_INFO("Shutting down AMR ROS Bridge...");
    running_ = false;

    // 发送停止命令
    om_modbus_master::om_query msg;
    msg.slave_id = 0x01;
    msg.func_code = 1;
    msg.write_addr = 4960;
    msg.write_num = 4;
    msg.data[0] = 0;
    msg.data[1] = 0;
    msg.data[2] = 0;
    msg.data[3] = 0;

    modbus_->sendAndReceive(msg);

    ros::Duration(0.5).sleep();  // 等待命令执行完成
    ROS_INFO("AMR ROS Bridge shutdown complete");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "amr_ros_bridge");
    ros::NodeHandle nh;

    // 创建异步处理器
    ros::AsyncSpinner spinner(4);  // 使用4个线程以适应更高频率的消息处理
    spinner.start();

    try {
        AmrRosBridge bridge(nh);

        if (!bridge.init()) {
            ROS_ERROR("Failed to initialize AMR ROS Bridge");
            return 1;
        }

        bridge.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in AMR ROS Bridge: %s", e.what());
        return 1;
    }

    spinner.stop();
    return 0;
}
