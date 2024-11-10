#include "om_modbus_master/amr_ros_bridge.hpp"

AmrRosBridge::AmrRosBridge(ros::NodeHandle& nh) : nh_(nh) {
    loadParameters();

    // 初始化发布器和订阅器
    query_pub_ = nh_.advertise<om_modbus_master::om_query>("om_query1", 1);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 50);

    response_sub_ = nh_.subscribe("om_response1", 1,
                                 &AmrRosBridge::responseCallback, this);
    state_sub_ = nh_.subscribe("om_state1", 1,
                              &AmrRosBridge::stateCallback, this);
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1,
                                &AmrRosBridge::cmdVelCallback, this);
}

AmrRosBridge::~AmrRosBridge() {
    shutdown();
}

bool AmrRosBridge::loadParameters() {
    nh_.param<std::string>("base_frame", base_frame_, "base_footprint");
    nh_.param<std::string>("odom_frame", odom_frame_, "odom");
    nh_.param<std::string>("imu_frame", imu_frame_, "imu_link");
    nh_.param<double>("update_rate", update_rate_, 20.0);
    return true;
}

bool AmrRosBridge::init() {
    ROS_INFO("Initializing AMR ROS Bridge...");

    // 设置寄存器映射
    std::vector<int32_t> map_data = {
        1069, 1070, 1071,  // 里程计 X,Y,Theta
        1038, 1039, 1040,  // IMU加速度
        1041, 1042, 1043,  // IMU角速度
        0, 0, 0, 0, 0, 0, 0,
        993, 994, 995, 996 // 速度控制
    };

    if (!sendModbusCommand(0x01, 1, 0, 0, 4864, map_data.size(), map_data)) {
        ROS_ERROR("Failed to configure register mapping");
        return false;
    }

    ros::Duration(0.1).sleep();
    running_ = true;
    return true;
}

void AmrRosBridge::run() {
    if (!running_) {
        ROS_WARN("AMR ROS Bridge not initialized");
        return;
    }

    ros::Rate rate(update_rate_);
    while (ros::ok() && running_) {
        // 读取里程计和IMU数据
        std::vector<int32_t> cmd_data = {1, 0, 0, 0}; // 默认速度指令
        if (!busy_) {
            sendModbusCommand(0x01, 2, 4928, 9, 4960, 4, cmd_data);
        }

        ros::Time current = ros::Time::now();
        publishOdomAndTf(current);
        publishImu(current);

        ros::spinOnce();
        rate.sleep();
    }
}

void AmrRosBridge::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 限制速度在安全范围内
    double vx = std::clamp(msg->linear.x, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    double vy = std::clamp(msg->linear.y, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    double vth = std::clamp(msg->angular.z, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

    // 发送速度指令
    std::vector<int32_t> cmd_data = {
        1,  // 使能
        static_cast<int32_t>(vx * 1000),
        static_cast<int32_t>(vth * 1000000),
        static_cast<int32_t>(vy * 1000)
    };

    sendModbusCommand(0x01, 1, 0, 0, 4960, 4, cmd_data);
}

void AmrRosBridge::responseCallback(
    const om_modbus_master::om_response::ConstPtr& msg) {

    if (!msg || msg->data.size() < 9) {
        busy_ = false;
        return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    // 更新里程计数据
    odom_data_.x = msg->data[0] / 1000.0;
    odom_data_.y = msg->data[1] / 1000.0;
    odom_data_.yaw = msg->data[2] / 1000000.0;

    // 更新IMU数据
    imu_data_.acc_x = msg->data[3] * ACC_SCALE;
    imu_data_.acc_y = msg->data[4] * ACC_SCALE;
    imu_data_.acc_z = msg->data[5] * ACC_SCALE;
    imu_data_.gyro_x = msg->data[6] * GYRO_SCALE;
    imu_data_.gyro_y = msg->data[7] * GYRO_SCALE;
    imu_data_.gyro_z = msg->data[8] * GYRO_SCALE;

    busy_ = false;
}

void AmrRosBridge::stateCallback(const om_modbus_master::om_state::ConstPtr& msg) {
    if (msg) {
        error_ = msg->state_error;
    }
}

void AmrRosBridge::publishOdomAndTf(const ros::Time& time) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 创建并发布TF
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = time;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = odom_data_.x;
    tf_msg.transform.translation.y = odom_data_.y;
    tf_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, odom_data_.yaw);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(tf_msg);

    // 创建并发布Odometry消息
    nav_msgs::Odometry odom_msg;
    odom_msg.header = tf_msg.header;
    odom_msg.child_frame_id = tf_msg.child_frame_id;

    odom_msg.pose.pose.position.x = odom_data_.x;
    odom_msg.pose.pose.position.y = odom_data_.y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf_msg.transform.rotation;

    odom_msg.twist.twist.linear.x = odom_data_.vx;
    odom_msg.twist.twist.linear.y = odom_data_.vy;
    odom_msg.twist.twist.angular.z = odom_data_.vth;

    odom_pub_.publish(odom_msg);
}

void AmrRosBridge::publishImu(const ros::Time& time) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = time;
    imu_msg.header.frame_id = imu_frame_;

    imu_msg.linear_acceleration.x = imu_data_.acc_x;
    imu_msg.linear_acceleration.y = imu_data_.acc_y;
    imu_msg.linear_acceleration.z = imu_data_.acc_z;

    imu_msg.angular_velocity.x = imu_data_.gyro_x;
    imu_msg.angular_velocity.y = imu_data_.gyro_y;
    imu_msg.angular_velocity.z = imu_data_.gyro_z;

    imu_pub_.publish(imu_msg);
}

bool AmrRosBridge::sendModbusCommand(uint8_t slave_id, uint8_t func_code,
                                    uint16_t read_addr, uint16_t read_num,
                                    uint16_t write_addr, uint16_t write_num,
                                    const std::vector<int32_t>& data) {
    if (busy_) return false;

    om_modbus_master::om_query msg;
    msg.slave_id = slave_id;
    msg.func_code = func_code;
    msg.read_addr = read_addr;
    msg.read_num = read_num;
    msg.write_addr = write_addr;
    msg.write_num = write_num;

    if (!data.empty()) {
        std::copy(data.begin(), data.end(), msg.data);
    }

    busy_ = true;
    query_pub_.publish(msg);

    // 等待响应
    ros::Time start = ros::Time::now();
    while (busy_ && (ros::Time::now() - start).toSec() < TIMEOUT) {
        ros::Duration(0.001).sleep();
        ros::spinOnce();
    }

    return !busy_;
}

void AmrRosBridge::shutdown() {
    if (!running_) return;

    ROS_INFO("Shutting down AMR ROS Bridge...");

    // 发送停止命令
    std::vector<int32_t> stop_cmd = {0, 0, 0, 0};
    sendModbusCommand(0x01, 1, 0, 0, 4960, 4, stop_cmd);

    running_ = false;
    ros::Duration(0.1).sleep();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "amr_ros_bridge");
    ros::NodeHandle nh;

    try {
        AmrRosBridge bridge(nh);
        if (!bridge.init()) {
            return 1;
        }
        bridge.run();
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }

    return 0;
}