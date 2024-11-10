#include "om_modbus_master/amr_ros_bridge.hpp"

AmrRosBridge::AmrRosBridge(ros::NodeHandle& nh) : nh_(nh) {
    loadParameters();

    // 延迟初始化以等待其他节点就绪
    ros::Duration(1.0).sleep();

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

    // 等待Modbus主节点启动
    ros::Duration(2.0).sleep();

    // 检查Modbus主节点是否在线
    if (query_pub_.getNumSubscribers() == 0) {
        ROS_ERROR("Modbus master node not found. Please check if it's running.");
        return false;
    }

    try {
        // 设置寄存器映射
        om_modbus_master::om_query init_msg;
        init_msg.slave_id = 0x01;
        init_msg.func_code = 1;
        init_msg.write_addr = 4864;
        init_msg.write_num = 32;

        // 清空数据数组
        for(int i = 0; i < 64; i++) {
            init_msg.data[i] = 0;
        }

        // 设置寄存器映射
        init_msg.data[0] = 1069;  // 里程计X
        init_msg.data[1] = 1070;  // 里程计Y
        init_msg.data[2] = 1071;  // 里程计Theta
        init_msg.data[3] = 1038;  // IMU加速度X
        init_msg.data[4] = 1039;  // IMU加速度Y
        init_msg.data[5] = 1040;  // IMU加速度Z
        init_msg.data[6] = 1041;  // IMU角速度X
        init_msg.data[7] = 1042;  // IMU角速度Y
        init_msg.data[8] = 1043;  // IMU角速度Z
        init_msg.data[16] = 993;  // 直接数据运行模式
        init_msg.data[17] = 994;  // 前后平移速度(Vx)
        init_msg.data[18] = 995;  // 角速度(ω)
        init_msg.data[19] = 996;  // 左右平移速度(Vy)

        query_pub_.publish(init_msg);
        ros::Duration(0.5).sleep();

        // 初始化通信测试
        om_modbus_master::om_query test_msg;
        test_msg.slave_id = 0x01;
        test_msg.func_code = 2;
        test_msg.read_addr = 4928;
        test_msg.read_num = 9;
        test_msg.write_addr = 4960;
        test_msg.write_num = 4;

        // 清空数据数组
        for(int i = 0; i < 64; i++) {
            test_msg.data[i] = 0;
        }

        query_pub_.publish(test_msg);
        ros::Duration(0.5).sleep();

        running_ = true;
        ROS_INFO("AMR ROS Bridge initialized successfully");
        return true;
    }
    catch (const std::exception& e) {
        ROS_ERROR("Initialization failed: %s", e.what());
        return false;
    }
}

void AmrRosBridge::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if (!running_) {
        ROS_WARN_THROTTLE(1.0, "AMR ROS Bridge not running");
        return;
    }

    static ros::Time last_cmd_time = ros::Time::now();
    ros::Time current = ros::Time::now();

    // 限制命令发送频率
    if ((current - last_cmd_time).toSec() < 0.05) {
        return;
    }

    // 发送速度指令
    om_modbus_master::om_query cmd_msg;
    cmd_msg.slave_id = 0x01;
    cmd_msg.func_code = 1;  // 写入功能码
    cmd_msg.write_addr = 4960;
    cmd_msg.write_num = 4;

    // 确保数据数组清空
    for(int i = 0; i < 64; i++) {
        cmd_msg.data[i] = 0;
    }

    // 限制速度并转换单位
    cmd_msg.data[0] = 1;  // 使能
    cmd_msg.data[1] = static_cast<int32_t>(
        std::clamp(msg->linear.x, -MAX_LINEAR_VEL, MAX_LINEAR_VEL) * 1000);
    cmd_msg.data[2] = static_cast<int32_t>(
        std::clamp(msg->angular.z, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL) * 1000000);
    cmd_msg.data[3] = static_cast<int32_t>(
        std::clamp(msg->linear.y, -MAX_LINEAR_VEL, MAX_LINEAR_VEL) * 1000);

    if (!busy_) {
        query_pub_.publish(cmd_msg);
        last_cmd_time = current;
        busy_ = true;  // 设置忙碌标志

        // 使用定时器在一定时间后重置忙碌状态
        ros::Timer timer = nh_.createTimer(
            ros::Duration(0.05),  // 50ms超时
            [this](const ros::TimerEvent&) {
                this->busy_ = false;
            },
            true  // 一次性定时器
        );
    }
}

void AmrRosBridge::run() {
    if (!running_) {
        ROS_WARN("AMR ROS Bridge not initialized");
        return;
    }

    ros::Rate rate(update_rate_);
    ros::Time last_query_time = ros::Time::now();

    while (ros::ok() && running_) {
        try {
            ros::Time current_time = ros::Time::now();

            // 定期查询里程计和IMU数据
            if ((current_time - last_query_time).toSec() >= (1.0 / update_rate_)) {
                if (!busy_) {
                    // 读取里程计和IMU数据
                    om_modbus_master::om_query query_msg;
                    query_msg.slave_id = 0x01;
                    query_msg.func_code = 2;  // 读取功能码
                    query_msg.read_addr = 4928;
                    query_msg.read_num = 9;
                    query_msg.write_addr = 4960;
                    query_msg.write_num = 4;

                    // 清空数据数组
                    for(int i = 0; i < 64; i++) {
                        query_msg.data[i] = 0;
                    }
                    query_msg.data[0] = 1;  // 使能位

                    query_pub_.publish(query_msg);
                    last_query_time = current_time;
                    busy_ = true;

                    // 使用定时器在一定时间后重置忙碌状态
                    ros::Timer timer = nh_.createTimer(
                        ros::Duration(0.05),  // 50ms超时
                        [this](const ros::TimerEvent&) {
                            this->busy_ = false;
                        },
                        true  // 一次性定时器
                    );
                }
                publishOdomAndTf(current_time);
                publishImu(current_time);
            }

            ros::spinOnce();
            rate.sleep();
        }
        catch (const std::exception& e) {
            ROS_ERROR_THROTTLE(1.0, "Error in main loop: %s", e.what());
            busy_ = false;  // 发生错误时重置忙碌状态
        }
    }
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

    busy_ = false;  // 收到响应后重置忙碌状态
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

void AmrRosBridge::shutdown() {
    if (!running_) return;

    ROS_INFO("Shutting down AMR ROS Bridge...");

    // 发送停止命令
    om_modbus_master::om_query stop_msg;
    stop_msg.slave_id = 0x01;
    stop_msg.func_code = 1;
    stop_msg.write_addr = 4960;
    stop_msg.write_num = 4;
    stop_msg.data[0] = 0;
    stop_msg.data[1] = 0;
    stop_msg.data[2] = 0;
    stop_msg.data[3] = 0;

    query_pub_.publish(stop_msg);

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