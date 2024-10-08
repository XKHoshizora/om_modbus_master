#include <ros/ros.h>
#include <sensor_msgs/Imu.h>  // ROS IMU 消息类型
#include <om_modbus_master/om_query.h>  // Modbus 查询消息类型
#include <om_modbus_master/om_response.h>  // Modbus 响应消息类型
#include <om_modbus_master/om_state.h>  // Modbus 状态消息类型
#include <mutex>  // 互斥锁保护共享资源

// 全局变量，用于存储从 IMU 获取的加速度和角速度数据
double imu_acc_x = 0.0, imu_acc_y = 0.0, imu_acc_z = 0.0;
double imu_gyro_x = 0.0, imu_gyro_y = 0.0, imu_gyro_z = 0.0;

// 用于保护IMU数据的互斥锁，避免多线程并发修改数据时出错
std::mutex imu_mutex;

// 发布 IMU 数据的 ROS 发布者
ros::Publisher imu_pub;

// 用于追踪通信状态
int gState_error = 0;

// Modbus 响应回调函数：处理 Modbus 响应，提取 IMU 数据
void resCallback(const om_modbus_master::om_response::ConstPtr& res) {
    std::lock_guard<std::mutex> lock(imu_mutex);  // 使用互斥锁保护共享变量
    if (res->data.size() >= 6) {
        // 从 Modbus 响应中提取加速度和角速度数据
        imu_acc_x = res->data[3] * 1000;  // 假设寄存器中的加速度数据需要乘以 0.001
        imu_acc_y = res->data[4] * 1000;
        imu_acc_z = res->data[5] * 1000;
        imu_gyro_x = res->data[6] * 100000;  // 假设寄存器中的角速度数据需要乘以 0.0001
        imu_gyro_y = res->data[7] * 100000;
        imu_gyro_z = res->data[8] * 100000;

        // 创建 ROS IMU 消息并发布
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();  // 使用当前时间戳
        imu_msg.header.frame_id = "imu";  // IMU 的参考坐标系，与 URDF 文件中的 link 名称一致

        // 设置加速度数据
        imu_msg.linear_acceleration.x = imu_acc_x;
        imu_msg.linear_acceleration.y = imu_acc_y;
        imu_msg.linear_acceleration.z = imu_acc_z;

        // 设置角速度数据
        imu_msg.angular_velocity.x = imu_gyro_x;
        imu_msg.angular_velocity.y = imu_gyro_y;
        imu_msg.angular_velocity.z = imu_gyro_z;

        // 发布 IMU 消息到 /imu 话题
        imu_pub.publish(imu_msg);
    } else {
        ROS_WARN("Received incomplete IMU data from Modbus");
    }
}

// Modbus 状态回调函数：用于处理通信状态
void stateCallback(const om_modbus_master::om_state::ConstPtr& msg) {
    gState_error = msg->state_error;  // 记录通信错误状态
    if (gState_error != 0) {
        ROS_ERROR("Modbus communication error: %d", gState_error);
    }
}

// 初始化函数：发送初始化指令到IMU设备（如果有初始化过程）
void init(om_modbus_master::om_query& msg, ros::Publisher& pub) {
    msg.slave_id = 1;  // 设置IMU设备的Modbus从机ID
    msg.func_code = 1;  // 假设功能码1用于初始化
    msg.write_addr = 100;  // 初始化寄存器地址
    msg.write_num = 1;
    msg.data[0] = 1;  // 发送初始化指令
    pub.publish(msg);  // 发送到Modbus设备
    ros::Duration(0.03).sleep();  // 等待初始化完成
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "amr_imu_publisher");
    ros::NodeHandle nh;

    // 从参数服务器获取设备 IP、端口号和发布频率
    std::string device_ip;
    int device_port;
    nh.param<std::string>("device_ip", device_ip, "192.168.0.10");  // 默认IP为192.168.0.10
    nh.param<int>("device_port", device_port, 502);  // 默认端口为502
    double update_rate;
    nh.param("update_rate", update_rate, 10.0);  // 默认更新频率为10Hz

    // 使用这些参数进行后续的设备初始化和通信配置
    ROS_INFO("Connecting to device at %s:%d with update rate: %f Hz", device_ip.c_str(), device_port, update_rate);

    // 定义 IMU 数据的发布者，将 IMU 数据发布到 /imu 话题
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 10);

    // 定义 Modbus 响应和状态的订阅者，订阅来自 Modbus 设备的响应和状态消息
    ros::Subscriber sub_response = nh.subscribe("om_response1", 1000, resCallback);
    ros::Subscriber sub_state = nh.subscribe("om_state1", 1000, stateCallback);

    // 定义 Modbus 查询的发布者，向 IMU 设备发送查询请求
    ros::Publisher pub_query = nh.advertise<om_modbus_master::om_query>("om_query1", 10);

    // 初始化IMU设备（如果需要）
    om_modbus_master::om_query query_msg;
    init(query_msg, pub_query);

    ros::Rate loop_rate(update_rate);

    // 主循环
    while (ros::ok()) {
        // 每隔固定的时间发送Modbus查询请求以获取IMU数据
        query_msg.slave_id = 0x01;  // IMU设备的Modbus从机ID
        query_msg.func_code = 3;  // 读取保持寄存器的功能码
        query_msg.read_addr = 4928;  // 假设IMU数据的寄存器起始地址为100
        query_msg.read_num = 6;  // 读取6个寄存器（3个用于加速度，3个用于角速度）
        query_msg.data[3] = 1038;
        query_msg.data[4] = 1039;
        query_msg.data[5] = 1040;
        query_msg.data[6] = 1041;
        query_msg.data[7] = 1042;
        query_msg.data[8] = 1043;

        pub_query.publish(query_msg);  // 发送查询请求

        // 处理ROS的回调函数并等待下一次循环
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
