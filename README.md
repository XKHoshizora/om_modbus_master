# AMR ROS Bridge

## 功能概述

AMR ROS Bridge是一个ROS节点，用于处理移动机器人的里程计和IMU数据，并提供以下功能：

1. 通过Modbus协议与下位机通信
2. 接收和处理cmd_vel速度控制命令
3. 发布里程计数据（包括TF变换）
4. 发布IMU传感器数据
5. 支持可配置的坐标系和通信参数

## 依赖项

- ROS Noetic
- om_modbus_master（Modbus通信功能包）
- tf2_ros
- geometry_msgs
- nav_msgs
- sensor_msgs

## 安装

1. 首先确保您的ROS工作空间已经配置好：

```bash
# 如果还没有创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

2. 克隆并编译功能包：

```bash
cd ~/catkin_ws/src
git clone <repository_url> om_modbus_master
cd ..
catkin_make
```

## 使用方法

### 基本启动

使用launch文件启动节点：

```bash
roslaunch om_modbus_master amr_ros_bridge.launch
```

### 参数配置

launch文件支持以下参数配置：

```bash
# 使用自定义参数启动
roslaunch om_modbus_master amr_ros_bridge.launch \
    com:=/dev/ttyUSB0 \
    baudrate:=115200 \
    updateRate:=20 \
    base_frame:=base_link
```

### 主要参数说明

#### 通信参数
- `com`: Modbus设备串口，默认值：/dev/om_controller
- `baudrate`: 波特率，默认值：230400
- `updateRate`: 更新频率（Hz），默认值：20
- `topicID`: 话题ID，默认值：1

#### 坐标系参数
- `base_frame`: 机器人基座坐标系，默认值：base_footprint
- `odom_frame`: 里程计坐标系，默认值：odom
- `imu_frame`: IMU坐标系，默认值：imu_link

#### 其他参数
- `firstGen`: 第一代设备配置，默认值：""
- `secondGen`: 第二代设备配置，默认值："1,"
- `globalID`: 全局ID，默认值：-1
- `axisNum`: 轴数量，默认值：1

### 发布的话题

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| /odom | nav_msgs/Odometry | 里程计数据 |
| /imu/data | sensor_msgs/Imu | IMU传感器数据 |
| /tf | tf2_msgs/TFMessage | 坐标系转换 |

### 订阅的话题

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| /cmd_vel | geometry_msgs/Twist | 速度控制命令 |
| om_response1 | om_modbus_master/om_response | Modbus响应数据 |
| om_state1 | om_modbus_master/om_state | Modbus状态数据 |

## 坐标系转换

节点会发布以下TF转换：
- odom → base_footprint
- base_footprint → imu_link

## 注意事项

1. 确保串口设备具有正确的权限：
```bash
sudo chmod 666 /dev/om_controller
```

2. 检查波特率设置与设备匹配

3. 更新频率(updateRate)建议不要超过50Hz，以确保稳定性

## 故障排除

### 常见问题

1. 无法打开串口设备
   - 检查设备是否正确连接
   - 验证设备权限
   - 确认设备名称是否正确

2. 数据更新频率不稳定
   - 检查系统负载
   - 确认波特率设置
   - 验证updateRate参数是否合适

3. 坐标转换异常
   - 检查frame_id参数配置
   - 确认TF树是否完整
   - 验证里程计数据的准确性

### 调试方法

1. 使用rostopic工具检查数据：
```bash
# 查看里程计数据
rostopic echo /odom

# 查看IMU数据
rostopic echo /imu/data

# 检查TF转换
rosrun tf tf_echo odom base_footprint
```

2. 使用rqt_graph查看节点关系：
```bash
rosrun rqt_graph rqt_graph
```

## 维护者

- Hoshizora
- [GitHub](https://github.com/XKHoshizora)

## 许可证

[MIT License](LICENSE)

## 更新日志

### v1.0.0 (2024-11-10)
- 初始版本发布
- 实现基本功能
- 支持参数配置