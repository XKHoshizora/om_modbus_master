/** @file    om_changer.cpp

@attention  対象機種 Mobile Robot Controller
@details
                        処理内容1:cmd_vel値を書き込み、読み出したモニタ値を/odom->/base_footprint->/base_linkに反映させる
                        処理内容2:検出位置の確認(LOOP:5回)
*/

/*---------------------------------------------------------------------------*/
/* ファイル取り込み */
#include "ros/ros.h"
#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_state.h"

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ADDED: For thread safety
#include <mutex>

/* グローバル変数 */
int gState_driver = 0; /* 通信可能フラグ変数(0:通信可能,1:通信中) */
int gState_mes = 0;    /* メッセージ(0:メッセージなし,1:メッセージ到達,2:メッセージエラー) */
int gState_error = 0;  /* エラー(0:エラーなし,1:無応答,2:例外応答) */

// MODIFIED: Added mutex for thread safety
std::mutex odom_mutex;
double x_spd = 0.0; /* 前後並進速度(Vx) [mm/s] */
double y_spd = 0.0; /* 角速度(ω) [rad/s] */
double z_ang = 0.0; /* 左右並進速度(Vy) [mm/s] */
double odm_x = 0.0;
double odm_y = 0.0;
double odm_th = 0.0;

// MODIFIED: Removed hard-coded value, will be set from parameter server
double ROBOT_BASE_HEIGHT;

void messageCb(const geometry_msgs::Twist& twist) {
    std::lock_guard<std::mutex> lock(odom_mutex);
    x_spd = int(twist.linear.x * 1000.0);
    y_spd = int(twist.linear.y * 1000.0);
    z_ang = int(twist.angular.z * 1000000.0);
}

/*---------------------------------------------------------------------------*/
/** レスポンスコールバック関数

@details    購読したレスポンスデータをグローバル変数に反映する
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void resCallback(const om_modbus_master::om_response msg) {
    if (msg.slave_id == 1)  // && msg.func_code == 2)
    {
        /* 号機番号が1かつ読み込みのときに値を更新 */
        std::lock_guard<std::mutex> lock(odom_mutex);
        odm_x = msg.data[0] / 1000.0;
        odm_y = msg.data[1] / 1000.0;
        odm_th = msg.data[2] / 1000000.0;
    }
}

/*---------------------------------------------------------------------------*/
/** ステータスコールバック

@details    購読したステータスデータをグローバル変数に反映する
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void stateCallback(const om_modbus_master::om_state msg) {
    gState_driver = msg.state_driver;
    gState_mes = msg.state_mes;
    gState_error = msg.state_error;
}

/*---------------------------------------------------------------------------*/
/** 処理待ちサービス関数

@details    規定時間後(30ms)、通信可能になるまでウェイトがかかるサービス
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void wait(void) {
    ros::Duration(0.03).sleep();
    ros::spinOnce();

    /* ドライバの通信が終了するまでループ */
    while (gState_driver == 1) {
        ros::spinOnce();
    }
}

/*---------------------------------------------------------------------------*/
/** 初期化関数

@details    処理内容:信号初期化
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void init(om_modbus_master::om_query msg, ros::Publisher pub) {
    msg.slave_id = 0x00; /* 号機選択(Hex): ブロードキャスト */
    msg.func_code = 1;   /* ファンクションコード選択: 1(Write) */
    msg.write_addr = 124; /* 先頭アドレス選択(Dec): ドライバ入力指令 */
    msg.write_num = 1; /* 書き込みデータサイズ: 1 (32bit) */
    msg.data[0] = 0;   /* 書き込みデータ: 全ビットOFF */
    pub.publish(msg); /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
    wait();   /* 処理待ち */
}

/*---------------------------------------------------------------------------*/
/** メイン関数

@details
処理内容1:軸1、軸2にダイレクトデータ運転を書き込み(相対位置決め運転)
                        処理内容2:検出位置の読み込み(LOOP:5回)
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int main(int argc, char** argv) {
    ros::init(argc, argv, "om_ros_node");
    ros::NodeHandle n;

    // ADDED: Get parameters from parameter server
    double update_rate;
    n.param("update_rate", update_rate, 10.0); // 里程计数据的更新和发布频率,默认为10Hz。
    n.param("robot_base_height", ROBOT_BASE_HEIGHT, 0.2); // 表示 "base_link" 相对于 "base_footprint" 在垂直方向上的偏移,如果没有设置则使用默认值0.1。

    ros::Publisher pub = n.advertise<om_modbus_master::om_query>("om_query1", 1);
    ros::Subscriber sub1 = n.subscribe("om_response1", 1, resCallback);
    ros::Subscriber sub2 = n.subscribe("om_state1", 1, stateCallback);

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf2_ros::TransformBroadcaster odom_broadcaster;
    tf2_ros::StaticTransformBroadcaster static_broadcaster;

    ros::Subscriber sub = n.subscribe("cmd_vel", 1, &messageCb);

    om_modbus_master::om_query msg;

    // Broadcasting the static transform between base_footprint and base_link
    geometry_msgs::TransformStamped base_to_link;
    base_to_link.header.stamp = ros::Time::now();
    base_to_link.header.frame_id = "base_footprint";
    base_to_link.child_frame_id = "base_link";
    base_to_link.transform.translation.x = 0.0;
    base_to_link.transform.translation.y = 0.0;
    base_to_link.transform.translation.z = ROBOT_BASE_HEIGHT;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);  // 无旋转
    base_to_link.transform.rotation = tf2::toMsg(q);

    static_broadcaster.sendTransform(base_to_link);

    ros::Duration(1.0).sleep();
    init(msg, pub);
    ros::Rate loop_rate(update_rate);

    printf("START\n");

    /* 間接参照にアドレスを設定します。 */
    msg.slave_id = 0x01; /* 号機選択(Hex): 1 */
    msg.func_code = 1;   /* ファンクションコード選択: 1(Write) */
    msg.write_addr = 4864; /* 先頭アドレス選択(Dec): 間接参照（0）アドレス設定 (1300h)*/
    msg.write_num = 32; /* 書き込みデータサイズ: 32(8x32bit) */
    // msg.data[0] = 1069; /* モニタ値 現在位置(検出)X */
    // msg.data[1] = 1070; /* モニタ値 現在位置(検出)Y */
    // msg.data[2] = 1071; /* モニタ値 現在位置(検出)θ */
    msg.data[0] = 1032; /* モニタ値 ジャイロオドメトリX */
    msg.data[1] = 1033; /* モニタ値 ジャイロオドメトリY */
    msg.data[2] = 1037; /* モニタ値 ジャイロオドメトリYAW */
    msg.data[3] = 0;
    msg.data[4] = 0;
    msg.data[5] = 0;
    msg.data[6] = 0;
    msg.data[7] = 0;
    msg.data[8] = 0;
    msg.data[9] = 0;
    msg.data[10] = 0;
    msg.data[11] = 0;
    msg.data[12] = 0;
    msg.data[13] = 0;
    msg.data[14] = 0;
    msg.data[15] = 0;
    msg.data[16] = 993; /* ダイレクトデータ運転 運転方式 */
    msg.data[17] = 994; /* ダイレクトデータ運転 前後並進速度(Vx) */
    msg.data[18] = 995; /* ダイレクトデータ運転 角速度(ω) */
    msg.data[19] = 996; /* ダイレクトデータ運転 左右並進速度(Vy) */
    msg.data[20] = 0;
    msg.data[21] = 0;
    msg.data[22] = 0;
    msg.data[23] = 0;
    msg.data[24] = 0;
    msg.data[25] = 0;
    msg.data[26] = 0;
    msg.data[27] = 0;
    msg.data[28] = 0;
    msg.data[29] = 0;
    msg.data[30] = 0;
    msg.data[31] = 0;

    pub.publish(msg);
    wait();

    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();

        /* 間接参照に設定した項目に書き込み、および読み出しを行います。 */
        msg.slave_id = 0x01;
        msg.func_code = 2;
        msg.read_addr = 4928;
        msg.read_num = 3;
        msg.write_addr = 4960;
        msg.write_num = 4;
        {
            std::lock_guard<std::mutex> lock(odom_mutex);
            msg.data[0] = 1;
            msg.data[1] = x_spd;
            msg.data[2] = z_ang;
            msg.data[3] = y_spd;
        }
        pub.publish(msg);

        tf2::Transform odom_tf;
        tf2::Quaternion q;
        {
            std::lock_guard<std::mutex> lock(odom_mutex);
            odom_tf.setOrigin(tf2::Vector3(odm_x, odm_y, 0.0));
            q.setRPY(0, 0, odm_th);
            odom_tf.setRotation(q);
        }

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";
        
        // 正确转换 tf2::Transform 到 geometry_msgs::Transform
        tf2::convert(odom_tf, odom_trans.transform);

        odom_broadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        // 正确设置 geometry_msgs::Pose
        odom.pose.pose.position.x = odom_tf.getOrigin().x();
        odom.pose.pose.position.y = odom_tf.getOrigin().y();
        odom.pose.pose.position.z = odom_tf.getOrigin().z();
        tf2::convert(odom_tf.getRotation(), odom.pose.pose.orientation);

        {
            std::lock_guard<std::mutex> lock(odom_mutex);
            odom.twist.twist.linear.x = x_spd / 1000.0;
            odom.twist.twist.linear.y = y_spd / 1000.0;
            odom.twist.twist.angular.z = z_ang / 1000000.0;
        }

        odom_pub.publish(odom);

        ROS_INFO_THROTTLE(1, "Current position: x=%f, y=%f, theta=%f", odm_x, odm_y, odm_th);

        wait();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}