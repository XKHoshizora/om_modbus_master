/** @file	om_changer.cpp

@attention  対象機種 Mobile Robot Controller
@details	
			処理内容1:cmd_vel値を書き込み、読み出したモニタ値を/odom->/base_linkに反映させる
			処理内容2:検出位置の確認(LOOP:5回)

/*---------------------------------------------------------------------------*/
/* ファイル取り込み */
#include "ros/ros.h"
#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_state.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

/* グローバル変数 */
int gState_driver = 0;  /* 通信可能フラグ変数(0:通信可能,1:通信中) */
int gState_mes = 0;     /* メッセージ(0:メッセージなし,1:メッセージ到達,2:メッセージエラー) */
int gState_error = 0;   /* エラー(0:エラーなし,1:無応答,2:例外応答) */

double x_spd = 0.0;
double y_spd = 0.0;
double z_ang = 0.0;
double odm_x = 0.0;
double odm_y = 0.0;
double odm_th = 0.0;



void messageCb(const geometry_msgs::Twist& twist){
	x_spd = int(twist.linear.x *1000.0);
	y_spd = int(twist.linear.y *1000.0);
	z_ang = int(twist.angular.z *1000000.0);
}


/*---------------------------------------------------------------------------*/
/** レスポンスコールバック関数

@details	購読したレスポンスデータをグローバル変数に反映する
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void resCallback(const om_modbus_master::om_response msg)
{
	if(msg.slave_id == 1)// && msg.func_code == 2)
	{
		/* 号機番号が1かつ読み込みのときに値を更新 */
		odm_x = msg.data[0]/1000.0;
		odm_y = msg.data[1]/1000.0;
		odm_th = msg.data[2]/1000000.0;
	}

}



/*---------------------------------------------------------------------------*/
/** ステータスコールバック

@details	購読したステータスデータをグローバル変数に反映する
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void stateCallback(const om_modbus_master::om_state msg)
{
	gState_driver = msg.state_driver;
	gState_mes = msg.state_mes;
	gState_error = msg.state_error;
}


/*---------------------------------------------------------------------------*/
/** 処理待ちサービス関数

@details	規定時間後(30ms)、通信可能になるまでウェイトがかかるサービス
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void wait(void)
{
	ros::Duration(0.03).sleep();
	ros::spinOnce();
	
	/* ドライバの通信が終了するまでループ */
	while(gState_driver == 1)
	{
		ros::spinOnce();
	}
}


/*---------------------------------------------------------------------------*/
/** 初期化関数

@details	処理内容:信号初期化
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void init(om_modbus_master::om_query msg, ros::Publisher pub)
{
	msg.slave_id = 0x00;	/* 号機選択(Hex): ブロードキャスト */
	msg.func_code = 1;		/* ファンクションコード選択: 1(Write) */
	msg.write_addr = 124;	/* 先頭アドレス選択(Dec): ドライバ入力指令 */
	msg.write_num = 1;		/* 書き込みデータサイズ: 1 (32bit) */
	msg.data[0] = 0;		/* 書き込みデータ: 全ビットOFF */
	pub.publish(msg);		/* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait();					/* 処理待ち */
}


/*---------------------------------------------------------------------------*/
/** メイン関数

@details	処理内容1:軸1、軸2にダイレクトデータ運転を書き込み(相対位置決め運転)
			処理内容2:検出位置の読み込み(LOOP:5回)
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "om_ros_node");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<om_modbus_master::om_query>("om_query1",1);	/* OMノードに送信するための定義 */
	ros::Subscriber sub1 = n.subscribe("om_response1",1, resCallback);				/* レスポンスのコールバック定義 */
	ros::Subscriber sub2 = n.subscribe("om_state1",1, stateCallback);				/* レスポンスのコールバック定義 */
	
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",50);	/* Odmに送信するための定義 */
	tf::TransformBroadcaster odom_broadcaster;
	ros::Subscriber sub = n.subscribe("cmd_vel",1, &messageCb);
	
	om_modbus_master::om_query msg;	/* ノードで定義されたメッセージを使用 */
	
	ros::Duration(1.0).sleep();		/* 1秒待機 */
	init(msg, pub);					/* 初期化関数のコール */
	ros::Rate loop_rate(20);			/* 周期の設定 */
	
	ros::Time current_time;
	current_time = ros::Time::now();
	
	printf("START\n");
	
	/* 間接参照にアドレスを設定します。 */
	
		msg.slave_id = 0x01;	    /* 号機選択(Hex): 1 */
		msg.func_code = 1;		    /* ファンクションコード選択: 1(Write) */
		msg.write_addr = 4864;	    /* 先頭アドレス選択(Dec): 間接参照（0）アドレス設定 (1300h)*/
		msg.write_num = 32;		    /* 書き込みデータサイズ: 32(8x32bit) */
		msg.data[0] = 1069;			/* モニタ値 現在位置(検出)X */
		msg.data[1] = 1070;			/* モニタ値 現在位置(検出)Y */
		msg.data[2] = 1071;			/* モニタ値 現在位置(検出)θ */
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
		msg.data[16] = 993;		    /* ダイレクトデータ運転 運転方式 */
		msg.data[17] = 994;		    /* ダイレクトデータ運転 前後並進速度(Vx) */
		msg.data[18] = 995;		    /* ダイレクトデータ運転 角速度(ω) */
		msg.data[19] = 996;			/* ダイレクトデータ運転 左右並進速度(Vy) */
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
		
		pub.publish(msg);		    /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
		wait();                     /* 処理待ち */
		
		
		loop_rate.sleep();

	while(ros::ok())
	{ 

		current_time = ros::Time::now();
		
	/* 間接参照に設定した項目に書き込み、および読み出しを行います。 */
		
		msg.slave_id = 0x01;	    /* 号機選択(Hex): 1 */
		msg.func_code = 2;		    /* ファンクションコード選択: 2(Read & Write) */
		msg.read_addr = 4928;	    /* 先頭アドレス選択(Dec): 間接参照（０）データ (1340h) */
		msg.read_num = 3;		    /* 読み込みデータサイズ: 3 (3x32bit) */
		msg.write_addr = 4960;	    /* 先頭アドレス選択(Dec): 間接参照（１６データ）(1360h) */
		msg.write_num = 4;		    /* 書き込みデータサイズ: 4(4x32bit) */
		msg.data[0] = 1;		    /* 運転方式: vwモード */
		msg.data[1] = x_spd;		/* 前後並進速度(Vx) [mm/s] */
		msg.data[2] = z_ang;		/* 角速度(ω) [rad/s] */
		msg.data[3] = y_spd;		/* 左右並進速度(Vy) [mm/s] */
		pub.publish(msg);			/* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
		

		
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		
		odom_trans.transform.translation.x = odm_x;
		odom_trans.transform.translation.y = odm_y;
		odom_trans.transform.translation.z = 0.0;
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odm_th);
		odom_trans.transform.rotation = odom_quat;
		
		odom_broadcaster.sendTransform(odom_trans);
		
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		
		odom.pose.pose.position.x = odm_x;
		odom.pose.pose.position.y = odm_y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = x_spd/1000.0;//m/s
		odom.twist.twist.linear.y = y_spd/1000.0;//m/s
		odom.twist.twist.angular.z = z_ang/1000000.0;//rad/s
		
		odom_pub.publish(odom);
		
        wait();                     /* 処理待ち */
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

