/** @file	sample2_2.cpp

@attention  対象機種:AZ
@details	処理内容1:ダイレクトデータ運転を書き込み

/*---------------------------------------------------------------------------*/
/* ファイル取り込み */
#include "ros/ros.h"
#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_state.h"
#include <geometry_msgs/Twist.h>

int x_spd;
int z_ang;
bool set;

void messageCb(const geometry_msgs::Twist& twist){
	x_spd = int(twist.linear.x *1000);
	z_ang = int(twist.angular.z *180000 / M_PI);
	char buf[100];
	sprintf(buf, "x=%d",x_spd);
	set = true;
}

//ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb);


/* グローバル変数 */
int gState_driver = 0;	/* 通信可能フラグ変数(0:通信可能,1:通信中) */
int gState_mes = 0;		/* メッセージ(0:メッセージなし,1:メッセージ到達,2:メッセージエラー) */
int gState_error = 0;	/* エラー(0:エラーなし,1:無応答,2:例外応答) */


const int MESSAGE_ERROR = 2;
const int EXCEPTION_RESPONSE = 2;


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
/** 停止サービス関数

@details	運転入力指令をOFFにする（停止指令を行う）サービス
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void stop(om_modbus_master::om_query msg, ros::Publisher pub)
{
	msg.slave_id = 0x01;	/* 号機選択(Hex): 1号機 */
	msg.func_code = 1;		/* ファンクションコード選択: 1(Write) */
	msg.write_addr = 124;	/* 先頭アドレス選択(Dec): ドライバ入力指令 */
	msg.write_num = 1;		/* 書き込みデータサイズ: 1 (32bit) */
	msg.data[0] = 32;		/* 書き込みデータ: (0000 0000 0010 0000) = 32(STOP信号ON) */
	pub.publish(msg);		/* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait();					/* 処理待ち */
	
	msg.slave_id = 0x01;	/* 号機選択(Hex): 1号機 */
	msg.func_code = 1;		/* ファンクションコード選択: 1(Write) */
	msg.write_addr = 124;	/* 先頭アドレス選択(Dec): ドライバ入力指令 */
	msg.write_num = 1;		/* 書き込みデータサイズ: 1 (32bit) */
	msg.data[0] = 0;		/* 書き込みデータ: 全ビットOFF */
	pub.publish(msg);		/* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait();					/* 処理待ち */
}

void run(om_modbus_master::om_query msg, ros::Publisher pub)
{
	msg.slave_id = 0x01;	/* 号機選択(Hex): 1号機 */
	msg.func_code = 1;		/* ファンクションコード選択: 1(Write) */
	msg.write_addr = 1986;	/* 先頭アドレス選択(Dec): ダイレクト運転データNo.(0058h) */
	msg.write_num = 8;		/* 書き込みデータサイズ: 8 (8x32bit) */
	msg.data[0] = 1;        /* 運転データNo.: 0 */
	msg.data[1] = x_spd;        /* 方式: 2:相対位置決め(指令位置基準) */
	msg.data[2] = z_ang;     /* 位置: 5000[step] */
	pub.publish(msg);		/* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait();					/* 処理待ち */
}


/*---------------------------------------------------------------------------*/
/** メイン関数

@details	処理内容1:ダイレクトデータ運転を書き込み
			
			
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "sample2_2");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<om_modbus_master::om_query>("om_query1",1);	/* OMノードに送信するための定義 */
	//ros::Subscriber sub = n.subscribe("om_state1",1, stateCallback);				/* レスポンスのコールバック定義 */
	
	ros::Subscriber sub = n.subscribe("cmd_vel",1000, &messageCb);
	
	om_modbus_master::om_query msg;		/* ノードで定義されたメッセージを使用 */
	
	//ros::Duration(1.0).sleep();
	
	ros::Rate loop_rate(200.0);
	while(ros::ok())
	{ 
		/* ダイレクトデータ運転 */

		
		if(set){
			run(msg, pub);
		}

 
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

