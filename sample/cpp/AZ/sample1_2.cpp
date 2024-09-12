/** @file	sample1_2.cpp

@attention  対象機種 AZ
@details	読み込みのサンプル
			処理内容1:運転データNo.0の位置の読み込み

/*---------------------------------------------------------------------------*/
/* ファイル取り込み */
#include "ros/ros.h"
#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_state.h"

/* グローバル変数 */
int gState_driver = 0;  /* 通信可能フラグ変数(0:通信可能,1:通信中) */
int gMotor_pos = 0;


/*---------------------------------------------------------------------------*/
/** レスポンスコールバック関数

@details	購読したレスポンスデータをグローバル変数に反映する
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void resCallback(const om_modbus_master::om_response msg)
{
	if(msg.slave_id == 1 && msg.func_code == 3)
	{
		/* 号機番号が1かつ読み込みのときに値を更新 */
		gMotor_pos = msg.data[0];
	}
}


/*---------------------------------------------------------------------------*/
/** ステータスコールバック

@details	購読したステータスデータをグローバル変数に反映する
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void stateCallback(const om_modbus_master::om_state msg)
{
	gState_driver = msg.state_driver;
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
/** メイン関数

@details	処理内容1:運転データNo.0の位置の読み込み
			
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "sample1_2");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<om_modbus_master::om_query>("om_query1",1);	/* OMノードに送信するための定義 */
	ros::Subscriber sub1 = n.subscribe("om_response1",1, resCallback);				/* レスポンスのコールバック定義 */
	ros::Subscriber sub2 = n.subscribe("om_state1",1, stateCallback);				/* レスポンスのコールバック定義 */
	
	om_modbus_master::om_query msg;		/* ノードで定義されたメッセージを使用 */
	
	ros::Duration(1.0).sleep();
	
	/* 読み込み(運転データNo.0の位置) */
	msg.slave_id = 0x01;	/* 号機選択(Hex): 1号機 */
	msg.func_code = 0;		/* ファンクションコード選択: 0(Read) */
	msg.read_addr = 6146;	/* 先頭アドレス選択(Dec): 運転データNo.0の位置 */
	msg.read_num = 1;		/* 読み込みデータサイズ: 1 (32bit) */
	pub.publish(msg);		/* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait();					/* 処理待ち */

	printf("position = %d[step]\n", gMotor_pos);	/* 読み込んだ値を表示 */
	
	printf("END\n");		/* 終了表示 */
	
    ros::Rate loop_rate(1);

	while(ros::ok())
	{  
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

