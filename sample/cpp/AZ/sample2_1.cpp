/** @file	sample2_1.cpp

@attention  対象機種:AZ
@details	処理内容1:運転データを書き込み
			処理内容2:運転開始(START信号ON)

/*---------------------------------------------------------------------------*/
/* ファイル取り込み */
#include "ros/ros.h"
#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_state.h"

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


/*---------------------------------------------------------------------------*/
/** メイン関数

@details	処理内容1:運転データを書き込み
			処理内容2:運転開始(START信号ON)
			
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "sample2_1");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<om_modbus_master::om_query>("om_query1",1);	/* OMノードに送信するための定義 */
	ros::Subscriber sub = n.subscribe("om_state1",1, stateCallback);				/* レスポンスのコールバック定義 */
	
	om_modbus_master::om_query msg;		/* ノードで定義されたメッセージを使用 */
	
	ros::Duration(1.0).sleep();

	/* 運転データを書き込み */
	msg.slave_id = 0x01;	/* 号機選択(Hex): 1号機 */
	msg.func_code = 1;		/* ファンクションコード選択: 1(Write) */
	msg.write_addr = 6144;	/* 先頭アドレス選択(Dec): 運転データNo.0の方式(1800h) */
	msg.write_num = 6;		/* 書き込みデータサイズ: 6 (6x32bit) */
	msg.data[0] = 2;        /* 方式: 2:相対位置決め(指令位置基準) */
	msg.data[1] = 5000;     /* 位置: 5000[step] */
	msg.data[2] = 1000;     /* 速度: 1000[Hz] */
	msg.data[3] = 1000;     /* 起動・変速レート: 1.0[kHz/s] */
	msg.data[4] = 1000;     /* 停止レート: 1.0[kHz/s] */
	msg.data[5] = 1000;     /* 運転電流: 100[%] */
	pub.publish(msg);		/* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait();					/* 処理待ち */

    /* 運転開始 */
	msg.slave_id = 0x01;	/* 号機選択(Hex): 1 */
	msg.func_code = 1;		/* ファンクションコード選択: 1(Write) */
	msg.write_addr = 124;	/* 先頭アドレス選択(Dec): ドライバ入力指令 */
	msg.write_num = 1;		/* 書き込みデータサイズ: 1 (32bit) */
	msg.data[0] = 8;		/* 書き込みデータ: (0000 0000 0000 1000) = 8(START信号ON) */
	pub.publish(msg);		/* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait();					/* 処理待ち */
	
    if(gState_mes == MESSAGE_ERROR)
	{
		/* メッセージエラーの発生 */
		stop(msg, pub);		/* 運転停止 */
		return -1;			/* 処理の強制終了 */
	}
	
	if(gState_error == EXCEPTION_RESPONSE)
	{
		/* 例外応答の発生 */
		stop(msg, pub);		/* 運転停止 */
		return -1;			/* 処理の強制終了 */
	}
	
	ros::Duration(5.0).sleep();	/* 5秒待機 */
	stop(msg, pub);				/* 運転停止 */
	printf("END\n");			/* 終了表示 */

	ros::Rate loop_rate(1);

	while(ros::ok())
	{  
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

