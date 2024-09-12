/** @file	sample3.cpp

@attention  対象機種 AZ(2軸)
@details	
			処理内容1:軸1、軸2にダイレクトデータ運転を書き込み(相対位置決め運転)(LOOP:5回)
			処理内容2:検出位置の確認(LOOP:5回)

/*---------------------------------------------------------------------------*/
/* ファイル取り込み */
#include "ros/ros.h"
#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_state.h"
#include <geometry_msgs/Twist.h>

/* グローバル変数 */
int gState_driver = 0;  /* 通信可能フラグ変数(0:通信可能,1:通信中) */
int gState_mes = 0;     /* メッセージ(0:メッセージなし,1:メッセージ到達,2:メッセージエラー) */
int gState_error = 0;   /* エラー(0:エラーなし,1:無応答,2:例外応答) */

int x_spd;
int z_ang;
bool set;
int odm_x;
int odm_y;
int odm_th;



void messageCb(const geometry_msgs::Twist& twist){
	x_spd = int(twist.linear.x *1000);
	z_ang = int(twist.angular.z *180000 / M_PI);
}


/*---------------------------------------------------------------------------*/
/** レスポンスコールバック関数

@details	購読したレスポンスデータをグローバル変数に反映する
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void resCallback(const om_modbus_master::om_response msg)
{
	if(msg.slave_id == 1 && msg.func_code == 3)
	{
		/* 号機番号が1かつ読み込みのときに値を更新 */
		odm_x = msg.data[0]/1000.0;
		odm_y = msg.data[1]/1000.0;
		odm_th = msg.data[2]/180000.0*M_PI;
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
/** 停止サービス関数

@details	運転入力指令をOFFにする（停止指令を行う）サービス
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void stop(om_modbus_master::om_query msg, ros::Publisher pub)
{
	msg.slave_id = 0x00;	/* 号機選択(Hex): ブロードキャスト */
	msg.func_code = 1;		/* ファンクションコード選択: 1(Write) */
	msg.write_addr = 124;	/* 先頭アドレス選択(Dec): ドライバ入力指令 */
	msg.write_num = 1;		/* 書き込みデータサイズ: 1 (32bit) */
	msg.data[0] = 32;		/* 書き込みデータ: (0000 0000 0010 0000) = 32(STOP信号ON) */
	pub.publish(msg);		/* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait();					/* 処理待ち */
	
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
	ros::init(argc, argv, "sample3");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<om_modbus_master::om_query>("om_query1",1);	/* OMノードに送信するための定義 */
	ros::Subscriber sub1 = n.subscribe("om_response1",1, resCallback);				/* レスポンスのコールバック定義 */
	ros::Subscriber sub2 = n.subscribe("om_state1",1, stateCallback);				/* レスポンスのコールバック定義 */
	
	ros::Subscriber sub = n.subscribe("cmd_vel",1, &messageCb);
	
	om_modbus_master::om_query msg;	/* ノードで定義されたメッセージを使用 */
	
	ros::Duration(1.0).sleep();		/* 1秒待機 */
	init(msg, pub);					/* 初期化関数のコール */
	ros::Rate loop_rate(20);			/* 周期の設定 */
	
	printf("START\n");

	while(ros::ok())
	{ 
		/* 指令位置の書き込み */
		msg.slave_id = 0x01;	    /* 号機選択(Hex): 1 */
		msg.func_code = 1;		    /* ファンクションコード選択: 1(Write) */
		msg.write_addr = 1986;	    /* 先頭アドレス選択(Dec): ダイレクト運転データNo.(0058h) */
		msg.write_num = 8;		    /* 書き込みデータサイズ: 8(8x32bit) */
		msg.data[0] = 1;		    /* 運転データNo.: 0 */
		msg.data[1] = x_spd;		    /* 方式: 2:相対位置決め(指令位置基準) */
		msg.data[2] = z_ang;		    /* 位置: 100[step] */
		pub.publish(msg);		    /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
        wait();                     /* 処理待ち */

		ros::Duration(0.02).sleep();//ms 50ms 20Hz

		msg.slave_id = 0x01;	    /* 号機選択(Hex): 1 */
		msg.func_code = 0;		    /* ファンクションコード選択: 0(Read) */
		msg.read_addr = 2138;	    /* 先頭アドレス選択(Dec): 検出位置(step) */
		msg.read_num = 3;		    /* 読み込みデータサイズ: 1 (32bit) */

		pub.publish(msg);		    /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
		wait();					    /* 処理待ち */

		
//		printf("FeedbackPosition1 = %d[step]\n", odm_x);	/* 軸1の検出位置の表示 */


	 
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

