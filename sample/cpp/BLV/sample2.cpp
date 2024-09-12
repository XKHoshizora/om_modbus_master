/** @file	sample2.cpp

@attention  対象機種:BLV
@details	処理内容1:運転入力方式を3ワイヤ方式に変更
			処理内容2:運転データNo.2の回転速度を書き込み
			処理内容3:運転指令(FWD方向)
			処理内容4:停止指令(減速停止)

/*---------------------------------------------------------------------------*/
/* ファイル取り込み */
#include "ros/ros.h"
#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_state.h"


/* 定数 */
const int MESSAGE_ERROR = 2;
const int EXCEPTION_RESPONSE = 2;

/* グローバル変数 */
int gState_driver = 0;  /* 通信可能フラグ変数(0:通信可能,1:通信中) */
int gState_mes = 0;     /* メッセージ(0:メッセージなし,1:メッセージ到達,2:メッセージエラー) */
int gState_error = 0;   /* エラー(0:エラーなし,1:無応答,2:例外応答) */


/*---------------------------------------------------------------------------*/
/** ステータスコールバック関数

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
	msg.slave_id = 0x01;		/* 号機選択(Hex):  1号機 */
	msg.func_code = 1;			/* ファンクションコード選択: 1(Write) */
	msg.write_addr = 124;		/* 先頭アドレス選択(Dec): 動作コマンド */
	msg.write_num = 1;			/* 書き込みデータサイズ: 1 (32bit) */
	msg.data[0] = 18;			/* 書き込みデータ: ONビット(0000 0000 0001 0010) = 18 */
	pub.publish(msg);			/* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait();						/* 処理待ち */
}


/*---------------------------------------------------------------------------*/
/** 初期化関数

@details	処理内容1:運転入力方式を3ワイヤ方式に変更
			処理内容2:運転データNo.2の回転速度の初期化(0[r/min])
			処理内容3:Configrationの実行
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void init(om_modbus_master::om_query msg, ros::Publisher pub)
{
	/* 運転入力方式の変更(3ワイヤ) */
	msg.slave_id = 0x01;	/* 号機選択(Hex): 1号機 */
	msg.func_code = 1;		/* ファンクションコード選択: 1(Write) */
	msg.write_addr = 4160;	/* 先頭アドレス選択(Dec): 運転入力方式パラメータ */
	msg.write_num = 1;		/* 書き込みデータサイズ: 1(32bit) */
	msg.data[0] = 1;		/* 書き込みデータ: 0(2ワイヤ),1(3ワイヤ) */
	pub.publish(msg);		/* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait();					/* 処理待ち */

	/* 運転データ 回展速度No.2を0[r/min]に初期化 */
	msg.slave_id = 0x01;	/* 号機選択(Hex): 1号機 */
	msg.func_code = 1;		/* ファンクションコード選択: 1(Write) */
	msg.write_addr = 1156;	/* 先頭アドレス選択(Dec): データNo.2 回転速度 */
	msg.write_num = 1;		/* 書き込みデータサイズ: 1 (32bit) */
	msg.data[0] = 0;		/* 書き込みデータ: 0[r/min] */
	pub.publish(msg);		/* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait();					/* 処理待ち */

	/* Configrationの実行 */
	msg.slave_id = 0x01;	/* 号機選択(Hex): 1号機 */
	msg.func_code = 1;		/* ファンクションコード選択: 1(Write) */
	msg.write_addr = 396;	/* 先頭アドレス選択(Dec): Configration実行コマンド */
	msg.write_num = 1;		/* 書き込みデータサイズ: 1 (32bit) */
	msg.data[0] = 1;		/* 書き込みデータ: 1(実行) */
	pub.publish(msg);		/* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait();					/* 処理待ち */
}


/*---------------------------------------------------------------------------*/
/** メイン関数

@details    処理内容1:運転データNo.2の回転速度を書き込み
			処理内容2:運転指令(FWD方向)
			処理内容3:停止指令(減速停止)
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "sample2");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<om_modbus_master::om_query>("om_query1",1);	/* OMにノードに送信するまでの定義 */
	ros::Subscriber sub = n.subscribe("om_state1",1, stateCallback);				/* レスポンスのコールバック定義 */
	om_modbus_master::om_query msg;													/* OMノードで作成したメッセージを使用 */
	ros::Duration(1.0).sleep();

	init(msg, pub);		/* 初期化関数のコール */

	/* 速度の設定 */
	msg.slave_id = 0x01;		/* 号機選択(Hex): 1号機 */
	msg.func_code = 1;			/* ファンクションコード選択: 1(Write) */
	msg.write_addr = 1156;		/* 先頭アドレス選択(Dec): データNo.2 回転速度 */
	msg.write_num = 1;			/* 書き込みデータサイズ: 1 (32bit) */
	msg.data[0] = 1000;			/* 書き込みデータ: 1000[r/min] */
	pub.publish(msg);			/* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait();						/* 処理待ち */

	/* 運転指令(FWD方向)(M1,START/STOP,RUN/BRAKEをON) */
	msg.slave_id = 0x01;		/* 号機選択(Hex): 1号機 */
	msg.func_code = 1;			/* ファンクションコード選択: 1(Write) */
	msg.write_addr = 124;		/* 先頭アドレス選択(Dec): 動作コマンド */
	msg.write_num = 1;			/* 書き込みデータサイズ: 1 (32bit) */
	msg.data[0] = 26;			/* 書き込みデータ: ONビット(0000 0000 0001 1010) = 26 */
	pub.publish(msg);			/* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait();						/* 処理待ち */

	/* メッセージエラーの発生 */
	if(gState_mes == MESSAGE_ERROR)
	{
		stop(msg , pub);
		return -1;	/* 処理の強制終了 */
	}
	/* 例外応答の発生 */
	if(gState_error == EXCEPTION_RESPONSE)
	{
		stop(msg , pub);
		return -1;	/* 処理の強制終了 */
	}

	/* 5秒待機 */
	ros::Duration(5).sleep();

	/* 停止指令 */
    stop(msg, pub);

	printf("END\n");

	ros::Rate loop_rate(1);
	
	while(ros::ok())
	{  
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

