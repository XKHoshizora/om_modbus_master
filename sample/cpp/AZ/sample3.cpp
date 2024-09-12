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

/* グローバル変数 */
int gState_driver = 0;  /* 通信可能フラグ変数(0:通信可能,1:通信中) */
int gState_mes = 0;     /* メッセージ(0:メッセージなし,1:メッセージ到達,2:メッセージエラー) */
int gState_error = 0;   /* エラー(0:エラーなし,1:無応答,2:例外応答) */
int gMotor_pos1 = 0;
int gMotor_pos2 = 0;


/*---------------------------------------------------------------------------*/
/** レスポンスコールバック関数

@details	購読したレスポンスデータをグローバル変数に反映する
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void resCallback(const om_modbus_master::om_response msg)
{
	if(msg.slave_id == 1 && msg.func_code == 3)
	{
		/* 号機番号が1かつ読み込みのときに値を更新 */
		gMotor_pos1 = msg.data[0];
	}
    else if(msg.slave_id == 2 && msg.func_code == 3)
    {
        /* 号機番号が2かつ読み込みのときに値を更新 */
		gMotor_pos2 = msg.data[0];
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
	
	om_modbus_master::om_query msg;	/* ノードで定義されたメッセージを使用 */
	
	ros::Duration(1.0).sleep();		/* 1秒待機 */
	init(msg, pub);					/* 初期化関数のコール */
	ros::Rate loop_rate(1);			/* 周期の設定 */
	
	printf("START\n");

	for(int i = 0; i < 5; i++)
	{
		/* 指令位置の書き込み */
		msg.slave_id = 0x01;	    /* 号機選択(Hex): 1 */
		msg.func_code = 1;		    /* ファンクションコード選択: 1(Write) */
		msg.write_addr = 88;	    /* 先頭アドレス選択(Dec): ダイレクト運転データNo.(0058h) */
		msg.write_num = 8;		    /* 書き込みデータサイズ: 8(8x32bit) */
		msg.data[0] = 0;		    /* 運転データNo.: 0 */
		msg.data[1] = 2;		    /* 方式: 2:相対位置決め(指令位置基準) */
		msg.data[2] = 100;		    /* 位置: 100[step] */
		msg.data[3] = 2000;		    /* 速度: 2000[Hz] */
		msg.data[4] = 2000;		    /* 起動・変速レート: 2.0[kHz/s] */
		msg.data[5] = 2000;		    /* 停止レート: 2.0[kHz/s] */
		msg.data[6] = 1000;		    /* 運転電流: 100[%] */
		msg.data[7] = 1;		    /* 反映トリガ: 1(全データ反映) */
		pub.publish(msg);		    /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
        wait();                     /* 処理待ち */

        msg.slave_id = 0x02;	    /* 号機選択(Hex): 2 */
		msg.func_code = 1;		    /* ファンクションコード選択: 1(Write) */
		msg.write_addr = 88;	    /* 先頭アドレス選択(Dec): ダイレクト運転データNo.(0058h) */
		msg.write_num = 8;		    /* 書き込みデータサイズ: 8(8x32bit) */
		msg.data[0] = 0;		    /* 運転データNo.: 0 */
		msg.data[1] = 2;		    /* 方式: 2:相対位置決め(指令位置基準) */
		msg.data[2] = 200;		    /* 位置: 200[step] */
		msg.data[3] = 2000;		    /* 速度: 2000[Hz] */
		msg.data[4] = 2000;		    /* 起動・変速レート: 2.0[kHz/s] */
		msg.data[5] = 2000;		    /* 停止レート: 2.0[kHz/s] */
		msg.data[6] = 1000;		    /* 運転電流: 100[%] */
		msg.data[7] = 1;		    /* 反映トリガ: 1(全データ反映) */
		pub.publish(msg);		    /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
        wait();                     /* 処理待ち */

		ros::Duration(0.5).sleep();

		msg.slave_id = 0x01;	    /* 号機選択(Hex): 1 */
		msg.func_code = 0;		    /* ファンクションコード選択: 0(Read) */
		msg.read_addr = 204;	    /* 先頭アドレス選択(Dec): 検出位置(step) */
		msg.read_num = 1;		    /* 読み込みデータサイズ: 1 (32bit) */
		pub.publish(msg);		    /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
		wait();					    /* 処理待ち */

        msg.slave_id = 0x02;	    /* 号機選択(Hex): 2 */
		msg.func_code = 0;		    /* ファンクションコード選択: 0(Read) */
		msg.read_addr = 204;	    /* 先頭アドレス選択(Dec): 検出位置(step) */
		msg.read_num = 1;		    /* 読み込みデータサイズ: 1 (32bit) */
		pub.publish(msg);		    /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
		wait();					    /* 処理待ち */
		loop_rate.sleep();		    /* 1秒周期になるまで待機 */
		
		printf("FeedbackPosition1 = %d[step]\n", gMotor_pos1);	/* 軸1の検出位置の表示 */
        printf("FeedbackPosition2 = %d[step]\n", gMotor_pos2);	/* 軸2の検出位置の表示 */
	}
	
    stop(msg, pub); /* 運転停止 */
	printf("END\n");/* 終了表示 */

	while(ros::ok())
	{  
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

