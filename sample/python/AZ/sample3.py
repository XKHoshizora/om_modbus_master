#!/usr/bin/python
# -*- coding: utf-8 -*-

#
# 対象機種:AZ(2軸)
# 処理内容1:軸1、軸2にダイレクトデータ運転を書き込み(相対位置決め運転)(LOOP:5回)
# 処理内容2:検出位置の確認(LOOP:5回)
#

# モジュールのインポート
import rospy
import time
from om_modbus_master.msg import om_query
from om_modbus_master.msg import om_response
from om_modbus_master.msg import om_state

# グローバル変数
gState_driver = 0   # 通信可能フラグ変数(0:通信可能,1:通信中)
gState_mes = 0      # メッセージ(0:メッセージなし,1:メッセージ到達,2:メッセージエラー)
gState_error = 0    # エラー(0:エラーなし,1:無応答,2:例外応答)
gMotor_pos1 = 0
gMotor_pos2 = 0

def resCallback(res):
    """レスポンスコールバック関数

    購読したレスポンスデータをグローバル変数に反映する

    """
    global gMotor_pos1
    global gMotor_pos2
    if (res.slave_id == 1 and res.func_code == 3):
        # 号機番号が1かつ読み込みのときに値を更新
        gMotor_pos1 = res.data[0]
    elif (res.slave_id == 2 and res.func_code == 3):
        # 号機番号が2かつ読み込みのときに値を更新
        gMotor_pos2 = res.data[0]


def stateCallback(res):
    """ステータスコールバック関数

    購読したステータスデータをグローバル変数に反映する

    """
    global gState_driver
    global gState_mes
    global gState_error
    gState_driver = res.state_driver
    gState_mes = res.state_mes
    gState_error = res.state_error


def wait():
    """処理待ちサービス関数

    規定時間後(30ms)、通信可能になるまでウェイトがかかるサービス

    """
    global gState_driver
    time.sleep(0.03)
    # ドライバの通信が終了するまでループ
    while (gState_driver == 1):
        pass


def init(msg, pub):
    """初期化関数

    処理内容:信号初期化

    """
    # 信号初期化
    msg.slave_id = 0x00				# 号機選択(Hex): ブロードキャスト
    msg.func_code = 1				# ファンクションコード選択: 1(Write)
    msg.write_addr = 124			# 先頭アドレス選択(Dec): ドライバ入力指令
    msg.write_num = 1				# 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 0				    # 書き込みデータ: 全ビットOFF
    pub.publish(msg)				# クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                          # 処理待ち


def stop(msg, pub):
    """停止サービス関数

    運転入力指令をOFFにする（停止指令を行う）サービス

    """
    msg.slave_id = 0x00				# 号機選択(Hex): ブロードキャスト
    msg.func_code = 1				# ファンクションコード選択: 1(Write)
    msg.write_addr = 124			# 先頭アドレス選択(Dec): ドライバ入力指令
    msg.write_num = 1				# 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 32				# 書き込みデータ: (0000 0000 0010 0000) = 32(STOP信号ON)
    pub.publish(msg)				# クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                          # 処理待ち

    msg.slave_id = 0x00				# 号機選択(Hex): ブロードキャスト
    msg.func_code = 1				# ファンクションコード選択: 1(Write)
    msg.write_addr = 124			# 先頭アドレス選択(Dec): ドライバ入力指令
    msg.write_num = 1				# 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 0				    # 書き込みデータ: 全ビットOFF
    pub.publish(msg)				# クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                          # 処理待ち


def main():
    """メイン関数

    処理内容1:軸1、軸2にダイレクトデータ運転を書き込み(相対位置決め運転)
    処理内容2:検出位置の読み込み(LOOP:5回)

    """
    global gMotor_pos1
    global gMotor_pos2
    rospy.init_node("sample3", anonymous=True)
    pub = rospy.Publisher("om_query1", om_query,
                          queue_size=1)  # OMノードに送信するための定義

    rospy.Subscriber("om_state1", om_state,
                     stateCallback)  # レスポンスのコールバック定義
    rospy.Subscriber("om_response1", om_response,
                     resCallback)  # レスポンスのコールバック定義

    msg = om_query()  # OMノードで作成したメッセージの保存
    time.sleep(1)

    init(msg, pub)    # 初期化関数のコール
    r = rospy.Rate(1) # ループ周期1秒

    print("START")

    for i in range(5):  # LOOP処理(5回)
        # 指令位置の書き込み
        msg.slave_id = 0x01         # 号機選択(Hex): 1
        msg.func_code = 1           # ファンクションコード選択: 1(Write)
        msg.write_addr = 88         # 先頭アドレス選択(Dec): ダイレクト運転データNo.(0058h)
        msg.write_num = 8           # 書き込みデータサイズ: 8(8x32bit)
        msg.data[0] = 0             # 運転データNo.: 0
        msg.data[1] = 2             # 方式: 2:相対位置決め(指令位置基準)
        msg.data[2] = 100           # 位置: 100[step]
        msg.data[3] = 2000          # 速度: 2000[Hz]
        msg.data[4] = 2000          # 起動・変速レート: 2.0[kHz/s]
        msg.data[5] = 2000          # 停止レート: 2.0[kHz/s]
        msg.data[6] = 1000          # 運転電流: 100[%]
        msg.data[7] = 1             # 反映トリガ: 1(全データ反映)
        pub.publish(msg)            # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        wait()

        msg.slave_id = 0x02         # 号機選択(Hex): 2
        msg.func_code = 1           # ファンクションコード選択: 1(Write)
        msg.write_addr = 88         # 先頭アドレス選択(Dec): ダイレクト運転データNo.(0058h)
        msg.write_num = 8           # 書き込みデータサイズ: 8(8x32bit)
        msg.data[0] = 0             # 運転データNo.: 0
        msg.data[1] = 2             # 方式: 2:相対位置決め(指令位置基準)
        msg.data[2] = 200           # 位置: 200[step]
        msg.data[3] = 2000          # 速度: 2000[Hz]
        msg.data[4] = 2000          # 起動・変速レート: 2.0[kHz/s]
        msg.data[5] = 2000          # 停止レート: 2.0[kHz/s]
        msg.data[6] = 1000          # 運転電流: 100[%]
        msg.data[7] = 1             # 反映トリガ: 1(全データ反映)
        pub.publish(msg)            # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        wait()

        time.sleep(0.5)

        msg.slave_id = 0x01         # 号機選択(Hex): 1
        msg.func_code = 0           # ファンクションコード選択: 0(Read)
        msg.read_addr = 204         # 先頭アドレス選択(Dec): 検出位置(step)
        msg.read_num = 1            # 読み込みデータサイズ: 1 (32bit)
        pub.publish(msg)            # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        wait()
		
        msg.slave_id = 0x02         # 号機選択(Hex): 2
        msg.func_code = 0           # ファンクションコード選択: 0(Read)
        msg.read_addr = 204         # 先頭アドレス選択(Dec): 検出位置(step)
        msg.read_num = 1            # 読み込みデータサイズ: 1 (32bit)
        pub.publish(msg)            # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        wait()
        r.sleep()

        print("FeedbackPosition1 = {0:}[step]".format(gMotor_pos1))   # 軸1の検出位置の表示
        print("FeedbackPosition2 = {0:}[step]".format(gMotor_pos2))   # 軸2の検出位置の表示
        

    stop(msg, pub)  # 運転停止    
    print("END")    # 終了表示
    rospy.spin()    # msg取得用関数の実行(必須)


if __name__ == '__main__':
    main()
