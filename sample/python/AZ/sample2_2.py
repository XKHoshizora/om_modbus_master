#!/usr/bin/python
# -*- coding: utf-8 -*-

#
# 対象機種:AZ
# 処理内容1:ダイレクトデータ運転を書き込み
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
    time.sleep(0.03)  # ウェイト時間の設定(1 = 1.00s)
    # 通信が終了するまでループ
    while (gState_driver == 1):
        pass


def stop(msg, pub):
    """停止サービス関数

    運転入力指令をOFFにする（停止指令を行う）サービス

    """
    msg.slave_id = 0x01				# 号機選択(Hex): 1号機
    msg.func_code = 1				# ファンクションコード選択: 1(Write)
    msg.write_addr = 124			# 先頭アドレス選択(Dec): ドライバ入力指令
    msg.write_num = 1				# 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 32				# 書き込みデータ: (0000 0000 0010 0000) = 32(STOP信号ON)
    pub.publish(msg)				# クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                          # 処理待ち

    msg.slave_id = 0x01				# 号機選択(Hex): 1号機
    msg.func_code = 1				# ファンクションコード選択: 1(Write)
    msg.write_addr = 124			# 先頭アドレス選択(Dec): ドライバ入力指令
    msg.write_num = 1				# 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 0				    # 書き込みデータ: 全ビットOFF
    pub.publish(msg)				# クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                          # 処理待ち


def main():
    """メイン関数

    処理内容1:ダイレクトデータ運転を書き込み

    """
    global gState_mes
    global gState_error
    MESSAGE_ERROR = 2
    EXCEPTION_RESPONSE = 2

    rospy.init_node("sample2_2", anonymous=True)
    pub = rospy.Publisher("om_query1", om_query,
                          queue_size=1)  # OMノードに送信するための定義
    rospy.Subscriber("om_state1", om_state,
                     stateCallback)  # レスポンスのコールバック定義
    msg = om_query()  # OMノードで作成したメッセージの保存
    time.sleep(1)

    # ダイレクトデータ運転
    msg.slave_id = 0x01     # 号機選択(Hex): 1号機
    msg.func_code = 1       # ファンクションコード選択: 1(Write)
    msg.write_addr = 88     # 先頭アドレス選択(Dec): ダイレクト運転データNo.(0058h)
    msg.write_num = 8       # 書き込みデータサイズ: 8(8x32bit)
    msg.data[0] = 0         # 運転データNo.: 0
    msg.data[1] = 2         # 方式: 2:相対位置決め(指令位置基準)
    msg.data[2] = 5000      # 位置: 5000[step]
    msg.data[3] = 1000      # 速度: 1000[Hz]
    msg.data[4] = 1000      # 起動・変速レート: 1.0[kHz/s]
    msg.data[5] = 1000      # 停止レート: 1.0[kHz/s]
    msg.data[6] = 1000      # 運転電流: 100[%]
    msg.data[7] = 1         # 反映トリガ: 1(全データ反映)
    pub.publish(msg)        # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                  # 処理待ち

    # メッセージエラーの発生
    if (gState_mes == MESSAGE_ERROR):
        stop(msg, pub)  # 運転停止
        exit()          # 処理の強制終了

    # 例外応答の発生
    if (gState_error == EXCEPTION_RESPONSE):
        stop(msg, pub)  # 運転停止
        exit()          # 処理の強制終了

    time.sleep(5)   # 5秒待機
    stop(msg, pub)  # 運転停止
    print("END")    # 終了表示

    rospy.spin()  # msg取得用関数の実行(必須)


if __name__ == '__main__':
    main()
