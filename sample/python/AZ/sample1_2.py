#!/usr/bin/python
# -*- coding: utf-8 -*-

#
# 対象機種:AZ
# 処理内容1:運転データNo.0の位置の読み込み
#

# モジュールのインポート
import rospy
import time
from om_modbus_master.msg import om_query
from om_modbus_master.msg import om_response
from om_modbus_master.msg import om_state

# グローバル変数
gState_driver = 0   # 通信可能フラグ変数(0:通信可能,1:通信中)
gMotor_pos = 0


def resCallback(res):
    """レスポンスコールバック関数

    購読したレスポンスデータをグローバル変数に反映する

    """
    global gMotor_pos
    if (res.slave_id == 1 and res.func_code == 3):
        # 号機番号が1かつ読み込みのときに値を更新
        gMotor_pos = res.data[0]


def stateCallback(res):
    """ステータスコールバック関数

    購読したステータスデータをグローバル変数に反映する

    """
    global gState_driver
    gState_driver = res.state_driver


def wait():
    """処理待ちサービス関数

    規定時間後(30ms)、通信可能になるまでウェイトがかかるサービス

    """
    global gState_driver
    time.sleep(0.03) # ウェイト時間の設定(1 = 1.00s)
    # 通信が終了するまでループ
    while (gState_driver == 1):
        pass


def main():
    """メイン関数

    処理内容1:運転データNo.0の位置の読み込み

    """
    global gMotor_pos
    rospy.init_node("sample1_2", anonymous=True)
    pub = rospy.Publisher("om_query1", om_query,
                          queue_size=1)  # OMノードに送信するための定義

    rospy.Subscriber("om_response1", om_response,
                     resCallback)  # レスポンスのコールバック定義
    rospy.Subscriber("om_state1", om_state, stateCallback)  # レスポンスのコールバック定義
    msg = om_query()  # ノードで定義されたメッセージを使用
    time.sleep(1)

    # 読み込み(運転データNo.0の位置)
    msg.slave_id = 0x01     # 号機選択(Hex): 1号機
    msg.func_code = 0       # ファンクションコード選択: 0(Read)
    msg.read_addr = 6146    # 先頭アドレス選択(Dec): 運転データNo.0の位置
    msg.read_num = 1        # 読み込みデータサイズ: 1 (32bit)
    pub.publish(msg)        # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                  # 処理待ち

    # 読み込んだ値を表示
    print("position = {0:}[step]".format(gMotor_pos))

    print("END")    # 終了表示
    rospy.spin()


if __name__ == '__main__':
    main()
