#!/usr/bin/python
# -*- coding: utf-8 -*-

#
# 対象機種:AZ
# 処理内容1:運転データNo.0の位置を書き込み

# モジュールのインポート
import rospy
import time
from om_modbus_master.msg import om_query
from om_modbus_master.msg import om_response
from om_modbus_master.msg import om_state

# グローバル変数
gState_driver = 0   # 通信可能フラグ変数(0:通信可能,1:通信中)


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
    time.sleep(0.03)  # ウェイト時間の設定(1 = 1.00s)
    # 通信が終了するまでループ
    while (gState_driver == 1):
        pass


def main():
    """メイン関数

    処理内容1:運転データNo.0の位置を書き込み
    

    """
    rospy.init_node("sample1_1", anonymous=True)
    pub = rospy.Publisher("om_query1", om_query,
                          queue_size=1)  # OMノードに送信するための定義

    rospy.Subscriber("om_state1", om_state, stateCallback)  # レスポンスのコールバック定義
    msg = om_query()  # ノードで定義されたメッセージを使用
    time.sleep(1)

    # 書き込み(運転データNo.0の位置)
    msg.slave_id = 0x01     # 号機選択(Hex): 1号機
    msg.func_code = 1       # ファンクションコード選択: 1(Write)
    msg.write_addr = 6146   # 先頭アドレス選択(Dec): 運転データNo.0の位置
    msg.write_num = 1       # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 1000      # 位置[step]
    pub.publish(msg)        # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                  # 処理待ち

    print("END")            # 終了表示
    rospy.spin()


if __name__ == '__main__':
    main()
