#!/usr/bin/python
# -*- coding: utf-8 -*-

#
# 対象機種:BLV
# 処理内容1:運転入力方式を3ワイヤ方式に変更
# 処理内容2:運転データNo.2の回転速度を書き込み
# 処理内容3:運転指令(FWD方向)
# 処理内容4:停止指令(減速停止)
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
    msg.write_addr = 124			# 先頭アドレス選択(Dec): 動作コマンド
    msg.write_num = 1				# 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 18                # 書き込みデータ: ONビット(0000 0000 0001 0010) = 18
    pub.publish(msg)				# クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()							# 処理待ち


def init(msg, pub):
    """初期化関数

    処理内容1:運転入力方式を3ワイヤ方式に変更
    処理内容2:運転データNo.2の回転速度の初期化(0[r/min])
    処理内容3:Configrationの実行

    """
    # 処理1 3ワイヤに変更
    msg.slave_id = 0x01     # 号機選択(Hex): 1号機
    msg.func_code = 1       # ファンクションコード選択: 1(Write)
    msg.write_addr = 4160   # 先頭アドレス選択(Dec): 運転入力方式パラメータ
    msg.write_num = 1       # 書き込みデータサイズ: 1(32bit)
    msg.data[0] = 1         # 書き込みデータ: 0(2ワイヤ),1(3ワイヤ)
    pub.publish(msg)        # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                  # 処理待ち

    # 処理2 運転データNo.2の回転速度の初期化(0[r/min])
    msg.slave_id = 0x01     # 号機選択(Hex): 1号機
    msg.func_code = 1       # ファンクションコード選択: 1(Write)
    msg.write_addr = 1156   # 先頭アドレス選択(Dec): データNo.2 回転速度
    msg.write_num = 1       # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 0         # 書き込みデータ: 0[r/min]
    pub.publish(msg)        # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                  # 処理待ち

    # 処理3 Configrationの実行
    msg.slave_id = 0x01   # 号機選択(Hex): 1号機
    msg.func_code = 1     # ファンクションコード選択: 1(Write)
    msg.write_addr = 396  # 先頭アドレス選択(Dec): Configration実行コマンド
    msg.write_num = 1     # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 1       # 書き込みデータ: 1(実行)
    pub.publish(msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                # 処理待ち


def main():
    """メイン関数

    処理内容1:運転データNo.2の回転速度を書き込み
    処理内容2:運転指令(FWD方向)
    処理内容3:停止指令(減速停止)

    """
    global gState_mes
    global gState_error
    MESSAGE_ERROR = 2
    EXCEPTION_RESPONSE = 2

    rospy.init_node("sample2", anonymous=True)
    pub = rospy.Publisher("om_query1", om_query,
                          queue_size=1)  # OMにノードに送信するまでの定義
    rospy.Subscriber("om_state1", om_state,
                     stateCallback)  # レスポンスのコールバック定義
    msg = om_query()  # OMノードで作成したメッセージを使用
    time.sleep(1)

    init(msg, pub)  # 初期化関数のコール

    # 速度設定               # 運転データNo.2の回転速度を書き込み
    msg.slave_id = 0x01     # 号機選択(Hex): 1号機
    msg.func_code = 1       # ファンクションコード選択: 1(Write)
    msg.write_addr = 1156   # 先頭アドレス選択(Dec): データNo.2 回転速度
    msg.write_num = 1       # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 1000      # 書き込みデータ: 1000[r/min]
    pub.publish(msg)        # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                  # 処理待ち

    # 運転指令(FWD方向)(M1,START/STOP,RUN/BRAKEをON)
    msg.slave_id = 0x01   # 号機選択(Hex): 1号機
    msg.func_code = 1     # ファンクションコード選択: 1(Write)
    msg.write_addr = 124  # 先頭アドレス選択(Dec): 動作コマンド
    msg.write_num = 1     # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 26      # 書き込みデータ: ONビット(0000 0000 0001 1010) = 26
    pub.publish(msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                # 処理待ち

    # メッセージエラーの発生
    if (gState_mes == MESSAGE_ERROR):
        stop(msg, pub)
        exit()  # 処理の強制終了

    # 例外応答の発生
    if (gState_error == EXCEPTION_RESPONSE):
        stop(msg, pub)
        exit()  # 処理の強制終了

    # 5秒待機
    time.sleep(5)

    # 停止指令
    stop(msg, pub)

    print("END")  # 終了表示
    rospy.spin()  # msg取得用関数の実行(必須)


if __name__ == '__main__':
    main()
