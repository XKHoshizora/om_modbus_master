#!/usr/bin/python
# -*- coding: utf-8 -*-

#
# 対象機種:BLV(2軸)
# 処理内容1:運転データNo.2の回転速度を書き込み
# 処理内容2:運転指令(軸1 FWD方向, 軸2 REV方向)
# 処理内容3:運転データNo.2の回転速度の変更(LOOP:5回)
# 処理内容4:検出速度の確認(LOOP:5回)
# 処理内容5:停止指令(軸1 減速停止、軸2 瞬時停止)
#
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
gMotor_spd1 = 0
gMotor_spd2 = 0

def resCallback(res):
    """レスポンスコールバック関数

    購読したレスポンスデータをグローバル変数に反映する

    """
    global gMotor_spd1
    global gMotor_spd2
    if (res.slave_id == 1 and res.func_code == 3):
        # 号機番号が1かつ読み込みのときに値を更新
        gMotor_spd1 = res.data[0]
    elif (res.slave_id == 2 and res.func_code == 3):
        # 号機番号が2かつ読み込みのときに値を更新
        gMotor_spd2 = res.data[0]


def stateCallback(res):
    """ステータスコールバック関数

    購読したレスポンスデータをグローバル変数に反映する

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

    処理内容1:運転入力方式を3ワイヤ方式に変更
    処理内容2:運転データNo.2の回転速度の初期化(0[r/min])
    処理内容3:Configrationの実行

    """
    # 運転入力方式の変更(3ワイヤ)
    msg.slave_id = 0x00     # 号機選択(Hex): 0(ブロードキャスト)
    msg.func_code = 1       # ファンクションコード選択: 1(Write)
    msg.write_addr = 4160   # 先頭アドレス選択(Dec): 運転入力方式パラメータ
    msg.write_num = 1       # 書き込みデータサイズ: 1(32bit)
    msg.data[0] = 1         # 書き込みデータ: 0(2ワイヤ),1(3ワイヤ)
    pub.publish(msg)        # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                  # 処理待ち

    # 運転データ 回展速度No.2を0[r/min]に初期化
    msg.slave_id = 0x00     # 号機選択(Hex): 0(ブロードキャスト)
    msg.func_code = 1       # ファンクションコード選択: 1(Write)
    msg.write_addr = 1156   # 先頭アドレス選択(Dec): データNo.2 回転速度
    msg.write_num = 1       # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 0         # 書き込みデータ: 0[r/min]
    pub.publish(msg)        # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                  # 処理待ち

    # Configrationの実行
    msg.slave_id = 0x00   # 号機選択(Hex): 0(ブロードキャスト)
    msg.func_code = 1     # ファンクションコード選択: 1(Write)
    msg.write_addr = 396  # 先頭アドレス選択(Dec): Configration実行コマンド
    msg.write_num = 1     # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 1       # 書き込みデータ: 1(実行)
    pub.publish(msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                # 処理待ち


def main():
    """メイン関数

    処理内容1:運転指令(軸1 FWD方向、軸2 REV方向)
    処理内容2:運転データNo.2の回転速度の変更
    処理内容3:検出速度の確認
    処理内容4:停止指令(軸1 減速停止、軸2 瞬時停止)

    """
    global gMotor_spd1
    global gMotor_spd2

    rospy.init_node("sample3", anonymous=True)
    pub = rospy.Publisher("om_query1", om_query,
                          queue_size=1)  # OMにノードに送信するまでの定義

    rospy.Subscriber("om_state1", om_state,
                     stateCallback)  # レスポンスのコールバック定義
    rospy.Subscriber("om_response1", om_response,
                     resCallback)  # レスポンスのコールバック定義

    msg = om_query()  # OMノードで作成したメッセージを使用

    time.sleep(1)     # 1秒待機

    init(msg, pub)    # 初期化関数のコール

    # 運転指令(FWD方向)(M1,START/STOP,RUN/BRAKEをON)
    msg.slave_id = 0x01     # 号機選択(Hex): 1号機
    msg.func_code = 1       # ファンクションコード選択: 1(Write)
    msg.write_addr = 124    # 先頭アドレス選択(Dec): 動作コマンド
    msg.write_num = 1       # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 26        # 書き込みデータ: ONビット(0000 0000 0001 1010) = 26
    pub.publish(msg)        # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                  # 処理待ち

    # 運転指令(REV方向)(M1,START/STOP,RUN/BRAKE,FWD/REVをON)
    msg.slave_id = 0x02     # 号機選択(Hex): 2号機
    msg.func_code = 1       # ファンクションコード選択: 1(Write)
    msg.write_addr = 124    # 先頭アドレス選択(Dec): 動作コマンド
    msg.write_num = 1       # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 58        # 書き込みデータ: ONビット(0000 0000 0011 1010) = 58
    pub.publish(msg)        # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                  # 処理待ち

    for i in range(5):  # LOOP処理(5回)
        # 回転速度の設定
        msg.slave_id = 0x01         # 号機選択(Hex): 1号機
        msg.func_code = 1           # ファンクションコード選択: 1(Write)
        msg.write_addr = 1156       # 先頭アドレス選択(Dec): データNo.2 回転速度
        msg.write_num = 1           # 書き込みデータサイズ: 1 (32bit)
        msg.data[0] = 500 + i * 200 # 書き込みデータ: 500/700/900/1100/1300[r/min]
        pub.publish(msg)            # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        wait()                      # 処理待ち
        # 回転速度の設定
        msg.slave_id = 0x02         # 号機選択(Hex): 2号機
        msg.func_code = 1           # ファンクションコード選択: 1(Write)
        msg.write_addr = 1156       # 先頭アドレス選択(Dec): データNo.2 回転速度
        msg.write_num = 1           # 書き込みデータサイズ: 1 (32bit)
        msg.data[0] = 500 + i * 200 # 書き込みデータ: 500/700/900/1100/1300[r/min]
        pub.publish(msg)            # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        wait()                      # 処理待ち

        time.sleep(2)

        # 回転速度読み込み
        msg.slave_id = 0x01   # 号機選択(Hex): 1号機
        msg.func_code = 0     # ファンクションコード選択: 0(Read)
        msg.read_addr = 206   # 先頭アドレス選択(Dec): フィードバック速度[r/min](符号付)
        msg.read_num = 1      # 読み込みデータサイズ: 1 (32bit)
        pub.publish(msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        wait()
        msg.slave_id = 0x02   # 号機選択(Hex): 2号機
        msg.func_code = 0     # ファンクションコード選択: 0(Read)
        msg.read_addr = 206   # 先頭アドレス選択(Dec): フィードバック速度[r/min](符号付)
        msg.read_num = 1      # 読み込みデータサイズ: 1 (32bit)
        pub.publish(msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        wait()

        print("FeedbackSpeed1 = {0:}[r/min]".format(gMotor_spd1))     # 取得した速度の表示[r/min]
        print("FeedbackSpeed2 = {0:}[r/min]".format(gMotor_spd2))     # 取得した速度の表示[r/min]

    # 減速停止
    msg.slave_id = 0x01   # 号機選択(Hex): 1号機
    msg.func_code = 1     # ファンクションコード選択: 1(Write)
    msg.write_addr = 124  # 先頭アドレス選択(Dec): 動作コマンド
    msg.write_num = 1     # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 18      # 書き込みデータ: ONビット(0000 0000 0001 0010) = 18
    pub.publish(msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                # 処理待ち
    
    # 即時停止
    msg.slave_id = 0x02   # 号機選択(Hex): 2号機
    msg.func_code = 1     # ファンクションコード選択: 1(Write)
    msg.write_addr = 124  # 先頭アドレス選択(Dec): 動作コマンド
    msg.write_num = 1     # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 10      # 書き込みデータ: ONビット(0000 0000 0010 1010) = 10
    pub.publish(msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                # 処理待ち

    print("END")  # 終了表示

    rospy.spin()  # msg取得用関数の実行(必須)


if __name__ == '__main__':
    main()
