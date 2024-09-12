#!/usr/bin/python
# -*- coding: utf-8 -*-

#
# ●対象機種:
#       AZ
#
# ●ドライバ設定:
#       パラメータ名称: [1軸目, 2軸目]
#       通信ID: [1, 2]
#       Baudrate: [230400, 230400]
#
# ●launchファイル設定:                       
#       com:="/dev/ttyUSB0" topicID:=1 baudrate:=230400 updateRate:=1000 firstGen:="" secondGen:="1,2," globalID:="10" axisNum:="2"
#
# ●処理内容：
#       ID Shareモードで2軸を5回2000[step]動かしたあと、1回-10000[step]動かす。
#       その間、0.3[s]周期で検出位置を表示させる。
#       

import rospy
import time
import math
import datetime
from om_modbus_master.msg import om_query
from om_modbus_master.msg import om_response
from om_modbus_master.msg import om_state

# グローバル変数
gState_driver = 0   # 0:通信可能　1:通信中
gState_error = 0    # 0:正常  1:no response 2: exception response
msg = om_query()
pub = None
gDoesWorkTimer = False # タイマー処理を実行するか

# ドライバ状態のコールバック関数
def stateCallback(res):
    global gState_driver, gState_error
    gState_driver = res.state_driver
    gState_error = res.state_error

# レスポンスのコールバック関数
def resCallback(res):
    # 例外応答のとき
    if gState_error == 2:
        print("Exception")
        return
    # ID Shareモードで読み込みを行ったとき
    if isIdShare(res) and (res.func_code==0x03):
        data_num = int(msg.read_num)    # データ数
        print('{0}: {1}[step], {2}[step]'.format(datetime.datetime.now(), res.data[0], res.data[1]))  # [0]:1軸目の検出位置、[1]:2軸目の検出位置

# パラメータサーバとresponseのslave_idから、現在ID Shareモードか調べる
def isIdShare(res):
    global_id = rospy.get_param("/om_modbusRTU_1/global_id")
    return int(global_id) == res.slave_id

# t[s]待機する
def wait(t):
    global gState_driver
    time.sleep(t)  
    
    while (gState_driver == 1):
        pass

# 一定周期で実行する処理
def timeProcess(event):
    global gDoesWorkTimer
    if gDoesWorkTimer:
        detectPosition()

# ID Shareモードで検出位置を取得する
def detectPosition():
    sencing_msg = om_query()
    sencing_msg.slave_id = 10       # スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
    sencing_msg.func_code = 0       # 0:Read
    sencing_msg.read_addr = 0x000C  # 読み出すアドレスの起点(検出位置)
    sencing_msg.read_num = 2        # 各軸1個ずつで計2個
    pub.publish(sencing_msg)        # 配信する

# 各軸のID Shareモードの設定を行う
def setShareReadWriteData():
    global pub
    global msg

    # 1軸目の設定
    msg.slave_id = 1    # 書き込むドライバのスレーブID
    msg.func_code = 1   # 1:Write
    msg.write_addr = 0x0980     # 書き込みの起点：Share Control Global IDのアドレス
    msg.write_num = 3   # 書き込むデータ数=3
    msg.data[0] = 10    # Share control global ID
    msg.data[1] = 2     # Share control number
    msg.data[2] = 1     # Share control local ID
    pub.publish(msg)    # 配信
    wait(0.03)          # 配信後の待機

    msg.write_addr = 0x0990     # 書き込みの起点：Share Read data[0]
    msg.write_num = 24  # 書き込むデータ数=24
    msg.data[0] = 45    # Share Read data[0] → DDO運転方式
    msg.data[1] = 46    # Share Read data[1] → DDO位置
    msg.data[2] = 47    # Share Read data[2] → DDO速度
    msg.data[3] = 48    # Share Read data[3] → DDO起動・変速レート
    msg.data[4] = 49    # Share Read data[4] → DDO停止レート
    msg.data[5] = 50    # Share Read data[5] → DDO運転電流
    msg.data[6] = 102   # Share Read data[6] → 検出位置
    msg.data[7] = 103   # Share Read data[7] → 検出速度(r/min)
    msg.data[8] = 0     # Share Read data[8] → 
    msg.data[9] = 0     # Share Read data[9] → 
    msg.data[10] = 0    # Share Read data[10] → 
    msg.data[11] = 0    # Share Read data[11] → 

    msg.data[12] = 45  # Share Write data[0] → DDO運転方式
    msg.data[13] = 46  # Share Write data[1] → DDO位置
    msg.data[14] = 47  # Share Write data[2] → DDO速度
    msg.data[15] = 51  # Share Write data[3] → DDO反映トリガ
    msg.data[16] = 0   # Share Write data[4] → 
    msg.data[17] = 0   # Share Write data[5] → 
    msg.data[18] = 0   # Share Write data[6] → 
    msg.data[19] = 0   # Share Write data[7] → 
    msg.data[20] = 0   # Share Write data[8] → 
    msg.data[21] = 0   # Share Write data[9] → 
    msg.data[22] = 0   # Share Write data[10] → 
    msg.data[23] = 0   # Share Write data[11] → 
    pub.publish(msg)   # 配信
    wait(0.03)         # 配信後の待機

    # 2軸目の設定
    msg.slave_id = 2    # 書き込むドライバのスレーブID
    msg.func_code = 1   # 1:Write
    msg.write_addr = 0x0980     # 書き込みの起点：Share Control Global IDのアドレス
    msg.write_num = 3   # 書き込むデータ数=3
    msg.data[0] = 10    # Share control global ID
    msg.data[1] = 2     # Share control number
    msg.data[2] = 2     # Share control local ID
    pub.publish(msg)    # 配信
    wait(0.03)          # 配信後の待機

    msg.write_addr = 0x0990 # 書き込むアドレスの起点: Share Read data[0]
    msg.write_num = 24  # 書き込むデータ数=24
    msg.data[0] = 45    # Share Read data[0] → DDO運転方式
    msg.data[1] = 46    # Share Read data[1] → DDO位置
    msg.data[2] = 47    # Share Read data[2] → DDO速度
    msg.data[3] = 48    # Share Read data[3] → DDO起動・変速レート
    msg.data[4] = 49    # Share Read data[4] → DDO停止レート
    msg.data[5] = 50    # Share Read data[5] → DDO運転電流
    msg.data[6] = 102   # Share Read data[6] → 検出位置
    msg.data[7] = 103   # Share Read data[7] → 検出速度(r/min)
    msg.data[8] = 0     # Share Read data[8] → 
    msg.data[9] = 0     # Share Read data[9] → 
    msg.data[10] = 0    # Share Read data[10] → 
    msg.data[11] = 0    # Share Read data[11] → 

    msg.data[12] = 45  # Share Write data[0] → DDO運転方式
    msg.data[13] = 46  # Share Write data[1] → DDO位置
    msg.data[14] = 47  # Share Write data[2] → DDO速度
    msg.data[15] = 51  # Share Write data[3] → DDO反映トリガ
    msg.data[16] = 0   # Share Write data[4] → 
    msg.data[17] = 0   # Share Write data[5] → 
    msg.data[18] = 0   # Share Write data[6] → 
    msg.data[19] = 0   # Share Write data[7] → 
    msg.data[20] = 0   # Share Write data[8] → 
    msg.data[21] = 0   # Share Write data[9] → 
    msg.data[22] = 0   # Share Write data[10] → 
    msg.data[23] = 0   # Share Write data[11] → 
    pub.publish(msg)   # 配信
    wait(0.03)         # 配信後の待機

def main():
    global pub
    global msg
    global gDoesWorkTimer
    global gIsTimeProcEnd

    rospy.init_node("idshare_sample1_1", anonymous=True)
    pub = rospy.Publisher("om_query1", om_query, queue_size=1)
    rospy.Subscriber("om_state1", om_state, stateCallback)
    rospy.Subscriber("om_response1", om_response, resCallback)  # ドライバのレスポンスに関するメッセージを受け取るsubscriber作成
    time.sleep(1)

    # ユニキャストモードで通信するため、global_idを-1に設定する
    rospy.set_param("/om_modbusRTU_1/global_id", "-1")

    # ID Shareモードの設定
    setShareReadWriteData()

    # ID Shareモードで通信するため、global_id=10に設定
    rospy.set_param("/om_modbusRTU_1/global_id", "10")

    # 一定周期で検出位置を取得するようにする
    detectPosition()        # 運転前の検出位置を取得
    wait(0.03)               # 配信後の待機
    rospy.Timer(rospy.Duration(0.3), timeProcess)

    # ID Shareモードで各軸2000[step]の運転を5回行う
    msg.slave_id = 10           # スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
    msg.func_code = 1           # 0:read 1:write 2:read/write
    msg.write_addr = 0x0000     # DDO運転方式から書き込む(Modbus-Share設定でそのように設定している)
    msg.write_num =8            # 全軸合わせたデータ項目数を代入する 4個*2軸分
    #1軸目のデータ
    msg.data[0] = 2         # DDO運転方式 2:相対位置決め(指令位置基準)
    msg.data[1] = 2000      # DDO運転位置
    msg.data[2] = 2000      # DDO運転速度
    msg.data[3] = 1         # DDO運転反映トリガ
    # 2軸目のデータ
    msg.data[4] = 2         # DDO運転方式 2:相対位置決め(指令位置基準)
    msg.data[5] = 2000      # DDO運転位置
    msg.data[6] = 2000      # DDO運転速度
    msg.data[7] = 1         # DDO運転反映トリガ
    # 配信
    for i in range(5):
        gDoesWorkTimer = False  # タイマー処理停止
        wait(0.03)              # タイマー処理中のpublishとぶつからないための待機
        pub.publish(msg)        # msgの配信
        wait(0.03)              # 配信後の待機
        gDoesWorkTimer = True   # タイマー処理再開
        wait(2)                 # 運転終了まで待機

    # ID Shareモードで各軸ずつ-10000[step]運転させる
    msg.slave_id = 10       # スレーブID指定
    msg.func_code = 1       # 1:write
    msg.write_addr = 0x0000 # 書き込むアドレスの起点:DDO運転方式
    msg.write_num = 8       # 書き込むデータ数
    #1軸目のデータ
    msg.data[0] = 2         # DDO運転方式 2:相対位置決め(指令位置基準)
    msg.data[1] = -10000    # DDO運転位置
    msg.data[2] = 5000      # DDO運転速度
    msg.data[3] = 1         # DDO反映トリガ
    #1軸目のデータ
    msg.data[4] = 2         # DDO運転方式 2:相対位置決め(指令位置基準)
    msg.data[5] = -10000    # DDO運転位置
    msg.data[6] = 5000      # DDO運転速度
    msg.data[7] = 1         # DDO反映トリガ
    gDoesWorkTimer = False  # タイマー処理停止
    wait(0.03)              # タイマー処理中のpublishとぶつからないための待機
    pub.publish(msg)        # msgの配信
    wait(0.03)              # 配信後の待機
    gDoesWorkTimer = True   # タイマー処理再開
    wait(3)                 # 運転終了まで待機

    gDoesWorkTimer = False

    print("END")            
    rospy.spin()


if __name__ == '__main__':
    main()
