#!/usr/bin/python
# -*- coding: utf-8 -*-

#
# ●対象機種:
#       BLV-R
#
# ●ドライバ設定:
#       パラメータ名称: [1軸目, 2軸目]
#       通信ID: [1, 2]
#       Baudrate: [230400, 230400]
#
# ●launchファイル設定:                       
#       com:="/dev/ttyUSB0" topicID:=1 baudrate:=230400 updateRate:=1000 firstGen:="" secondGen:="1,2," globalID:="10" axisNum:="2"
#
# ●処理内容: 
#       Writeにより連続運転(速度制御)でモーターを運転させる。
#       Readにより一定周期でモーターの検出速度を取得し、表示させる。

import rospy
import time
import datetime
from om_modbus_master.msg import om_query
from om_modbus_master.msg import om_response
from om_modbus_master.msg import om_state

# グローバル変数
gState_driver = 0   # 0:通信可能 1:通信中
gState_mes = 0      # 0:メッセージなし 1:メッセージ到達 2:メッセージエラー 
gState_error = 0    # 0:エラーなし 1:無応答 2:例外応答
pub = None
msg = om_query()
gMotor_pos = 0
gMotor_speed = 0
gDoesWorkTimer = False

# ドライバ状態のコールバック関数
def stateCallback(res):
    global gState_driver
    global gState_mes
    global gState_error
    gState_driver = res.state_driver
    gState_mes = res.state_mes
    gState_error = res.state_error

# レスポンスのコールバック関数
def resCallback(res):
    global gMotor_pos
    global gMotor_speed
    # 例外応答のとき
    if gState_error == 2:
        print("Exception")
        return
    # ID Shareモードのとき
    if isIdShare(res) and (res.func_code==0x03):
        axis_num = int(rospy.get_param("/om_modbusRTU_1/axis_num")) # roslaunchで設定した軸数情報の取得
        data_num = int(msg.read_num)    # データ数
        for axis in range(axis_num):
            print('{0}: {1}[r/min], {2}[r/min]'.format(datetime.datetime.now(), res.data[0], res.data[1]))  # [0]:1軸目の検出速度、[1]:2軸目の検出速度

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
        msg.slave_id = 10       # スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
        msg.func_code = 0       # 0:Read
        msg.read_addr = 0x000E  # 読み出すアドレスの起点
        msg.read_num = 2        # 各軸1個ずつ
        pub.publish(msg)        # 配信する

# 各軸のID Shareモードの設定を行う
def setShareReadWriteData():
    global pub
    global msg

    # 1軸目の設定
    msg.slave_id = 1    # 書き込むドライバのスレーブID
    msg.func_code = 1   # 1:Write
    msg.write_addr = 0x0980     # 書き込みの起点：Share Control Global IDのアドレス
    msg.write_num = 3   # 書き込む数
    msg.data[0] = 10    # Share control global ID
    msg.data[1] = 2     # Share control number
    msg.data[2] = 1     # Share control local ID
    pub.publish(msg)    # 配信する
    wait(0.03)

    msg.write_addr = 0x0990     # 書き込みの起点：Share Read data[0]
    msg.write_num = 24  # 書き込むデータ数*軸数=24
    msg.data[0] = 45    # Share Read data[0] → DDO運転方式
    msg.data[1] = 46    # Share Read data[1] → DDO位置
    msg.data[2] = 47    # Share Read data[2] → DDO速度
    msg.data[3] = 48    # Share Read data[3] → DDO加速レート
    msg.data[4] = 49    # Share Read data[4] → DDO減速レート
    msg.data[5] = 50    # Share Read data[5] → DDOトルク制限値
    msg.data[6] = 102   # Share Read data[6] → 検出位置[step]
    msg.data[7] = 103   # Share Read data[7] → 検出速度[r/min]
    msg.data[8] = 0     # Share Read data[8] → 
    msg.data[9] = 0     # Share Read data[9] → 
    msg.data[10] = 0    # Share Read data[10] → 
    msg.data[11] = 0    # Share Read data[11] → 

    msg.data[12] = 45   # Share Write data[0] → DDO運転方式
    msg.data[13] = 46   # Share Write data[1] → DDO位置
    msg.data[14] = 47   # Share Write data[2] → DDO速度
    msg.data[15] = 48   # Share Write data[3] → DDO加速レート
    msg.data[16] = 49   # Share Write data[4] → DDO減速レート
    msg.data[17] = 51   # Share Write data[5] → DDO反映トリガ
    msg.data[18] = 0    # Share Write data[6] → 
    msg.data[19] = 0    # Share Write data[7] → 
    msg.data[20] = 0    # Share Write data[8] → 
    msg.data[21] = 0    # Share Write data[9] → 
    msg.data[22] = 0    # Share Write data[10] → 
    msg.data[23] = 0    # Share Write data[11] → 
    pub.publish(msg)
    wait(0.03)

    # 2軸目の設定
    msg.slave_id = 2    # 書き込むドライバのスレーブID
    msg.func_code = 1   # 1:Write
    msg.write_addr = 0x0980     # 書き込みの起点：Share Control Global IDのアドレス
    msg.write_num = 3   # 書き込む数
    msg.data[0] = 10    # Share control global ID
    msg.data[1] = 2     # Share control number
    msg.data[2] = 2     # Share control local ID
    pub.publish(msg)
    wait(0.03)

    msg.write_addr = 0x0990     # 書き込みの起点：Share Read data[0]
    msg.write_num = 24  # 書き込むデータ数*軸数=24
    msg.data[0] = 45    # Share Read data[0] → DDO運転方式
    msg.data[1] = 46    # Share Read data[1] → DDO位置
    msg.data[2] = 47    # Share Read data[2] → DDO速度
    msg.data[3] = 48    # Share Read data[3] → DDO加速レート
    msg.data[4] = 49    # Share Read data[4] → DDO減速レート
    msg.data[5] = 50    # Share Read data[5] → DDOトルク制限値
    msg.data[6] = 102   # Share Read data[6] → DDO検出位置[step]
    msg.data[7] = 103   # Share Read data[7] → DDO検出速度[r/min]
    msg.data[8] = 0     # Share Read data[8] → 
    msg.data[9] = 0     # Share Read data[9] → 
    msg.data[10] = 0    # Share Read data[10] → 
    msg.data[11] = 0    # Share Read data[11] → 

    msg.data[12] = 45   # Share Write data[0] → DDO運転方式
    msg.data[13] = 46   # Share Write data[1] → DDO位置
    msg.data[14] = 47   # Share Write data[2] → DDO速度
    msg.data[15] = 48   # Share Write data[3] → DDO加速レート
    msg.data[16] = 49   # Share Write data[4] → DDO減速レート
    msg.data[17] = 51   # Share Write data[5] → DDO反映トリガ
    msg.data[18] = 0    # Share Write data[6] → 
    msg.data[19] = 0    # Share Write data[7] → 
    msg.data[20] = 0    # Share Write data[8] → 
    msg.data[21] = 0    # Share Write data[9] → 
    msg.data[22] = 0    # Share Write data[10] → 
    msg.data[23] = 0    # Share Write data[11] → 
    pub.publish(msg)
    wait(0.03)

def main():
    global gState_mes
    global gState_error
    global pub
    global gDoesWorkTimer
    MESSAGE_ERROR = 2
    EXCEPTION_RESPONSE = 2

    rospy.init_node("blv_idshare_sample1_1", anonymous=True)    # 上位ノード作成
    pub = rospy.Publisher("om_query1", om_query, queue_size=1)  # masterにメッセージを渡すpublisher作成
    rospy.Subscriber("om_state1", om_state, stateCallback)      # 通信状態に関するメッセージを受け取るsubscriber作成
    rospy.Subscriber("om_response1", om_response, resCallback)  # ドライバのレスポンスに関するメッセージを受け取るsubscriber作成
    wait(1)
    
    # ユニキャストモードで通信するため、global_idを-1に設定する
    rospy.set_param("/om_modbusRTU_1/global_id", "-1")

    # ID Shareモードの設定
    setShareReadWriteData()

    # 運転指令(S-ONをONする)
    msg.slave_id = 1      # スレーブID
    msg.func_code = 1     # ファンクションコード: 0:Read 1:Write 2:Read/Write
    msg.write_addr = 124  # アドレス指定： ドライバ入力指令
    msg.write_num = 1     # 書き込みデータ数: 1           
    msg.data[0] = 1       # S-ONを立ち上げる
    pub.publish(msg)      # 配信
    wait(0.3)

    msg.slave_id = 2      # スレーブID
    msg.func_code = 1     # ファンクションコード: 0:Read 1:Write 2:Read/Write
    msg.write_addr = 124  # アドレス指定： ドライバ入力指令
    msg.write_num = 1     # 書き込みデータ数: 1
    msg.data[0] = 1       # S-ONを立ち上げる
    pub.publish(msg)      # 配信
    wait(0.3)

    # ID Shareモードで通信するため、global_id=10に設定
    rospy.set_param("/om_modbusRTU_1/global_id", "10")

    # 一定周期でtimeProcessを実行する
    gDoesWorkTimer = True
    rospy.Timer(rospy.Duration(0.3), timeProcess)

    # ID Shareモードで各モーターを運転する
    # 120[r/min]で運転させる
    msg.slave_id = 10           # スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
    msg.func_code = 1           # 0:read 1:write 2:read/write
    msg.write_addr = 0x0000     # 書き込むアドレスの起点
    msg.write_num =12           # 全軸合わせたデータ項目数を代入する 
    # 1軸目のデータ
    msg.data[0] = 16        # DDO運転方式 16:連続運転(速度制御)
    msg.data[1] = 0         # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
    msg.data[2] = 120       # DDO運転速度(初期単位：r/min)
    msg.data[3] = 1000      # DDO加速レート(初期単位：ms)
    msg.data[4] = 1000      # DDO減速レート(初期単位：ms)
    msg.data[5] = 1         # DDO運転トリガ設定
    # 2軸目のデータ
    msg.data[6] = 16        # DDO運転方式 16:連続運転(速度制御)
    msg.data[7] = 0         # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
    msg.data[8] = 120       # DDO運転速度(初期単位：r/min)
    msg.data[9] = 1000      # DDO加速レート(初期単位：ms)
    msg.data[10] = 1000     # DDO減速レート(初期単位：ms)
    msg.data[11] = 1        # DDO運転トリガ設定
    # 配信
    gDoesWorkTimer = False
    wait(0.03)
    pub.publish(msg)
    wait(0.03)
    gDoesWorkTimer = True
    wait(5)

    # 240[r/min]で運転させる
    msg.slave_id = 10       # スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
    msg.func_code = 1       # 0:read 1:write 2:read/write
    msg.write_addr = 0x0000 # 書き込むアドレスの起点
    msg.write_num =12       # 書き込むデータ数*軸数=12
    # 1軸目のデータ
    msg.data[0] = 16        # DDO運転方式 16:連続運転(速度制御)
    msg.data[1] = 0         # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
    msg.data[2] = 240       # DDO運転速度(初期単位：r/min)
    msg.data[3] = 1000      # DDO加速レート(初期単位：ms)
    msg.data[4] = 1000      # DDO減速レート(初期単位：ms)
    msg.data[5] = 1         # DDO運転トリガ設定
    # 2軸目のデータ
    msg.data[6] = 16        # DDO運転方式 16:連続運転(速度制御)
    msg.data[7] = 0         # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
    msg.data[8] = 240       # DDO運転速度(初期単位：r/min)
    msg.data[9] = 1000      # DDO加速レート(初期単位：ms)
    msg.data[10] = 1000     # DDO減速レート(初期単位：ms)
    msg.data[11] = 1        # DDO運転トリガ設定
    # 配信
    gDoesWorkTimer = False  # タイマー処理停止
    wait(0.03)
    pub.publish(msg)
    wait(0.03)
    gDoesWorkTimer = True   # タイマー処理再開
    wait(5)

    # 360[r/min]で運転させる
    msg.slave_id = 10       # スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
    msg.func_code = 1       # 0:read 1:write 2:read/write
    msg.write_addr = 0x0000 # 書き込むアドレスの起点
    msg.write_num =12       # 書き込むデータ数*軸数=12
    # 1軸目のデータ
    msg.data[0] = 16        # DDO運転方式 16:連続運転(速度制御)
    msg.data[1] = 0         # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
    msg.data[2] = 360       # DDO運転速度(初期単位：r/min)
    msg.data[3] = 1000      # DDO加速レート(初期単位：ms)
    msg.data[4] = 1000      # DDO減速レート(初期単位：ms)
    msg.data[5] = 1         # DDO運転トリガ設定
    # 2軸目のデータ
    msg.data[6] = 16        # DDO運転方式 16:連続運転(速度制御)
    msg.data[7] = 0         # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
    msg.data[8] = 360       # DDO運転速度(初期単位：r/min)
    msg.data[9] = 1000      # DDO加速レート(初期単位：ms)
    msg.data[10] = 1000     # DDO減速レート(初期単位：ms)
    msg.data[11] = 1        # DDO運転トリガ設定
    # 配信
    gDoesWorkTimer = False
    wait(0.03)
    pub.publish(msg)
    wait(0.03)
    gDoesWorkTimer = True
    wait(5)

    # 停止するまで減速させる
    msg.slave_id = 10       # スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
    msg.func_code = 1       # 0:read 1:write 2:read/write
    msg.write_addr = 0x0000 # 書き込むアドレスの起点
    msg.write_num =12       # 書き込むデータ数*軸数=12
    # 1軸目のデータ
    msg.data[0] = 16        # DDO運転方式 16:連続運転(速度制御)
    msg.data[1] = 0         # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
    msg.data[2] = 0         # DDO運転速度(初期単位：r/min)
    msg.data[3] = 1000      # DDO加速レート(初期単位：ms)
    msg.data[4] = 3000      # DDO減速レート(初期単位：ms)
    msg.data[5] = 1         # DDO運転トリガ設定
    # 2軸目のデータ
    msg.data[6] = 16        # DDO運転方式 16:連続運転(速度制御)
    msg.data[7] = 0         # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
    msg.data[8] = 0         # DDO運転速度(初期単位：r/min)
    msg.data[9] = 1000      # DDO加速レート(初期単位：ms)
    msg.data[10] = 3000     # DDO減速レート(初期単位：ms)
    msg.data[11] = 1        # DDO運転トリガ設定
    # 配信
    gDoesWorkTimer = False
    wait(0.03)
    pub.publish(msg)
    wait(0.03)
    gDoesWorkTimer = True
    wait(4)

    # ユニキャストモードで通信するため、global_id=-1に設定
    gDoesWorkTimer = False
    rospy.set_param("/om_modbusRTU_1/global_id", "-1")
    # 運転指令(S-ONをOFFする)
    msg.slave_id = 1      # スレーブID
    msg.func_code = 1     # ファンクションコード: 0:Read 1:Write 2:Read/Write
    msg.write_addr = 124  # アドレス指定： ドライバ入力指令
    msg.write_num = 1     # 書き込みデータ数: 1           
    msg.data[0] = 0       # S-ONを立ち下げる
    wait(0.03)
    pub.publish(msg)      # 配信
    wait(0.1)

    msg.slave_id = 2      # スレーブID
    msg.func_code = 1     # ファンクションコード: 0:Read 1:Write 2:Read/Write
    msg.write_addr = 124  # アドレス指定： ドライバ入力指令
    msg.write_num = 1     # 書き込みデータ数: 1
    msg.data[0] = 0       # S-ONを立ち下げる
    wait(0.03)
    pub.publish(msg)      # 配信
    wait(0.1)


    if (gState_mes == MESSAGE_ERROR):
        stop(msg, pub)
        exit()  

    if (gState_error == EXCEPTION_RESPONSE):
        stop(msg, pub)
        exit()

    print("END")
    rospy.spin()

if __name__ == '__main__':
    main()
