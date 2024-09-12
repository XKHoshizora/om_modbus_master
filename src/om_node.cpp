/*
 * Copyright (c) 2019, ORIENTAL MOTOR CO.,LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the ORIENTAL MOTOR CO.,LTD. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
* @file	om_node.cpp
* @brief オブジェクト生成
* @details
* @attention
* @note

履歴 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
* @version  Ver.1.05 April.5.2022 K.Yamaguchi
    - ID Share対応

* @version	Ver.1.00 Mar.11.2019 T.Takahashi
                         - 新規作成
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
#include "om_modbus_master/om_node.h"

#include "om_modbus_master/om_base.h"
#include "om_modbus_master/om_broadcast.h"
#include "om_modbus_master/om_first_gen.h"
#include "om_modbus_master/om_ros_message.h"
#include "om_modbus_master/om_second_gen.h"
#include "ros/ros.h"

using std::cout;
using std::endl;
using std::string;
namespace ns = om_modbusRTU_node;

namespace om_modbusRTU_node {

/*---------------------------------------------------------------------------*/
/** 初期化関数
* 		    オブジェクトの生成
* @param[in]  vector<int>& first
* @param[in]  vector<int>& second
* @param[in]  Base *base_obj
* @param[in]  FirstGenModbusRTU *modbus_obj[]
* @param[in]  RosMessage *rosmes_obj
* @return     なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
// 第一世代や第二世代のクエリを生成する
void init(std::vector<int> &first, std::vector<int> &second, Base *base_obj,
          ICheckData *check_obj[], IConvertQueryAndResponse *convert_obj[],
          ICheckIdShareMode *check_idshare_obj, RosMessage *rosmes_obj) {
    // 入力変数の妥当性チェック
    if (0 == first.size() && 0 == second.size()) {
        ROS_ERROR("Error: No slave ID specified");
        throw;
    }
    if (MAX_SLAVE_NUM < (first.size() + second.size())) {
        ROS_ERROR("Error: Maximum number of connectable devices exceeded");
        throw;
    }

    for (int i = 0; i < MAX_SLAVE_ID; i++) {
        check_obj[i] = nullptr;
        convert_obj[i] = nullptr;
    }

    cout << "" << endl;
    cout << "Connectable devices" << endl;
    check_obj[0] = new ns::BroadcastModbusRTU();
    convert_obj[0] = new ns::BroadcastModbusRTU();

    // 第一世代のクエリを作成
    cout << " * /First Generation Number: " << first.size() << endl;
    for (unsigned int i = 0; i < first.size(); i++) {
        if (MIN_SLAVE_ID > first[i] || MAX_SLAVE_ID < first[i]) {
            ROS_ERROR("Error: Specified slave ID out of range");
            throw;
        }
        cout << " * /First Generation(ID):" << first[i] << endl;
        check_obj[first[i]] = new ns::FirstGenModbusRTU();
        convert_obj[first[i]] = new ns::FirstGenModbusRTU();

        ICheckData *pobj = check_obj[first[i]];
        check_obj[0]->setMaxAddress(pobj->getMaxAddress());
        check_obj[0]->setMaxDataNum(pobj->getMaxDataNum());
    }

    // 第二世代のクエリを作成（第一世代を継承している）
    cout << " * /Second Generation Number: " << second.size() << endl;
    for (unsigned int i = 0; i < second.size(); i++) {
        if (MIN_SLAVE_ID > second[i] || MAX_SLAVE_ID < second[i]) {
            ROS_ERROR("Error: Specified slave ID out of range");
            throw;
        }
        cout << " * /Second Generation(ID):" << second[i] << endl;
        check_obj[second[i]] = new ns::SecondGenModbusRTU();
        convert_obj[second[i]] = new ns::SecondGenModbusRTU();

        ICheckData *pobj = check_obj[second[i]];
        check_obj[0]->setMaxAddress(pobj->getMaxAddress());
        check_obj[0]->setMaxDataNum(pobj->getMaxDataNum());
    }

    // 設定したglobalIDの表示(追加)
    std::string str;
    ros::NodeHandle private_nh("~");
    private_nh.getParam("global_id", str);
    cout << " * /global_id:" << str << endl;

    rosmes_obj->init(base_obj, &check_obj[0],
                     check_idshare_obj);  // subscriberやpublisherの登録
    base_obj->init(rosmes_obj, &check_obj[0], &convert_obj[0],
                   check_idshare_obj);  // ポート初期化
}

/*---------------------------------------------------------------------------*/
/** 設定しているglobal_idの表示
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void dispGlobalId(std::string str) { cout << "global_id:" << str << endl; }

/*---------------------------------------------------------------------------*/
/** 終了処理関数
        オブジェクトの削除
* @param[in]   vector<int>& first
* @param[in]   vector<int>& second
* @param[in]   Base *base_obj
* @param[in]   FirstGenModbusRTU *modbus_obj[]
* @param[in]   RosMessage *rosmes_obj
* @return      なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void finalize(std::vector<int> &first, std::vector<int> &second, Base *base_obj,
              ICheckData *check_obj[], IConvertQueryAndResponse *convert_obj[],
              RosMessage *rosmes_obj) {
    delete rosmes_obj;
    delete base_obj;
    delete check_obj[0];
    delete convert_obj[0];
    for (unsigned int i = 0; i < first.size(); i++) {
        delete check_obj[first[i]];
        delete convert_obj[first[i]];
    }

    for (unsigned int i = 0; i < second.size(); i++) {
        delete check_obj[second[i]];
        delete convert_obj[second[i]];
    }
}

/*---------------------------------------------------------------------------*/
/** bgMain関数
       bgループ
* @param[in]  なし
* @return　　　なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void bgMain() { ros::spin(); }

/*---------------------------------------------------------------------------*/
/** 文字列→数値変換
 文字列を数値に変換する
* @param[in]　　　string& src 変換文字列
* @param[in]　　　char delim  区切り文字
* @return　　　　　vector<int>
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
std::vector<int> split(string &src, char delim = ',') {
    std::vector<int> vec;
    std::istringstream iss(src);
    string tmp;

    if ("\0" == src) {
        return vec;
    }

    chkComma(src);

    while (getline(iss, tmp, delim)) {
        vec.push_back(stoi(tmp));
    }
    return vec;
}

/*---------------------------------------------------------------------------*/
/** 重複チェック

* @param[in]　　　第１世代(配列)
* @param[in]　　　第2世代(配列)
* @return　　　　  void
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void chkDuplication(std::vector<int> &first, std::vector<int> &second) {
    int size;
    std::vector<int> inter;

    std::sort(first.begin(), first.end());
    std::sort(second.begin(), second.end());

    size = first.size();
    for (int i = 0; i < (size - 1); i++) {
        if (first[i] == first[i + 1]) {
            ROS_ERROR("Error: Duplicate firstGen");
            throw;
        }
    }

    size = second.size();
    for (int i = 0; i < (size - 1); i++) {
        if (second[i] == second[i + 1]) {
            ROS_ERROR("Error: Duplicate secondGen");
            throw;
        }
    }

    std::set_intersection(first.begin(), first.end(), second.begin(),
                          second.end(), std::back_inserter(inter));
    if (0 < inter.size()) {
        ROS_ERROR("Error: Duplicate firstGen and secondGen");
        throw;
    }
}

/*---------------------------------------------------------------------------*/
/** 文字列からスペースを削除

* @param[in]　第１世代(文字列)
* @param[in]　第2世代(文字列)
* @return　　　なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void deleteSpace(string &first, string &second) {
    first.erase(remove(first.begin(), first.end(), ' '), first.end());
    first.erase(remove(first.begin(), first.end(), '\t'), first.end());

    second.erase(remove(second.begin(), second.end(), ' '), second.end());
    second.erase(remove(second.begin(), second.end(), '\t'), second.end());
}

/*---------------------------------------------------------------------------*/
/** カンマ有無チェック

* @param[in]　チェックする文字列
* @return　　　なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void chkComma(string &str) {
    if (string::npos == str.find(',')) {
        ROS_ERROR("Error: Comma does not exist");
        throw;
    }
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* launchで設定したID Shareモード関係のパラメータが仕様内か調べる。例外を返す。
* @param[in]　なし
* @return　　　なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void chkIdShareParameter(int global_id, int axis_num) {
    // global_idが仕様外かつ-1でないときに例外を返す。
    if (global_id < MIN_GLOBAL_ID || global_id > MAX_GLOBAL_ID) {
        if (global_id != UNUSE_GLOBAL_ID) {
            ROS_ERROR("Error: Invalid number of globalId");
            throw;
        }
    }
    if (axis_num < MIN_AXIS_NUM || axis_num > MAX_AXIS_NUM) {
        ROS_ERROR("Error: Invalid number of axisNum");
        throw;
    }
}

}  // namespace om_modbusRTU_node

/*---------------------------------------------------------------------------*/
/** main関数
                一番最初にここが実行される
* @param[in]  int argc
* @param[in]  char **argv
* @return　　　int
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int main(int argc, char **argv) {
    string first_str, second_str, global_id_str;
    std::vector<int> first_gen, second_gen;

    ros::init(argc, argv, "om");
    ros::NodeHandle private_nh("~");

    // パラメータサーバから文字列を取り出す
    private_nh.getParam("first_gen", first_str);
    private_nh.getParam("second_gen", second_str);

    ns::deleteSpace(first_str, second_str);

    // ID Share関係のパラメータ取得
    string globa_id_str, axis_num_str;
    int global_id, axis_num;
    private_nh.getParam("global_id", global_id_str);
    private_nh.getParam("axis_num", axis_num_str);
    global_id = std::stoi(global_id_str);
    axis_num = std::stoi(axis_num_str);

    try {
        first_gen = ns::split(first_str);
        second_gen = ns::split(second_str);

        ns::chkDuplication(first_gen, second_gen);
        ns::chkIdShareParameter(global_id, axis_num);

        ns::RosMessage *ros_mes = new ns::
            RosMessage;  // RosMessageクラス(上位と配信/購読する)のインスタンス作成
        ns::Base *base =
            new ns::Base;  // シリアル通信するクラスのインスタンス生成
        ns::ICheckData *check_obj[ns::MAX_SLAVE_ID + 1];
        ns::IConvertQueryAndResponse *convert_obj[ns::MAX_SLAVE_ID + 1];
        ns::ICheckIdShareMode *pcheck_idshare_mode = new ns::SecondGenModbusRTU;

        ns::init(first_gen, second_gen, base, check_obj, convert_obj,
                 pcheck_idshare_mode, ros_mes);
        ns::bgMain();
        ns::finalize(first_gen, second_gen, base, check_obj, convert_obj,
                     ros_mes);
    } catch (int err) {
    }

    return 0;
}
