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
* @file	om_idshare_mode.cpp
* @brief ID Share Modeに対応するためのクラス
* @details
* @attention
* @note
履歴 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
* @version  Ver.1.05 April.5.2022 K.Yamaguchi
    - 新規作成
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
#include <string>
#include <iostream>
#include "ros/ros.h"
#include "om_modbus_master/om_idshare_mode.h"

namespace om_modbusRTU_node {

IdShareMode::IdShareMode() {}

IdShareMode::~IdShareMode() {}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* ID Shareモードか調べる 例外を返す
* @param[in]	int global_id
* @param[in]	int slave_id
* @return
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool IdShareMode::chkIdShareMode(int global_id, int slave_id) {
    if (global_id != slave_id) {
        throw "Exception in: " + (std::string) __func__;
    }
    return true;
}
/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* ID Shareモードか調べる boolを返す
* @param[in]	int global_id MEXE02でいうShare Control Global ID
* @param[in]	int slave_id
* @return		  true, false
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool IdShareMode::isIdShareMode(int global_id, int slave_id) {
    if (global_id != slave_id) {
        return false;
    }
    return true;
}
/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* ID Shareモードか調べてメンバに代入する
* @param[in]	int global_id MEXE02でいうShare Control Global ID
* @param[in]	int slave_id
* @return		  true, false
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void IdShareMode::setIdShareMode(int global_id, int slave_id) {
    is_idshare_mode_ = (global_id == slave_id);
}

/*---------------------------------------------------------------------------*/
/** 値チェック用関数
* @param[in]	int val: 入力値
* @param[in]	int min: 最小値
* @param[in]	int max: 最大値
* @return		  false:範囲外 true:範囲内
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool IdShareMode::chkRange(int val, int min, int max) {
    if (val >= min && val <= max) {
        return true;
    }
    return false;
}

/*---------------------------------------------------------------------------*/
/** 読み込みバイト数チェック 例外を返す
* @param[in]	read_num: 読み込みバイト数
* @return		  false:仕様外　true:仕様内
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool IdShareMode::chkRangeReadData(int read_num) {
    if (chkRange(read_num, MIN_READ_NUM, MAX_READ_NUM)) {
        return true;
    }
    throw DATA_ERROR;
}

/*---------------------------------------------------------------------------*/
/** 書き込みバイト数チェック 例外を返す
* @param[in]	write_num: 書き込みバイト数
* @return		  false:仕様外　true:仕様内
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool IdShareMode::chkRangeWriteData(int write_data) {
    if (chkRange(write_data, MIN_WRITE_NUM, MAX_WRITE_NUM)) {
        return true;
    }
    throw DATA_ERROR;
}

/*---------------------------------------------------------------------------*/
/** 軸数チェック　例外を返す
* @param[in]	axis_num: 軸数
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool IdShareMode::chkRangeAxis(int axis_num) {
    if (chkRange(axis_num, MIN_AXIS_NUM, MAX_AXIS_NUM)) {
        return true;
    }
    throw AXIS_NUM_ERROR;
}

/*---------------------------------------------------------------------------*/
/** global idチェック　例外を返す
* @param[in]	axis_num: 軸数
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool IdShareMode::chkRangeGlobalId(int global_id) {
    if (chkRange(global_id, MIN_GLOBAL_ID, MAX_GLOBAL_ID)) {
        return true;
    }
    throw GLOBAL_ID_ERROR;
}

/*---------------------------------------------------------------------------*/
/** ID Share read アドレスチェック　例外を返す
* @param[in]	addr: アドレス
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool IdShareMode::chkRangeShareReadAddress(int read_addr) {
    if (chkRange(read_addr, MIN_SHARE_READ_ADDRES, MAX_SHARE_READ_ADDRES)) {
        return true;
    }
    throw ADDRESS_ERROR;
}

/*---------------------------------------------------------------------------*/
/** share read/write アドレスチェック　例外を返す
* @param[in]	addr: アドレス
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool IdShareMode::chkRangeShareWriteAddress(int write_addr) {
    if (chkRange(write_addr, MIN_SHARE_WRITE_ADDRES, MAX_SHARE_WRITE_ADDRES)) {
        return true;
    }
    throw ADDRESS_ERROR;
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* クエリ(0x03)の生成
* @param[in]	int query_num
* @param[in]	int id 送信先のglobal_id
* @param[in]	int addr Share Read dataのレジスタアドレス
* @param[in]	int num 全軸合わせたデータ項目数
* @param[out]	std::vector<std::vector<char> >& pOut 生成されたクエリデータ
* @return		  クエリ数
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int IdShareMode::setQuery03h(int query_num, int id, int addr, int num,
                             std::vector<std::vector<char> >& pOut) {
    int ret;
    int max_ctrl_num = getParameter("axis_num");  // 最大軸数

    query_num = 0;  // 0番しか使われていない様子なので

    int axis_check_register =
        max_ctrl_num * 1;  // 全軸合わせた軸間エラーチェックのレジスタ数
    pOut[query_num].push_back(id);
    pOut[query_num].push_back(FUNCTION_CODE_READ);
    pOut[query_num].push_back((char)(0xff & (addr >> 8)));
    pOut[query_num].push_back((char)(0xff & (addr >> 0)));
    pOut[query_num].push_back(
        (char)(0xff & ((num * 2 + axis_check_register) >>
                       8)));  // 「データ」は上位下位レジスタがあるので*2
    pOut[query_num].push_back(
        (char)(0xff & ((num * 2 + axis_check_register) >> 0)));

    ret = chkFrm(ERROR_CHECK_INIT_VALUE,
                 (std::vector<unsigned char>&)pOut[query_num]);
    pOut[query_num].push_back((char)ret);
    pOut[query_num].push_back(ret >> 8);

    return ++query_num;
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* クエリ(0x010)の生成
*
* @param[in]	int query_num
* @param[in]	int id 送信先のglobal_id
* @param[in]	int addr Share Write dataのレジスタアドレス
* @param[in]	int num 全軸合わせたデータ項目数
* @param[in]	int *pVal
* @param[out]	std::vector<std::vector<char> >& pOut 生成されたクエリデータ
* @return		  クエリ数
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int IdShareMode::setQuery10h(int query_num, int id, int addr, int num,
                             int* pVal, std::vector<std::vector<char> >& pOut) {
    int ret;
    int val;

    query_num = 0;  // 0番しか使われていない様子なので

    pOut[query_num].push_back(id);
    pOut[query_num].push_back(FUNCTION_CODE_WRITE);
    pOut[query_num].push_back((char)(0xff & (addr >> 8)));
    pOut[query_num].push_back((char)(0xff & (addr >> 0)));
    pOut[query_num].push_back(
        (char)(0xff & (num * 2 >> 8)));  // データ項目数*2(上位・下位)*軸数
    pOut[query_num].push_back((char)(0xff & (num * 2 >> 0)));
    pOut[query_num].push_back(num * 2 * 2);  // レジスタ数*2

    for (int i = 0; i < num; i++) {
        val = *pVal;
        set4Byte(val, pOut[query_num]);
        pVal++;
    }

    ret = chkFrm(ERROR_CHECK_INIT_VALUE,
                 (std::vector<unsigned char>&)pOut[query_num]);
    pOut[query_num].push_back((char)ret);
    pOut[query_num].push_back(ret >> 8);

    return ++query_num;
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* クエリ(0x017)の生成
*
* @param[in]	int query_num
* @param[in]	int id 送信先のglobal_id
* @param[in]	int addr Share Write dataのレジスタアドレス
* @param[in]	int num 全軸合わせたデータ項目数
* @param[in]	int *pVal
* @param[out]	std::vector<std::vector<char> >& pOut 生成されたクエリデータ
* @return		  クエリ数
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int IdShareMode::setQuery17h(int id, int read_addr, int write_addr,
                             int read_num, int write_num, int* pVal,
                             std::vector<std::vector<char> >& pOut) {
    int val;
    int ret;
    int max_ctrl_num = getParameter("axis_num");  // 最大軸数

    int query_num = 0;  // 0番しか使われていない様子なので

    pOut[query_num].push_back(id);  // スレーブアドレス
    pOut[query_num].push_back(FUNCTION_CODE_READ_WRITE);  // ファンクションコード
    pOut[query_num].push_back(
        (char)(0xff & (read_addr >> 8)));  // 読み出しのID Shareレジスタアドレス
    pOut[query_num].push_back((char)(0xff & (read_addr >> 0)));
    pOut[query_num].push_back(
        (char)(0xff & ((read_num * 2 + max_ctrl_num) >>
                       8)));  // 読み出しのID Shareレジスタ数
                              // 軸間エラーチェック分も加える必要あり
    pOut[query_num].push_back(
        (char)(0xff & ((read_num * 2 + max_ctrl_num) >> 0)));
    pOut[query_num].push_back((
        char)(0xff & (write_addr >> 8)));  // 書き込みのID Shareレジスタアドレス
    pOut[query_num].push_back((char)(0xff & (write_addr >> 0)));
    pOut[query_num].push_back(
        (char)(0xff & (write_num * 2 >> 8)));  // 書き込みのID Shareレジスタ数
    pOut[query_num].push_back((char)(0xff & (write_num * 2 >> 0)));
    pOut[query_num].push_back(write_num *
                              4);  // 書き込みのクエリのレジスタ数の2倍の値

    for (int i = 0; i < write_num; i++) {
        val = *pVal;
        set4Byte(val, pOut[query_num]);
        pVal++;
    }

    ret = chkFrm(ERROR_CHECK_INIT_VALUE,
                 (std::vector<unsigned char>&)pOut[query_num]);
    pOut[query_num].push_back((char)ret);
    pOut[query_num].push_back(ret >> 8);
    return ++query_num;
}

/*---------------------------------------------------------------------------*/
/**
* intを1Byteに変換
* @param[in]	int 4byte
* @param[out]	std::vector<char>& pOut 1byte変換を格納
* @return		  なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void IdShareMode::set4Byte(int num, std::vector<char>& pOut) {
    pOut.push_back((unsigned char)((num & 0xff000000) >> 24));
    pOut.push_back((unsigned char)((num & 0x00ff0000) >> 16));
    pOut.push_back((unsigned char)((num & 0x0000ff00) >> 8));
    pOut.push_back((unsigned char)((num & 0x000000ff) >> 0));
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* パラメータサーバからデータを持ってくる
* @param[in] std::string param サーバから取り出したいパラメータ名称
* @return int サーバから取り出したパラメータ
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int IdShareMode::getParameter(std::string param) {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    std::string str;
    private_nh.getParam(param, str);
    return stoi(str);
}

/*---------------------------------------------------------------------------*/
/**
* 軸間エラーチェック CRC-16
* 1軸分のCRC-16の計算結果を返す
* @param[in] int init CRC-16の初期値
* @param[in] std::vector<unsigned char>& pFrm エラーチェックを計算するフレーム
* @return	int 計算結果
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int IdShareMode::chkFrm(int init, std::vector<unsigned char>& pFrm) {
    unsigned int i, j;
    unsigned int crc = init;
    auto itr = pFrm.begin();

    for (i = 0; i < pFrm.size(); i++) {
        crc ^= *itr;
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ ERROR_CHECK_XOR_VALUE;
            } else {
                crc = (crc >> 1);
            }
        }
        itr++;
    }
    return crc;
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応　レスポンスのエラーチェックが正しいか判定する。
* @param[in]	int qIni: CRC16の初期値
* @param[in]  char *pRes: エラーチェックが一致しているか調べるレスポンス
* @param[in]  int len: レスポンス長
* @return			　なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void IdShareMode::chkCrc(int qIni, unsigned char* pRes, int len) {
    int axis_num = getParameter("axis_num");

    chkCrcContents(qIni, pRes, len, axis_num);
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応　レスポンスのエラーチェックが正しいか判定する。
* @param[in]	int qIni: CRC16の初期値
* @param[in]  char *pRes: エラーチェックが一致しているか調べるレスポンス
* @param[in]  int len: レスポンス長
* @return			　なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void IdShareMode::chkCrcContents(int qIni, unsigned char* pRes, int len,
                                 int axis_num) {
    const int FUNC_CODE = 1;
    const int BYTE_NUM = 2;
    int byte = pRes[BYTE_NUM];
    int target_byte = 0;  // 次の軸間エラーチェックの要素数
    std::vector<std::vector<unsigned char> > vector_crc_target(
        axis_num +
        1);  //[x軸目][軸間エラーチェック用Byte列], +1は全体のエラーチェック分
    std::vector<int> vector_result_crc(axis_num +
                                       1);  //[x軸目][算出したエラーチェック]
    std::vector<int> vector_crc(axis_num +
                                1);  //[x軸目][レスポンス中のエラーチェック]

    // 書き込みのときはレスポンスの構造が異なる
    if (pRes[FUNC_CODE] == 0x10) {
        axis_num = 0;  // 0は1軸のこと
    }

    // 軸間エラーチェック用データを取得
    for (int axis = 0; axis < axis_num; axis++) {
        target_byte = 3 + (axis + 1) * byte / 2 - 2;
        std::vector<unsigned char> v(target_byte);
        for (int i = 0; i < target_byte; i++) {
            v[i] = pRes[i];
        }
        vector_crc_target[axis] = v;
    }

    // レスポンスから全体のエラーチェック用データを取得
    std::vector<unsigned char> v(len - 2);
    for (int i = 0; i < len - 2; i++) {
        v[i] = pRes[i];
    }
    vector_crc_target[axis_num] = v;

    // エラーチェックを計算
    for (int i = 0; i < axis_num + 1; i++) {
        vector_result_crc[i] = chkFrm(qIni, vector_crc_target[i]);
    }

    // 軸間エラーチェック取得
    int crc;
    for (int axis = 0; axis < axis_num; axis++) {
        crc = 0x0000;
        int erc_index =
            3 + (axis + 1) * byte / 2 - 2;  // 軸間エラーチェックの1byte手前
        crc |= pRes[erc_index + 1] << 8;
        crc |= pRes[erc_index];
        vector_crc[axis] = crc;
    }
    crc = 0x0000;
    crc |= pRes[len - 1] << 8;
    crc |= pRes[len - 2];
    vector_crc[axis_num] = crc;

    // レスポンス中のエラーチェックと算出したエラーチェックが一致するか判定
    for (unsigned int i = 0; i < vector_crc.size(); i++) {
        if (vector_crc[i] != vector_result_crc[i]) {
            throw CRC_ERROR;
        }
    }
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* 生成されるクエリが256Byte以下(仕様内)に収まるか判定する。データ数で判定している。
* @param[in]	const int write_data_num: クエリのデータ数(全軸の合計数)
* @return			　なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void IdShareMode::chkDataNumWriteQuery(const int write_data_num) {
    if (write_data_num > MAX_DATA_NUM_WRITE) {
        throw MODBUS_MESSAGE_LENGTH_ERROR;
    }
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* 生成されるクエリが256Byte以下(仕様内)に収まるか判定する。データ数で判定している。
* @param[in]	const int write_data_num: クエリのデータ数(全軸の合計数)
* @return			　なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void IdShareMode::chkDataNumReadWriteQuery(const int write_data_num) {
    if (write_data_num > MAX_DATA_NUM_READ_WRITE) {
        throw MODBUS_MESSAGE_LENGTH_ERROR;
    }
}

}  // namespace om_modbusRTU_node