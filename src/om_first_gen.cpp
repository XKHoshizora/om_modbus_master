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
* @file	om_first_gen.cpp
* @brief Modbus RTUのクエリを生成する(第１世代)
* @details
* @attention
* @note

履歴 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
* @version  Ver.1.05 April.5.2022 K.Yamaguchi
    - ID Share対応

* @version	Ver.1.00 Mar.11.2019 T.Takahashi
                         - 新規作成
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
#include "om_modbus_master/om_first_gen.h"

namespace om_modbusRTU_node {

FirstGenModbusRTU::FirstGenModbusRTU() {}

FirstGenModbusRTU::~FirstGenModbusRTU() {}

/*---------------------------------------------------------------------------*/
/** エラーチェック CRC-16
CRC-16の計算結果を返す
* @param[in]	int qIni 初期値
* @param[in]	std::vector<unsigned char>& pFrm	フレーム
* @return		  int 計算結果
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int FirstGenModbusRTU::chkFrm(int qIni, std::vector<unsigned char>& pFrm) {
    unsigned int i, j;
    unsigned int crc = qIni;
    auto itr = pFrm.begin();

    for (i = 0; i < pFrm.size(); i++) {
        crc ^= *itr;
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ XOR_VALUE;
            } else {
                crc = (crc >> 1);
            }
        }
        itr++;
    }
    return crc;
}

/*---------------------------------------------------------------------------*/
/** 範囲チェック

* @param[in]	入力値
* @param[in]	最小値
* @param[in]	最大値
* @return		  false:範囲外　true:範囲内
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool FirstGenModbusRTU::chkRange(int val, int min, int max) {
    bool ret = true;
    if (min > val || max < val) {
        ret = false;
    }
    return ret;
}

/*---------------------------------------------------------------------------*/
/** 4byteをintに変換

* @param[in]	レスポンスデータ
* @param[in]	読み込み位置
* @return		変換結果(int型 4byte)
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int FirstGenModbusRTU::getIntFrom4Byte(char* pFrm, int numRd) {
    int i;
    unsigned int dataU32 = 0;
    int buf;

    for (i = 0; i < 4; i++) {
        buf = (unsigned char)pFrm[(i + 3) + (numRd * 4)];
        dataU32 += buf << (24 - (8 * i));
    }
    return (int)dataU32;
}

/*---------------------------------------------------------------------------*/
/** intを1Byteに変換

* @param[in]	int 4byte
* @param[out]	std::vector<char>& pOut 1byte変換を格納
* @return		  なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void FirstGenModbusRTU::set4Byte(int num, std::vector<char>& pOut) {
    pOut.push_back((unsigned char)((num & 0xff000000) >> 24));
    pOut.push_back((unsigned char)((num & 0x00ff0000) >> 16));
    pOut.push_back((unsigned char)((num & 0x0000ff00) >> 8));
    pOut.push_back((unsigned char)((num & 0x000000ff) >> 0));
}

/*---------------------------------------------------------------------------*/
/** クエリ(0x03)の生成

* @param[in]	int query_num
* @param[in]	int id
* @param[in]	int addr
* @param[in]	int num
* @param[in]	std::vector<std::vector<char> >& pOut
* @return		  クエリ数
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int FirstGenModbusRTU::setQuery03h(int query_num, int id, int addr, int num,
                                   std::vector<std::vector<char> >& pOut) {
    int ret;

    pOut[query_num].push_back(id);
    pOut[query_num].push_back(FUNCTION_CODE_READ);
    pOut[query_num].push_back((char)(0xff & (addr >> 8)));
    pOut[query_num].push_back((char)(0xff & (addr >> 0)));
    pOut[query_num].push_back((char)(0xff & (num * 2 >> 8)));
    pOut[query_num].push_back((char)(0xff & (num * 2 >> 0)));

    ret = chkFrm(INIT_VALUE, (std::vector<unsigned char>&)pOut[query_num]);
    pOut[query_num].push_back((char)ret);
    pOut[query_num].push_back(ret >> 8);

    return ++query_num;
}

/*---------------------------------------------------------------------------*/
/** クエリ(0x010)の生成

* @param[in]	int query_num
* @param[in]	int id
* @param[in]	int addr
* @param[in]	int num
* @param[in]	int *pVal
* @param[in]	std::vector<std::vector<char> >& pOut
* @return		  クエリ数
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int FirstGenModbusRTU::setQuery10h(int query_num, int id, int addr, int num,
                                   int* pVal,
                                   std::vector<std::vector<char> >& pOut) {
    int ret;
    int val;

    pOut[query_num].push_back(id);
    pOut[query_num].push_back(FUNCTION_CODE_WRITE);
    pOut[query_num].push_back((char)(0xff & (addr >> 8)));
    pOut[query_num].push_back((char)(0xff & (addr >> 0)));
    pOut[query_num].push_back((char)(0xff & (num * 2 >> 8)));
    pOut[query_num].push_back((char)(0xff & (num * 2 >> 0)));
    pOut[query_num].push_back(num * 4);

    for (int i = 0; i < num; i++) {
        val = *pVal;
        set4Byte(val, pOut[query_num]);
        pVal++;
    }

    ret = chkFrm(INIT_VALUE, (std::vector<unsigned char>&)pOut[query_num]);
    pOut[query_num].push_back((char)ret);
    pOut[query_num].push_back(ret >> 8);

    return ++query_num;
}

/*---------------------------------------------------------------------------*/
/** アドレスチェック(インターフェイス)

* @param[in]	レジスタアドレス
* @return     false:範囲外　true:範囲内
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool FirstGenModbusRTU::chkAddress(int addr) {
    return chkRange(addr, MIN_ADDRESS, MAX_ADDRESS);
}

/*---------------------------------------------------------------------------*/
/** 読み込み、書き込み数チェック(インターフェイス)

* @param[in]	データ数
* @return		  false:範囲外　true:範囲内
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool FirstGenModbusRTU::chkDataNum(int num) {
    return chkRange(num, MIN_DATA_NUM, MAX_DATA_NUM);
}

/*---------------------------------------------------------------------------*/
/** 最大アドレスの取得(インターフェイス)

* @param[in]  なし
* @return		  最大アドレス
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int FirstGenModbusRTU::getMaxAddress(void) { return MAX_ADDRESS; }

/*---------------------------------------------------------------------------*/
/** 最大データ数の取得(インターフェイス)

* @param[in]  なし
* @return		  最大データ数
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int FirstGenModbusRTU::getMaxDataNum(void) { return MAX_DATA_NUM; }

/*---------------------------------------------------------------------------*/
/** 最大アドレスの設定(インターフェイス)

* @param[in]	アドレス
* @return		  なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void FirstGenModbusRTU::setMaxAddress(int addr) {}

/*---------------------------------------------------------------------------*/
/** 最大データ数の設定(インターフェイス)

* @param[in]	データ数
* @return		  なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void FirstGenModbusRTU::setMaxDataNum(int num) {}

/*---------------------------------------------------------------------------*/
/** インターフェイス

* @param[in]		char *pFrm レスポンスデータ
* @param[in]		int tumRd 読み込み位置
* @return			  変換結果(int型 4byte)
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int FirstGenModbusRTU::convertResponse(char* pFrm, int numRd) {
    return getIntFrom4Byte(pFrm, numRd);
}

/*---------------------------------------------------------------------------*/
/** インターフェイス

* @param[in]		int id スレーブID
* @param[in]		int addr
* @param[in]		int num
* @param[in]		vector<vector<char> >& pOut
* @return			  なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void FirstGenModbusRTU::setRead(int id, int addr, int num,
                                std::vector<std::vector<char> >& pOut) {
    int query_num = 0;
    setQuery03h(query_num, id, addr, num, pOut);
}

/*---------------------------------------------------------------------------*/
/** インターフェイス

* @param[in]		int id スレーブID
* @param[in]		int read_addr
* @param[in]		int write_addr
* @param[in]		int read_num
* @param[in]		int write_num
* @param[in]		int *pVal
* @param[in]		vector<vector<char> >& pOut
* @param[in]		int *resLen
* @return			　クエリ数
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int FirstGenModbusRTU::setReadAndWrite(int id, int read_addr, int write_addr,
                                       int read_num, int write_num, int* pVal,
                                       std::vector<std::vector<char> >& pOut,
                                       int* resLen) {
    int query_num = 0;
    query_num = setQuery10h(query_num, id, write_addr, write_num, pVal, pOut);
    resLen[0] = 8;
    query_num = setQuery03h(query_num, id, read_addr, read_num, pOut);
    resLen[1] = 5 + read_num * 4;
    return query_num;
}

/*---------------------------------------------------------------------------*/
/** インターフェイス

* @param[in]		int id スレーブID
* @param[in]		int addr
* @param[in]		int num
* @param[in]		int *pVal
* @param[in]		vector<vector<char> >& pOut
* @return			　なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void FirstGenModbusRTU::setWrite(int id, int addr, int num, int* pVal,
                                 std::vector<std::vector<char> >& pOut) {
    int query_num = 0;
    setQuery10h(query_num, id, addr, num, pVal, pOut);
}

/*---------------------------------------------------------------------------*/
/**
* インターフェイス
* ID Share対応　レスポンスのエラーチェックが正しいか判定する。
* @param[in]		int num
* @return			　なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void FirstGenModbusRTU::chkCrc(int qIni, unsigned char* pRes, int len) {
    std::vector<unsigned char> vector_res(len, 0);
    for (int i = 0; i < len; i++) {
        vector_res[i] = pRes[i];
    }
    chkFrm(qIni, vector_res);
}

}  // namespace om_modbusRTU_node