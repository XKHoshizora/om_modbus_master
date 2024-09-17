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
* @file	om_second_gen.cpp
* @brief Modbus RTUのクエリを生成する(第2世代)
* @details
* @attention
* @note

履歴 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
* @version  Ver.1.05 April.5.2022 K.Yamaguchi
    - ID Shareモード対応

* @version	Ver.1.00 Mar.11.2019 T.Takahashi
                         - 新規作成
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
#include "om_modbus_master/om_second_gen.h"
#include "om_modbus_master/om_idshare_mode.h"

namespace om_modbusRTU_node {

SecondGenModbusRTU::SecondGenModbusRTU() { idshare_mode_ = new IdShareMode(); }

SecondGenModbusRTU::~SecondGenModbusRTU() { delete idshare_mode_; }

/*---------------------------------------------------------------------------*/
/** アドレスチェック
* @param[in]	レジスタアドレス
* @return		  false:範囲外　true:範囲内
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool SecondGenModbusRTU::chkAddress(int addr) {
    // idshareモードかどうかで分岐させたい
    return chkRange(addr, MIN_ADDRESS, MAX_ADDRESS);
}

/*---------------------------------------------------------------------------*/
/** 読み込み、書き込み数チェック

* @param[in]	データ数
* @return		  false:範囲外　true:範囲内
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool SecondGenModbusRTU::chkDataNum(int num) {
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
int SecondGenModbusRTU::getMaxAddress(void) { return MAX_ADDRESS; }

/*---------------------------------------------------------------------------*/
/** 最大データ数の取得(インターフェイス)

* @param[in]  なし
* @return		  最大データ数
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int SecondGenModbusRTU::getMaxDataNum(void) { return MAX_DATA_NUM; }

/*---------------------------------------------------------------------------*/
/** 書き込み、読み込みデータ設定(0x17) 0x17のクエリを設定

* @param[in]	int id スレーブID
* @param[in]	int read_addr	読み出し起点となるレジスタアドレス
* @param[in]	int write_addr	書き込み起点となるレジスタアドレス
* @param[in]	int read_num	読み出すレジスタの数
* @param[in]	int write_num	書き込むレジスタの数
* @param[int]	int *pVal	書き込みデータ
* @param[out]	vector<vector<char> >& pOut	送信データ
* @return		int クエリ数
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int SecondGenModbusRTU::setQuery17h(int id, int read_addr, int write_addr,
                                    int read_num, int write_num, int *pVal,
                                    std::vector<std::vector<char> > &pOut) {
    int ret;
    int val;
    int query_num = 0;

    pOut[query_num].push_back(id);
    pOut[query_num].push_back(FUNCTION_CODE_READ_WRITE);
    pOut[query_num].push_back((char)(0xff & (read_addr >> 8)));
    pOut[query_num].push_back((char)(0xff & (read_addr >> 0)));
    pOut[query_num].push_back((char)(0xff & (read_num * 2 >> 8)));
    pOut[query_num].push_back((char)(0xff & (read_num * 2 >> 0)));
    pOut[query_num].push_back((char)(0xff & (write_addr >> 8)));
    pOut[query_num].push_back((char)(0xff & (write_addr >> 0)));
    pOut[query_num].push_back((char)(0xff & (write_num * 2 >> 8)));
    pOut[query_num].push_back((char)(0xff & (write_num * 2 >> 0)));
    pOut[query_num].push_back(write_num * 4);

    for (int i = 0; i < write_num; i++) {
        val = *pVal;
        set4Byte(val, pOut[query_num]);
        pVal++;
    }

    ret = chkFrm(INIT_VALUE, (std::vector<unsigned char> &)pOut[query_num]);
    pOut[query_num].push_back((char)ret);
    pOut[query_num].push_back(ret >> 8);
    return ++query_num;
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* インターフェイス
*
* @param[in]		int id スレーブID or global_id
* @param[in]		int addr レジスタアドレス(ID ShareモードではShare Write
dataのアドレス)
* @param[in]		int num レジスタ数
* @param[in]		vector<vector<char> >& pOut
* @return			  なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void SecondGenModbusRTU::setRead(int id, int addr, int num,
                                 std::vector<std::vector<char> > &pOut) {
    int query_num = 0;
    int global_id = idshare_mode_->getParameter("global_id");
    id_ = id;
    read_num_ = num;

    if (idshare_mode_->isIdShareMode(global_id, id))
    // ID Shareモードのときの処理
    {
        idshare_mode_->setQuery03h(query_num, id, addr, num, pOut);
    } else
    // Normalモードのときの処理
    {
        setQuery03h(query_num, id, addr, num, pOut);
    }
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* インターフェイス
* @param[in]		int id スレーブID
* @param[in]		int addr レジスタアドレス(ID ShareモードではShare Write
dataのアドレス)
* @param[in]		int num レジスタ数
* @param[in]		int *pVal
* @param[in]		vector<vector<char> >& pOut
* @return			　なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void SecondGenModbusRTU::setWrite(int id, int addr, int num, int *pVal,
                                  std::vector<std::vector<char> > &pOut) {
    int query_num = 0;
    int global_id = idshare_mode_->getParameter("global_id");
    id_ = id;

    if (idshare_mode_->isIdShareMode(global_id, id))
    // ID Shareモードのときの処理
    {
        idshare_mode_->setQuery10h(query_num, id, addr, num, pVal, pOut);
    } else
    // Normalモードのときの処理
    {
        setQuery10h(query_num, id, addr, num, pVal, pOut);
    }
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* 書き込み、読み込みデータ設定(インターフェイス)
* @param[in]	int id スレーブID
* @param[in]	int read_addr	読み出し起点となるレジスタアドレス
* @param[in]	int write_addr	書き込み起点となるレジスタアドレス
* @param[in]	int read_num	読み出すレジスタの数
* @param[in]	int write_num	書き込むレジスタの数
* @param[in]	int *pVal	書き込みデータ
* @param[out]	vector<vector<char> >& pOut	送信データ
* @param[out]	int *resLen 受信数
* @return		int クエリ数
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int SecondGenModbusRTU::setReadAndWrite(int id, int read_addr, int write_addr,
                                        int read_num, int write_num, int *pVal,
                                        std::vector<std::vector<char> > &pOut,
                                        int *resLen) {
    int global_id = idshare_mode_->getParameter("global_id");
    id_ = id;
    read_num_ = read_num;

    if (idshare_mode_->isIdShareMode(global_id, id))
    // ID Shareモードのときの処理
    {
        int result = idshare_mode_->setQuery17h(
            id, read_addr, write_addr, read_num, write_num, pVal, pOut);
        resLen[0] = pOut[0].size();
        return result;
    } else
    // Normalモードのときの処理
    {
        int result = setQuery17h(id, read_addr, write_addr, read_num, write_num,
                                 pVal, pOut);
        resLen[0] = pOut[0].size();
        return result;
    }
}

/*---------------------------------------------------------------------------*/
/** インターフェイス
* @param[in]		char *pFrm レスポンスデータ
* @param[in]		int tumRd 読み込み位置
* @return			  変換結果(int型 4byte)
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int SecondGenModbusRTU::convertResponse(char *pFrm, int numRd) {
    int global_id = idshare_mode_->getParameter("global_id");
    if (idshare_mode_->isIdShareMode(
            global_id,
            id_))  // インターフェイスの引数にidがないのでクラスメンバにid_を追加して、見てる

    // ID Shareモードのときの処理
    {
        int axis_num = idshare_mode_->getParameter("axis_num");

        std::array<char, 255> array_frm;
        for (int i = 0; i < 255; i++) {
            array_frm[i] = pFrm[i];
        }

        std::array<char, 255> del_error_check_rxd_data =
            deleteAxisErc(array_frm, axis_num);

        return getIntFrom4Byte(del_error_check_rxd_data.data(), numRd);
    } else
    // Normalモードのときの処理
    {
        return getIntFrom4Byte(pFrm, numRd);
    }
}

/*---------------------------------------------------------------------------*/
/**
* インターフェイス
* ID Share対応　1軸分のreadデータ数が仕様内か判定する。
* 例外を返す。
* @param[in]		int num
* @return			　なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void SecondGenModbusRTU::chkRangeReadData(int num) {
    idshare_mode_->chkRangeReadData(num);
}

/*---------------------------------------------------------------------------*/
/**
* インターフェイス
* ID Share対応　指定軸数が仕様内か判定する。
* 例外を返す。
* @param[in]		int num
* @return			　なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void SecondGenModbusRTU::chkRangeAxis(int num) {
    idshare_mode_->chkRangeAxis(num);
}

/*---------------------------------------------------------------------------*/
/**
* インターフェイス
* ID Share対応　1軸分のwriteデータ数が仕様内か判定する。
* @param[in]		int num
* @return			　なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void SecondGenModbusRTU::chkRangeWriteData(int num) {
    idshare_mode_->chkRangeWriteData(num);
}

/*---------------------------------------------------------------------------*/
/**
* インターフェイス
* ID Share対応　msg.slave_id(ID
Shareモード時なのでglobal_id)が仕様内か判定する。
* @param[in]		int num: global_id
* @return			　なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void SecondGenModbusRTU::chkRangeGlobalId(int num) {
    idshare_mode_->chkRangeGlobalId(num);
}

/*---------------------------------------------------------------------------*/
/**
* インターフェイス
* ID Share対応　ID Shareの読み出しアドレスが仕様内か判定する。
* @param[in]		int num
* @return			　なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void SecondGenModbusRTU::chkRangeShareReadAddress(int addr) {
    idshare_mode_->chkRangeShareReadAddress(addr);
}

/*---------------------------------------------------------------------------*/
/**
* インターフェイス
* ID Share対応　ID Shareの書き込みアドレスが仕様内か判定する。
* @param[in]		int num
* @return			　なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void SecondGenModbusRTU::chkRangeShareWriteAddress(int addr) {
    idshare_mode_->chkRangeShareWriteAddress(addr);
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* レスポンスから軸間エラーチェックを削除する。
* @param[in]		std::array<char, 255> rxd_data: レスポンスデータ  int
axis_num: 軸数
* @return			　std::array<char, 255>
軸間エラーチェックが削除されたレスポンスデータ
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
std::array<char, 255> SecondGenModbusRTU::deleteAxisErc(
    std::array<char, 255> rxd_data, int axis_num) {
    // 軸間エラーチェックを削除してresponse_msg_にいれる場合
    std::array<char, 255> del_error_check_rxd_data{0};
    int one_axis_read_num_byte =
        read_num_ / axis_num * 4;  // 1軸分のデータのbyte

    std::vector<int> skip_index;
    int byte_index;
    for (int axis = axis_num; axis > 0; axis--) {
        byte_index =
            3 + one_axis_read_num_byte * axis +
            (axis - 1) * 2;  //(axis-1)*2: 軸間チェック分 降順になっている
        skip_index.push_back(byte_index);  // 降順になっているため
                                           // 軸間エラーチェックは2byte/軸のため
        skip_index.push_back(byte_index + 1);
    }

    // rxd_data軸間エラーチェックを削除する
    int skip_count = 0;
    for (int i = 0; i < 255; i++) {
        auto result = std::find(skip_index.begin(), skip_index.end(), i);
        // 現在のindexが軸間エラーチェックの要素番号のとき
        if (result != skip_index.end()) {
            skip_count++;
            continue;
        } else {
            del_error_check_rxd_data[i - skip_count] = rxd_data[i];
        }
    }

    return del_error_check_rxd_data;
}

/*---------------------------------------------------------------------------*/
/**
* インターフェイス
* ID Share対応　レスポンスのエラーチェックが正しいか判定する。
* @param[in]		int num
* @return			　なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void SecondGenModbusRTU::chkCrc(int qIni, unsigned char *pRes, int len) {
    int global_id = idshare_mode_->getParameter("global_id");
    bool is_idshare_mode = idshare_mode_->isIdShareMode(global_id, id_);

    chkCrcContents(qIni, pRes, len, is_idshare_mode);
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* chkCrc関数の中身
* @param[in]		int num
* @return			　なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void SecondGenModbusRTU::chkCrcContents(int qIni, unsigned char *pRes, int len,
                                        bool is_idshare_mode) {
    int crc_result = 0x0000;  // crcの計算結果
    std::vector<unsigned char> vector_crc_target(len - 2, 0);

    // CRC以外の部分を取得
    for (int i = 0; i < len - 2; i++) {
        vector_crc_target[i] = pRes[i];
    }
    // CRC部分を取得
    int crc = 0x0000;
    crc |= (pRes[len - 1] << 8);
    crc |= (pRes[len - 2]);

    // ID Shareモードのとき
    if (is_idshare_mode) {
        idshare_mode_->chkCrc(qIni, pRes, len);
    }
    // Normalモードのとき
    else {
        crc_result = FirstGenModbusRTU::chkFrm(qIni, vector_crc_target);

        if (crc_result != crc) {
            throw CRC_ERROR;
        }
    }
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* インターフェース実装
* @param[in]		const int write_data_num: クエリのデータ数(全軸の合計)
* @return			　なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void SecondGenModbusRTU::chkDataNumWriteQueryIdShareMode(
    const int write_data_num) {
    idshare_mode_->chkDataNumWriteQuery(write_data_num);
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* インターフェース実装
* @param[in]		const int write_data_num: クエリのデータ数(全軸の合計)
* @return			　なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void SecondGenModbusRTU::chkDataNumReadWriteQueryIdShareMode(
    const int write_data_num) {
    idshare_mode_->chkDataNumReadWriteQuery(write_data_num);
}

}  // namespace om_modbusRTU_node
