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

/** @file	om_broadcast.cpp
@brief Modbus RTUのクエリを生成する。(ブロードキャスト)
@details
@attention
@note

履歴 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
@version	Ver.1.00 Mar.11.2019 T.Takahashi
      - 新規作成
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
#include "om_modbus_master/om_broadcast.h"

namespace om_modbusRTU_node {

BroadcastModbusRTU::BroadcastModbusRTU() {
    max_address_ = MAX_ADDRESS_MODBUS_RTU;
    max_data_num_ = MAX_DATA_NUM_MODBUS_RTU;
}

BroadcastModbusRTU::~BroadcastModbusRTU() {}

/*---------------------------------------------------------------------------*/
/** アドレスチェック

@task
@param[in]	レジスタアドレス
@return		  false:範囲外　true:範囲内
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool BroadcastModbusRTU::chkAddress(int addr) {
    return chkRange(addr, MIN_ADDRESS, max_address_);
}

/*---------------------------------------------------------------------------*/
/** 読み込み、書き込み数チェック

@task
@param[in]	データ数
@return		  false:範囲外　true:範囲内
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool BroadcastModbusRTU::chkDataNum(int num) {
    return chkRange(num, MIN_DATA_NUM, max_data_num_);
}

/*---------------------------------------------------------------------------*/
/** 最大アドレスの設定

@task       最大アドレスの設定
@param[in]	レジスタアドレス
@return		  なし
@details
@attention
@note       第１世代と第2世代が混在している場合は第１世代に合わせる
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void BroadcastModbusRTU::setMaxAddress(int addr) {
    if (max_address_ > addr) {
        max_address_ = addr;
    }
}

/*---------------------------------------------------------------------------*/
/** 最大データ数の設定

@task       最大データ数の設定
@param[in]	データ数
@return		  なし
@details
@attention
@note       第１世代と第2世代が混在している場合は第１世代に合わせる
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void BroadcastModbusRTU::setMaxDataNum(int num) {
    if (max_data_num_ > num) {
        max_data_num_ = num;
    }
}

}  // namespace om_modbusRTU_node
