#ifndef OM_BROADCAST_H
#define OM_BROADCAST_H

#include "om_modbus_master/om_base.h"

namespace om_modbusRTU_node
{

/*---------------------------------------------------------------------------*/
/** 
@brief ブロードキャスト送信の機能を実装するクラス
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
class BroadcastModbusRTU :  public FirstGenModbusRTU
{
private:
  int max_address_;
  int max_data_num_;
  
public:
  const static int MAX_DATA_NUM_MODBUS_RTU = 60;
  const static int MAX_ADDRESS_MODBUS_RTU = 0xFFFF;
  
  BroadcastModbusRTU();
  ~BroadcastModbusRTU();
  bool chkAddress(int addr) override;
  bool chkDataNum(int num) override;
  void setMaxAddress(int addr) override;
  void setMaxDataNum(int num) override;
  
};

}

#endif
