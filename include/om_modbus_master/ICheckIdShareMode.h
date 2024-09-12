#ifndef ICHECK_IDSHARE_MODE_H
#define ICHECK_IDSHARE_MODE_H

namespace om_modbusRTU_node
{

/*---------------------------------------------------------------------------*/
/** 
* @brief ID Shareモードで指定するパラメータの範囲チェックを行うインターフェース
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
class ICheckIdShareMode
{
public:
  virtual void chkRangeReadData(int num)=0;
  virtual void chkRangeAxis(int num)=0;
  virtual void chkRangeWriteData(int num)=0;
  virtual void chkRangeGlobalId(int num)=0;
  virtual void chkRangeShareReadAddress(int addr)=0;
  virtual void chkRangeShareWriteAddress(int addr)=0;
  virtual void chkDataNumWriteQueryIdShareMode(const int write_data_num)=0;
  virtual void chkDataNumReadWriteQueryIdShareMode(const int write_data_num)=0;
  ~ICheckIdShareMode(){};
};

}

#endif
