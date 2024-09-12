#ifndef ICHECK_DATA_H
#define ICHECK_DATA_H

namespace om_modbusRTU_node
{

/*---------------------------------------------------------------------------*/
/**
@brief 値チェック用のインターフェース
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
class ICheckData
{
public:
  virtual bool chkAddress(int addr)=0;
  virtual bool chkDataNum(int num)=0;
  virtual void setMaxAddress(int addr)=0;
  virtual void setMaxDataNum(int num)=0;
  virtual int getMaxAddress(void)=0;
  virtual int getMaxDataNum(void)=0;
  virtual void chkCrc(int qIni, unsigned char *pRes, int len)=0;
  virtual ~ICheckData(){};
};

}
#endif
