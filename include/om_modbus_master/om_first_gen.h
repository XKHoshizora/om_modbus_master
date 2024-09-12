#ifndef OM_FIRST_GEN_H
#define OM_FIRST_GEN_H

#include "om_modbus_master/om_base.h"
#include "om_modbus_master/IConvertQueryAndResponse.h"
#include "om_modbus_master/ICheckData.h"
#include "om_modbus_master/om_idshare_mode.h" //ID Share対応で追加

namespace om_modbusRTU_node
{
class Base;

/*---------------------------------------------------------------------------*/
/** 
@brief 第一世代の機能を実装するクラス
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
class FirstGenModbusRTU : public IConvertQueryAndResponse, public ICheckData
{
private:

public:
  const static int INIT_VALUE = 0xFFFF;
  const static int XOR_VALUE = 0xA001;
  const static int FUNCTION_CODE_WRITE = 0x10;
  const static int FUNCTION_CODE_READ = 0x03;
  const static int FUNCTION_CODE_READ_WRITE = 0x17;
  const static int MIN_DATA_NUM = 1;
  const static int MAX_DATA_NUM = 8;
  const static int MIN_ADDRESS = 0x0000;
  const static int MAX_ADDRESS = 0x1FFF;
  
  FirstGenModbusRTU();
  ~FirstGenModbusRTU();
  int chkFrm(int qIni, std::vector<unsigned char>& pFrm);
  bool chkRange(int val, int min, int max);
  int getIntFrom4Byte(char *pFrm, int numRd);
  void set4Byte(int buf, std::vector<char>& pOut);
  int setQuery03h(int query_num, int id, int addr, int num, std::vector<std::vector<char> >& pOut);
  int setQuery10h(int query_num, int id, int addr, int num, int *pVal, std::vector<std::vector<char> >& pOut);
  bool chkAddress(int addr) override;
  bool chkDataNum(int num) override;
  void setMaxAddress(int addr) override;
  void setMaxDataNum(int num) override;
  int getMaxAddress(void) override;
  int getMaxDataNum(void) override;
  int convertResponse(char *pFrm, int numRd) override;
  void setRead(int id, int addr, int num, std::vector<std::vector<char> >& pOut) override;
  int setReadAndWrite(int id, int read_addr, int write_addr, int read_num, int write_num, int *pVal, std::vector<std::vector<char> >& pOut, int *resLen) override;
  void setWrite(int id, int addr, int num, int *pVal, std::vector<std::vector<char> >& pOut) override;
  void chkCrc(int qIni, unsigned char *pRes, int len) override;
};

}

#endif
