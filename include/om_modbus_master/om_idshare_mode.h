#ifndef OM_IDSHARE_MODE_H
#define OM_IDSHARE_MODE_H

#include <string>
#include <vector>

namespace om_modbusRTU_node
{

/*---------------------------------------------------------------------------*/
/** 
* @brief ID Shareに対応するための機能を実装するクラス
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
class IdShareMode
{
private:
  //仕様書から作成したデータ範囲
  const static int MIN_GLOBAL_ID = 1; //MEXE02のShare Control Global IDと同義
  const static int MAX_GLOBAL_ID = 127;
  const static int MIN_AXIS_NUM = 1; //軸数
  const static int MAX_AXIS_NUM = 31;
  const static int MIN_READ_NUM = 1; //1軸分 初期値は0 ID Shareモード時1~12になる
  const static int MAX_READ_NUM = 12; 
  const static int MIN_WRITE_NUM = 1; //1軸分 初期値は0 ID Shareモード時1~12になる
  const static int MAX_WRITE_NUM = 12;
  const static int MIN_SHARE_READ_ADDRES = 0x0000; //(0~11)*2まで
  const static int MAX_SHARE_READ_ADDRES = 0x0016;
  const static int MIN_SHARE_WRITE_ADDRES = 0x0000; //(0~11)*2まで
  const static int MAX_SHARE_WRITE_ADDRES = 0x0016;

  //仕様書外だがMEXE02のデータ範囲(参考)
  const static int MIN_SHARE_CONTROL_NUMBER = -1;
  const static int MAX_SHARE_CONTROL_NUMBER = 31;
  const static int MIN_SHARE_CONTROL_GLOBAL_ID = 1;
  const static int MAX_SHARE_CONTROL_GLOBAL_ID = 127;
  const static int MIN_SHARE_CONTROL_LOCAL_ID = 0;
  const static int MAX_SHARE_CONTROL_LOCAL_ID = 31;
  const static int MIN_SHARE_READ_DATA = 1;
  const static int MAX_SHARE_READ_DATA = 12;
  const static int MIN_SHARE_WRITE_DATA = 1;
  const static int MAX_SHARE_WRITE_DATA = 12;

  //ファンクションコード
  const static int FUNCTION_CODE_WRITE = 0x10; //複数の保持レジスタへの書き込み
  const static int FUNCTION_CODE_READ = 0x03; //保持レジスタからの読み出し
  const static int FUNCTION_CODE_READ_WRITE = 0x17; //複数の保持レジスタの読み出し/書き込み

  //エラーチェック初期値
  const static int ERROR_CHECK_INIT_VALUE = 0xFFFF; //CRC-16の初期値
  const static int ERROR_CHECK_XOR_VALUE = 0xA001;

  //エラーコード
  const static int SLAVE_ID_ERROR = -1;
  const static int FUNCTION_CODE_ERROR = -2;
  const static int ADDRESS_ERROR = -3;
  const static int DATA_ERROR = -4;
  const static int GLOBAL_ID_ERROR = -5;
  const static int AXIS_NUM_ERROR = -6;
  const static int CRC_ERROR = -7; //レスポンスのCRCが不一致
  const static int MODBUS_MESSAGE_LENGTH_ERROR = -8; // Modbusメッセージ長が仕様範囲外

  // Modbusメッセージ長256Byte以下にするための最大データ数
  const static int MAX_DATA_NUM_WRITE = 61;
  const static int MAX_DATA_NUM_READ_WRITE = 60;

  bool is_idshare_mode_; //現在ID Shareモードか
  
public:
  IdShareMode();
  ~IdShareMode();
  bool chkIdShareMode(int global_id, int slave_id);
  bool isIdShareMode(int global_id, int slave_id);
  void setIdShareMode(int global_id, int slave_id);
  bool chkRange(int val, int min, int max);
  bool chkRangeReadData(int read_num);
  bool chkRangeWriteData(int write_num);
  bool chkRangeAxis(int axis_num);
  bool chkRangeGlobalId(int global_id);
  bool chkRangeShareReadAddress(int read_addr);
  bool chkRangeShareWriteAddress(int write_addr);
  int setQuery03h(int query_num, int id, int addr, int num, std::vector<std::vector<char> >& pOut);
  int setQuery10h(int query_num, int id, int addr, int num, int *pVal, std::vector<std::vector<char> >& pOut);
  int setQuery17h(int id, int read_addr, int write_addr, int read_num, int write_num, int *pVal, std::vector<std::vector<char> >& pOut);
  void set4Byte(int num, std::vector<char>& pOut);
  void dispQuery(std::vector<std::vector<char> >& q);
  int getParameter(std::string param);
  int chkFrm(int init, std::vector<unsigned char>& pFrm);
  void chkCrc(int qIni, unsigned char *pRes, int len);
  void chkCrcContents(int qIni, unsigned char *pRes, int len, int axis_num);
  void chkDataNumWriteQuery(const int write_data_num);
  void chkDataNumReadWriteQuery(const int write_data_num);
};


}

#endif
