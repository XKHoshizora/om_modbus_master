#ifndef OM_SECOND_GEN_H
#define OM_SECOND_GEN_H

#include "om_modbus_master/om_base.h"
#include "om_modbus_master/om_idshare_mode.h"
#include "om_modbus_master/ICheckIdShareMode.h"

namespace om_modbusRTU_node {

/*---------------------------------------------------------------------------*/
/**
* @brief 第二世代の機能を実装するクラス
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
class SecondGenModbusRTU : public FirstGenModbusRTU, public ICheckIdShareMode {
   private:
    void informIdShareMode();         // ID Share対応で追加
    void informNormalMode();          // ID Share対応で追加
    int id_;                          // ID Share対応で追加
    int read_num_;                    // ID Share対応で追加
    static const int CRC_ERROR = -7;  // レスポンスのCRCが不一致

   public:
    IdShareMode *idshare_mode_;  // ID Share対応で追加

    const static int MAX_DATA_NUM = 32;
    const static int MAX_ADDRESS = 0x57FF;
    SecondGenModbusRTU();
    ~SecondGenModbusRTU();
    bool chkAddress(int addr) override;
    bool chkDataNum(int num) override;
    int getMaxAddress(void) override;
    int getMaxDataNum(void) override;
    void setRead(int id, int addr, int num,
                 std::vector<std::vector<char> > &pOut) override;
    void setWrite(int id, int addr, int num, int *pVal,
                  std::vector<std::vector<char> > &pOut) override;
    int setQuery17h(int id, int read_addr, int write_addr, int read_num,
                    int write_num, int *pVal,
                    std::vector<std::vector<char> > &pOut);
    int setReadAndWrite(int id, int read_addr, int write_addr, int read_num,
                        int write_num, int *pVal,
                        std::vector<std::vector<char> > &pOut,
                        int *resLen) override;
    int convertResponse(char *pFrm, int numRd) override;
    void chkRangeReadData(int num) override;
    void chkRangeAxis(int num) override;
    void chkRangeWriteData(int num) override;
    void chkRangeGlobalId(int num) override;
    void chkRangeShareReadAddress(int addr) override;
    void chkRangeShareWriteAddress(int addr) override;
    std::array<char, 255> deleteAxisErc(std::array<char, 255> rxd_data,
                                        int axis_num);
    void chkCrc(int qIni, unsigned char *pRes, int len) override;
    void chkCrcContents(int qIni, unsigned char *pRes, int len,
                        bool is_idshare_mode);
    void chkDataNumWriteQueryIdShareMode(const int write_data_num);
    void chkDataNumReadWriteQueryIdShareMode(const int write_data_num);
};

}  // namespace om_modbusRTU_node

#endif
