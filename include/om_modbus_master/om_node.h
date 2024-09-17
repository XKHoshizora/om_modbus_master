#ifndef OM_NODE_H
#define OM_NODE_H

using std::string;

namespace om_modbusRTU_node {
class RosMessage;
class Base;
class ICheckData;
class IConvertQueryAndResponse;
class ICheckIdShareMode;

void init(std::vector<int> &first, std::vector<int> &second, Base *base_obj,
          ICheckData *check_obj[], IConvertQueryAndResponse *convert_obj[],
          ICheckIdShareMode *check_idshare_obj, RosMessage *rosmes_obj);
void finalize(std::vector<int> &first, std::vector<int> &second, Base *base_obj,
              ICheckData *check_obj[], IConvertQueryAndResponse *convert_obj[],
              RosMessage *rosmes_obj);
void bgMain(void);
void deleteSpace(string &str);
std::vector<int> split(string &src, char delim);
void chkComma(string &str);
void chkIdShareParameter(int global_id, int axis_num);

const static int MAX_SLAVE_NUM = 31;
const static int MIN_SLAVE_ID = 1;
const static int MAX_SLAVE_ID = 31;

// ID Shareモード関係のパラメータの仕様値
const static int UNUSE_GLOBAL_ID = -1;  // ID Shareモードを使用しないときの値
const static int MIN_GLOBAL_ID = 1;  // MEXE02のShare Control Global IDと同義
const static int MAX_GLOBAL_ID = 127;
const static int MIN_AXIS_NUM = 1;  // 軸数
const static int MAX_AXIS_NUM = 31;  // 31はModbusで設定できる軸数の最大値
}  // namespace om_modbusRTU_node
#endif
