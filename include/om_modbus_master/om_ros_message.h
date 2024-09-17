#ifndef OM_ROS_MESSAGE_H
#define OM_ROS_MESSAGE_H

#include "ros/ros.h"
#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_state.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_base.h"
#include "om_modbus_master/ISetResponse.h"
#include "om_modbus_master/ICheckData.h"
#include "om_modbus_master/ICheckIdShareMode.h"

using std::string;

namespace om_modbusRTU_node {
class Base;
class FirstGenModbusRTU;

/*---------------------------------------------------------------------------*/
/**
* @brief メッセージデータのクラス
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
class RosMessage : public ISetResponse {
   private:
    Base *base_obj_;
    ICheckData **check_data_;
    ICheckIdShareMode *check_idshare_mode_;
    ros::Publisher response_pub_;
    ros::Publisher state_pub_;
    ros::Subscriber query_sub_;
    om_modbus_master::om_response response_msg_;
    om_modbus_master::om_state state_msg_;
    ros::Timer timer_;

    bool isCommEnabled_;
    int topic_id_;
    int topic_update_rate_;

    const static int BUF_SIZE = 1;
    const static int MIN_UPDATE_RATE = 0;
    const static int MAX_UPDATE_RATE = 1000;
    const static int MAX_TOPIC_ID = 15;
    const static int MIN_TOPIC_ID = 0;
    const static int MIN_SLAVE_ID = 0;
    const static int MAX_SLAVE_ID = 31;

    const static int FUNCTION_CODE_READ = 0;
    const static int FUNCTION_CODE_WRITE = 1;
    const static int FUNCTION_CODE_READ_WRITE = 2;

    const static int STATE_DRIVER_NONE = 0;
    const static int STATE_DRIVER_COMM = 1;
    const static int STATE_MES_REACH = 1;
    const static int STATE_MES_ERROR = 2;
    const static int STATE_ERROR_NONE = 0;

    const static int SLAVE_ID_ERROR = -1;
    const static int FUNCTION_CODE_ERROR = -2;
    const static int ADDRESS_ERROR = -3;
    const static int DATA_ERROR = -4;
    const static int GLOBAL_ID_ERROR = -5;
    const static int AXIS_NUM_ERROR = -6;
    const static int CRC_ERROR = -7;
    const static int MODBUS_MESSAGE_LENGTH_ERROR = -8;

    const static int MIN_GLOBAL_ID = 1;    // ID Share対応
    const static int MAX_GLOBAL_ID = 127;  // ID Share対応

    void chkAddress(int addr, int id);
    void chkDataNum(int num, int id);
    bool chkRange(int val, int min, int max);
    void chkRangeOfData(const om_modbus_master::om_query msg);
    void chkReadSlaveID(int id);
    void chkWriteSlaveID(int id);
    void dispErrorMessage(int error);
    string makeTopicName(int num, string name);
    void queryCallback(const om_modbus_master::om_query msg);
    void timerCallback(const ros::TimerEvent &);

    // ID Share対応で追加
    void chkRangeOfDataIdShare(const om_modbus_master::om_query msg);
    bool chkIdShareMode(const om_modbus_master::om_query msg);
    void flowCtrlIdShare(const om_modbus_master::om_query msg);
    bool chkGlobalIdRange(int id);

   public:
    RosMessage();
    ~RosMessage();
    void init(Base *pObj, ICheckData *check_obj[],
              ICheckIdShareMode *check_idshare_obj);
    void setResponse(const om_modbus_master::om_response res) override;
    void setState(const om_modbus_master::om_state state) override;
    void setCommEnabled(bool val) override;
};
}  // namespace om_modbusRTU_node
#endif
