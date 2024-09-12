#ifndef ISET_RESPONSE_H
#define ISET_RESPONSE_H

namespace om_modbusRTU_node
{

/*---------------------------------------------------------------------------*/
/**
@brief レスポンス設定インターフェース
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
class ISetResponse
{
public:
  virtual void setResponse(const om_modbus_master::om_response res)=0;
  virtual void setState(const om_modbus_master::om_state state)=0;
  virtual void setCommEnabled(bool val)=0;
  virtual ~ISetResponse(){};
};

}
#endif
