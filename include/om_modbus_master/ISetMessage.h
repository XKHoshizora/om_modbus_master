#ifndef ISET_MESSAGE_H
#define ISET_MESSAGE_H

namespace om_modbusRTU_node
{
struct QUERY_DATA_T;

/*---------------------------------------------------------------------------*/
/**
@brief メッセージ設定インターフェース
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
class ISetMessage
{
public:
  virtual void setData(const om_modbus_master::om_query mes)=0;
  virtual ~ISetMessage(){};
};

}
#endif
