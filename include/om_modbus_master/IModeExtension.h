#ifndef IMODE_EXTENSION_H
#define IMODE_EXTENSION_H

namespace om_modbusRTU_node
{

/*---------------------------------------------------------------------------*/
/**
@brief モード拡張用インターフェース
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
class IModeExtension
{

public:
  virtual int extension(int)=0;
  virtual ~IModeExtension(){};
};

}

#endif
