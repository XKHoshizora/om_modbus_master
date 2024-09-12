#ifndef ICONVERT_QUERY_AND_RESPONSE_H
#define ICONVERT_QUERY_AND_RESPONSE_H

namespace om_modbusRTU_node
{

/*---------------------------------------------------------------------------*/
/**
@brief クエリとレスポンスのデータ変換用インターフェース
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
class IConvertQueryAndResponse
{

public:
  virtual void setRead(int id, int addr, int num, std::vector<std::vector<char> >& pOut)=0;
  virtual void setWrite(int id, int addr, int num, int *pVal, std::vector<std::vector<char> >& pOut)=0;
  virtual int setReadAndWrite(int id, int read_addr, int write_addr, int read_num, int write_num, int *pVal, std::vector<std::vector<char> >& pOut, int *resLen)=0;
  virtual int convertResponse(char *pFrm, int numRd)=0;
  virtual ~IConvertQueryAndResponse(){};
};

}

#endif
