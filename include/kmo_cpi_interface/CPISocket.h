// Definition of the ClientNavSocket class

#ifndef CPISocket_class
#define CPISocket_class

#include "Socket.h"
#include <vector>

typedef unsigned char BYTE;
typedef unsigned short int WORD;
typedef signed short int SWORD;
typedef unsigned int DWORD;
typedef signed int SDWORD;
typedef unsigned long LONG;
typedef signed long SLONG;

//! Inline function to convert WORD to BYTE using low byte first (Intel format).
inline void convertWORD_to_2BYTES(WORD w, BYTE* b)
{
  b[1] = (w >> 8) & 0xFF;
  b[0] =  w       & 0xFF;
}

inline void convertSWORD_to_2BYTES(SWORD sw, BYTE* b)
{
  b[1] = (sw >> 8) & 0xFF;
  b[0] =  sw       & 0xFF;
}

inline void convertDWORD_to_4BYTES(DWORD d, BYTE* b)
{
  b[3] = (d >> 24) & 0xFF;
  b[2] = (d >> 16) & 0xFF;
  b[1] = (d >> 8) & 0xFF;
  b[0] =  d       & 0xFF;
}

inline void convertSDWORD_to_4BYTES(SDWORD sd, BYTE* b)
{
  b[3] = (sd >> 24) & 0xFF;
  b[2] = (sd >> 16) & 0xFF;
  b[1] = (sd >> 8) & 0xFF;
  b[0] =  sd       & 0xFF;
}

inline void convert2BYTES_to_WORD(BYTE* b, WORD &w)
{
  w = *(WORD*)b;
}

inline void convert2BYTES_to_SWORD(BYTE* b, SWORD &sw)
{
  sw = *(SWORD*)b;
}

inline void convert4BYTES_to_DWORD(BYTE* b, DWORD &d)
{
  d = *(DWORD*)b;
}

inline void convert4BYTES_to_SWORD(BYTE* b, SDWORD &sd)
{
  sd = *(SDWORD*)b;
}



/////////////////////////////////////////////////////

class CPISocket : public Socket
{
 public:

  CPISocket () {};
  CPISocket ( std::string host, int port );
  virtual ~CPISocket(){};
  virtual void reconnect();

  void setParams(std::string host, int port );

  void sendGetRequest(std::string &reqTag, std::string &itemTag) ;
  void sendSetRequest(std::string &reqTag, std::string &itemTag) ;
  void sendSubscribeRequest() ;
  void sendUnsubscribeRequest() ;
  void sendLocalOrderRequest() ;
  void sendRequest(std::string &req) ;

  //!
  void receiveResponse(std::string &str) ; 
  void receiveGetResponse(std::string &reqTagName, std::string &itemTag) ;
  void receiveSetResponse(std::string &reqTag) ;
  void receiveSubscribeResponse(std::string &str) ;
  void receiveUnsubscribeResponse(std::string &str) ;
	  
  std::string extractValueFromItemTagResponse(const std::string &itemTag);
	  
  const CPISocket& operator << ( const std::string& ) ;
  const CPISocket& operator >> ( std::string& ) ;


  //
  void sendOperationRequest(int ptID, int opCode, 
			    int opParam1, int opParam2);	 

  void sendGotoPointRequest(int ptID);
  void sendLocalOrderMode();

  //
  void Set_SlowDown(bool slowDown);
  void Set_StopMoving(bool stop);
  bool Get_Waiting4NewOrder();
  bool Get_HasExecDriveCmd();
  bool Get_MovingStatus();
  int  Get_CurrentSegment();

  void liftForksBlocking(double delay);
  void lowerForksBlocking(double delay);
  bool getEStopFlag();
  bool getStopFlag();

  bool getBTForkLeft();
  bool getBTForkRight();
  bool getBTForkCenter();
  double getBTForkHeight();

  void setDebugFlag() { debug_ = true; }
private:
  void setLiftForksFlag(bool flag);
  void setLowerForksFlag(bool flag);

  bool debug_;
};


#endif					
