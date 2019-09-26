#include "kmo_cpi_interface/CPISocket.h"
#include "kmo_cpi_interface/SocketException.h"
#include <vector>
#include <iostream>
#include <sstream>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <stdio.h>

using namespace std;

//////////////////////////////////////////////////////////////////////////////////

CPISocket::CPISocket ( std::string host, int port ) : debug_(false)
{
  if ( ! Socket::create() )
    {
      throw SocketException ( "CPISocket Could not create client socket." );
    }

  if ( ! Socket::connect ( host, port ) )
    {
      throw SocketException ( "CPISocket Could not bind to port." );
    }

  this->host = host;
  this->port  = port;
  this->connected = true;
}

/////////////////////////////////////////////////////
//

void CPISocket::reconnect()
{
  this->connected = false;
  this->clean();
  if ( ! Socket::create() )
    {
      throw SocketException ( "CPISocket Could not create client socket." );
    }

  if ( ! Socket::connect ( host, port ) )
    {
      throw SocketException ( "CPISocket Could not bind to port." );
    }
   
  this->connected = true;
	
	
}

/////////////////////////////////////////////////////
//


const CPISocket& CPISocket::operator << ( const std::string& s ) 
{
  if ( ! Socket::send ( s ) )
    {
      throw SocketException ( "CPISocket Could not write to socket." );
    }

  return *this;

}

/////////////////////////////////////////////////////
//

const CPISocket& CPISocket::operator >> ( std::string& s ) 
{
  if( ! Socket::recv ( s ) )
    {
      throw SocketException ( "CPISocket Could not read from socket." );
    }

  return *this;
}


/////////////////////////////////////////////////////
//

void CPISocket::sendRequest(string &req) 
{
  string str = "<CPI2><Request>";
  str.append(req);
  str.append("</Request></CPI2>\n"); 
  
  if (debug_)
      cout << "CPISocket:: Sending Req: \n" << str << endl;
  
  if( ! Socket::send(str))
    {
      throw SocketException ( "CPISocket Could not write to socket." );
    }

}

/////////////////////////////////////////////////////
//

void CPISocket::sendGetRequest(string &reqTagName, string &itemTag) 
{
	
  string req ;
  stringstream ss (stringstream::in | stringstream::out);
  
  ss <<  "<Get Tag=" << "\"" << reqTagName <<  "\">" << itemTag << "</Get>\n" ; // \n
  
  getline(ss , req);
  
  sendRequest(req);
}

/////////////////////////////////////////////////////
//

void CPISocket::sendSetRequest(string &reqTagName, string &itemTag) 
{
	
  string req ;
  stringstream ss (stringstream::in | stringstream::out);
	
  ss <<  "<Set Tag=" << "\"" << reqTagName <<  "\">" << itemTag << "</Set>\n" ; // \n
	
  getline(ss , req);
  sendRequest(req);
}



/////////////////////////////////////////////////////
//

void CPISocket::receiveResponse(string &str) 
{
  //string str;
  if ( ! Socket::recv (str))
    {
      throw SocketException ( "Could not read from CPI socket." );
    }
    
  size_t pos;
  pos = str.find("<CPI2><Response>");    // 
  str = str.substr (pos + strlen("<CPI2><Response>")); 
  pos = str.find("</Response></CPI2>");    // 
  str = str.substr (0, pos);  
}


/////////////////////////////////////////////////////
//


void CPISocket::receiveGetResponse(string &reqTagName, string &itemTag) 
{
  string str;
  receiveResponse(str);
	
  // for debugging, we print what we have received
  // cout << "CPI(Get): receiveResponse returned " << str << endl; 
	
  size_t pos;
  pos = str.find("<Get Tag=\"");
  str = str.substr (pos + strlen("<Get Tag=\"")); 
  pos = str.find("</Get>");    
  str = str.substr (0, pos);   
    
  //----- reqTagName
	
  pos = str.find("\""); 
  reqTagName = str.substr (0,pos); 
	
  //-----itemTag
  pos = str.find(">"); //
  str = str.substr (pos + strlen(">")); 
  itemTag = str;
	
}

/////////////////////////////////////////////////////
//


string CPISocket::extractValueFromItemTagResponse(const string &itemTag)
{
  string val;
  size_t pos;
  pos = itemTag.find(">");
  val = itemTag.substr(pos + strlen(">")); 
  pos = val.find("<");
  val = val.substr (0, pos); 
  return val;
}


/////////////////////////////////////////////////////
//

void CPISocket::receiveSetResponse(string &reqTagName) 
{
  string str;
  receiveResponse(str);
	
  //cout << "receiveResponse returned " << str << endl; 
	
  size_t pos;
  pos = str.find("<Set Tag=\"");
  str = str.substr (pos + strlen("<Set Tag=\"")); 
	
  //----- reqTagName
	
  pos = str.find("\""); 
  reqTagName = str.substr (0,pos); 
}


/////////////////////////////////////////////////////
//

void  CPISocket::sendOperationRequest(int ptID = 0, int opCode = 0, 
				      int opParam1 = 0, int opParam2 = 0)

{

  char buff[128];
  sprintf(buff,"\"%d\"", ptID);
	
  string opMsg = "<Operation Path=\"LoadOperation\">";
  opMsg += "<StartLoadOperation OperationPointID=";
  opMsg += buff;
  opMsg += " OperationCode=" ;

  sprintf(buff,"\"%d\"", opCode);
  opMsg += buff ;

  sprintf(buff,"\"%d\"", opParam1);
  opMsg += " OperationParam1=" ;
  opMsg += buff;

  sprintf(buff,"\"%d\"", opParam2);
  opMsg += " OperationParam2=" ;
  opMsg += buff;

  opMsg  += " /></StartLoadOperation></Operation>";
  //
  
  
  sendRequest(opMsg);
  receiveResponse(opMsg);
  
  if (debug_)
      cout << " Recevied this: \n" << opMsg << endl; 
  
}



/////////////////////////////////////////////////////////////
//
//

void  CPISocket::sendGotoPointRequest(int ptID = 0)
				 

{
  char buff[128];
  sprintf(buff,"\"%d\"", ptID);
	
  string opMsg = "<Operation Path=\"Automatic\">";
  opMsg += "<GotoPoint PointID=";
  opMsg += buff;
  opMsg  += " ViaPoints=\"\" /></GotoPoint></Operation>";
  //
  
  
  sendRequest(opMsg);
  receiveResponse(opMsg);
  //cout << " sendGotoRequest Received this: \n" << opMsg << endl; 
}
 

////////////////////////////////////////////////////////////////
//
//

void  CPISocket::sendLocalOrderMode()

{
  
  string opMsg = "<Operation Path=\"Automatic\" ";
  opMsg += "Tag=\"Operation.OrderMode\">";
  opMsg  +=    "<OrderMode Source=\"Other\" /></Operation>";

  sendRequest(opMsg);
  receiveResponse(opMsg);
  //cout << " Recevied this: \n" << opMsg << endl; 
}


/////////////////////////////////////////////////////////////////
//
//

void CPISocket::Set_StopMoving(bool stop)
{

  string stateStr = (stop) ? "1" : "0"; 

  string reqTagName = "";
  string itemTag = "<Item tag=\"SetVehStop\" Path=\"Status.UserDefined.UniversityStop\"/>";
  itemTag += stateStr;
  itemTag += "</Item>";

  
  string respTagName, respItemTag;
  
  this->sendSetRequest(reqTagName, itemTag);

  this->receiveSetResponse(respTagName);
  

};



/////////////////////////////////////////////////////////////////
//
//

void CPISocket::Set_SlowDown(bool stop)
{

  string stateStr = (stop) ? "1" : "0"; 

  string reqTagName = "";
  string itemTag = "<Item tag=\"SetVehStop\" Path=\"Status.UserDefined.SlowDownTrigger_University\"/>";
  itemTag += stateStr;
  itemTag += "</Item>";

  string respTagName, respItemTag;
  
  this->sendSetRequest(reqTagName, itemTag);

  this->receiveSetResponse(respTagName);
  
};

///////////////////////////////////////////
bool CPISocket::Get_Waiting4NewOrder()
{
  bool status = false;
  string reqTagName = "getReq_DrivingCmd";
  string itemTag = "<Item tag=\"ExecDriveCmd\" Path=\"Status.Automatic.StatusID\"/>";
  
  string respTagName, respItemTag;
  
  this->sendGetRequest(reqTagName, itemTag);

  this->receiveGetResponse(respTagName, respItemTag);
  string val = this->extractValueFromItemTagResponse(respItemTag);
  status = (val.compare("0") == 0);
  
  return status;


};


bool CPISocket::Get_HasExecDriveCmd()
{
  bool status = false;
  string reqTagName = "getReq_DrivingCmd";
  string itemTag = "<Item tag=\"ExecDriveCmd\" Path=\"Status.Automatic.ExecDriveCommand\"/>";
  
  string respTagName, respItemTag;
  
  this->sendGetRequest(reqTagName, itemTag);

  this->receiveGetResponse(respTagName, respItemTag);
  string val = this->extractValueFromItemTagResponse(respItemTag);
  status = (val.compare("1") == 0);
  
  return status;


};

///////////////////////////////////////

bool CPISocket::Get_MovingStatus()
{

 bool status = false;
  string reqTagName = "getReqVehicleMoving";
  string itemTag = "<Item tag=\"ReqVehicleMovingStatus\" Path=\"Status.VehicleControl.Moving\"/>";
  
  string respTagName, respItemTag;
  
  this->sendGetRequest(reqTagName, itemTag);

  this->receiveGetResponse(respTagName, respItemTag);
  string val = this->extractValueFromItemTagResponse(respItemTag);
  status = (val.compare("1") == 0);
  
  return status;

};

///////////////////////////////////////

int CPISocket::Get_CurrentSegment()
{

  int segNum = 0;
  string reqTagName = "getReqSgmnt";
  string itemTag = "<Item tag=\"ReqSegmnt\" Path=\"Status.LayoutPosition.Segment\"/>";
  
  string respTagName, respItemTag;
  
  this->sendGetRequest(reqTagName, itemTag);

  this->receiveGetResponse(respTagName, respItemTag);
  string val = this->extractValueFromItemTagResponse(respItemTag);
  

  segNum = atoi(val.c_str());
  
  return segNum;

};

///////////////////////////////////////
void CPISocket::setLiftForksFlag(bool flag)
{
  string stateStr = (flag) ? "1" : "0"; 
  string reqTagName = "lift";
  string itemTag = "<Item Path=\"Status.UserDefined.Lift\">";
  itemTag += stateStr;
  itemTag += "</Item>";

  sendSetRequest(reqTagName, itemTag);
}
 

///////////////////////////////////////
void CPISocket::setLowerForksFlag(bool flag)
{
  string stateStr = (flag) ? "1" : "0"; 
  string reqTagName = "lower";
  string itemTag = "<Item Path=\"Status.UserDefined.Lower\">";
  itemTag += stateStr;
  itemTag += "</Item>";

  sendSetRequest(reqTagName, itemTag);
}
 
 ///////////////////////////////////////
void CPISocket::liftForksBlocking(double delay)
{
    setLiftForksFlag(true);
    usleep(delay*1000*1000);
    setLiftForksFlag(false);
}

 ///////////////////////////////////////
void CPISocket::lowerForksBlocking(double delay)
{
    setLowerForksFlag(true);
    usleep(delay*1000*1000);
    setLowerForksFlag(false);
}

bool CPISocket::getEStopFlag()
{
    bool status = false;
    string reqTagName = "getReq_EStop";
    
//    string itemTag = "<Item Path=\"Status.VehicleControl.Estop\"/>";
    string itemTag = "<Item Path=\"Status.SAVIE.EStop\"/>";
    
    string respTagName, respItemTag;
    
    this->sendGetRequest(reqTagName, itemTag);
    
    this->receiveGetResponse(respTagName, respItemTag);
    string val = this->extractValueFromItemTagResponse(respItemTag);
    status = (val.compare("1") == 0);
    
    return status;
}

bool CPISocket::getStopFlag()
{
    bool status = false;
    string reqTagName = "getReq_EStop";
    
//    string itemTag = "<Item Path=\"Status.VehicleControl.Estop\"/>";
    string itemTag = "<Item Path=\"Status.SAVIE.Stop\"/>";
    
    string respTagName, respItemTag;
    
    this->sendGetRequest(reqTagName, itemTag);
    
    this->receiveGetResponse(respTagName, respItemTag);
    string val = this->extractValueFromItemTagResponse(respItemTag);
    status = (val.compare("1") == 0);
    
    return status;
}
