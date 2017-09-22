#ifndef VMCNAVSERVER_HH
#define VMCNAVSERVER_HH

#include <sys/types.h>
#include <sys/socket.h>
#include "XSocket.hh"
#include <iostream>
#include <fstream>
class VMCNavserver
{
public:
     class Params
     {
     public:
	  Params()
	  {
	       host = "192.168.0.100";
	       port = 5432;
	       enc = false;
	       laser = false;
	       sick = false;
	       state = false;
	       received_logname = "";
	       sent_logname = "";
	       protocol_ver = 8;
	       epu = false;
	       protocol2Hack = true;
	  }
	  
	  std::string host;
	  int port;
	  bool enc;
	  bool laser;
	  bool sick;
	  bool state;
	  std::string received_logname;
	  std::string sent_logname;
	  int protocol_ver;
	  //! If EPU external position should be used.
	  bool epu;
          bool protocol2Hack;
	  friend std::ostream& operator<<(std::ostream &os, const VMCNavserver::Params &obj)
	  {
	       os << "\nhost             : " << obj.host;
	       os << "\nport             : " << obj.port;
	       os << "\nenc              : " << obj.enc;
	       os << "\nlaser            : " << obj.laser;
	       os << "\nsick             : " << obj.sick;
	       os << "\nstate            : " << obj.state;
	       os << "\nreceived_logname : " << obj.received_logname;
	       os << "\nsent_logname     : " << obj.sent_logname;
	       os << "\nprotocol_ver     : " << obj.protocol_ver;
	       os << "\nepu              : " << obj.epu;
	       os << "\nprotocol2Hack    : " << obj.protocol2Hack;
	       return os;
	  }
     };

     VMCNavserver() {}
     VMCNavserver(const Params& params) : _params(params) {}
     ~VMCNavserver();
     void setParams(const Params &params);
     bool connect();
     bool reconnect();
     void disconnect();
     int receive(std::string &data);
     void send(const std::string &data);
private:
     void startSubscription();
     void stopSubscription();
     VMCNavserver::Params _params;

     XSocket _ethSock;
     std::ofstream _receivedFile;
     std::ofstream _sentFile;
};
#endif
