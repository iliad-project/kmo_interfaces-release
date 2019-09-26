#include <kmo_navserver/kmo_navserver.hh>
#include <assert.h>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace std;

template<class T> std::string toString (const T& x)
{
    std::ostringstream o;

    if (!(o << x))
        throw std::runtime_error ("::toString()");

    return o.str ();
}

VMCNavserver::~VMCNavserver()
{
     disconnect();
}

void
VMCNavserver::setParams(const Params &params)
{
     _params = params;
}
     

bool 
VMCNavserver::connect()
{
     if (_params.received_logname != "")
	  _receivedFile.open(_params.received_logname.c_str());
     if (_params.received_logname != "")
	  _sentFile.open(_params.sent_logname.c_str());
     if (!_ethSock.connect(_params.host,_params.port))
	  return false;
     startSubscription();
     return true;
}

bool
VMCNavserver::reconnect()
{
     disconnect();
     if (connect() == false)
	  return false;
     startSubscription();
     return true;
}
 
void
VMCNavserver::disconnect()
{
     stopSubscription();
     _ethSock.disconnect();
}

int
VMCNavserver::receive(string &data)
{
     int byte = _ethSock.receive(data,2000);
     if (_params.received_logname != "")
     {
	  if (_receivedFile.is_open())
	       _receivedFile << data << endl;
     }
     return byte;
}

void
VMCNavserver::send(const string &data)
{
//     assert(false); // When starting to use EPU etc. don't forget to add the ugly protocol 2 hack (changing to protocol 0 before sending the epu and then switch back to protocol 2...)
     
     _ethSock.send(data);
     if (_params.sent_logname != "")
     {
	  if (_sentFile.is_open())
	       _sentFile << data << endl;
     }
}

void 
VMCNavserver::startSubscription()
{
     assert(_ethSock.isConnected());
     send("basetime");

     std::string protocol_str = "protocol " + toString(_params.protocol_ver);
     send(protocol_str);
     if (_params.enc)
	  send("subscribe enc");
     if (_params.laser)
	  send("subscribe laser");
     if (_params.state)
	  send("subscribe state");
     if (_params.sick)
	  send("subscribe sick");
     
     if (_params.epu)
	  send("nav epu");
}

void 
VMCNavserver::stopSubscription()
{
     if (_ethSock.isConnected())
	  _ethSock.send("!");
}
