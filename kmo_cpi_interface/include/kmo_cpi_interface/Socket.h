// Definition of the Socket class

#ifndef Socket_class
#define Socket_class


#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>


const int MAXHOSTNAME = 200;
const int MAXCONNECTIONS = 5;
const int MAXRECV = 500;

class Socket
{
 public:
  Socket();
  virtual ~Socket();

  // Server initialization
  bool create();
  bool bind ( const int port );
  bool listen() const;
  bool accept ( Socket& ) const;

  // Client initialization
  bool connect ( const std::string host, const int port );

  // Data Transimission
  bool send ( const std::string ) ;
  bool send ( void* buf, unsigned int size ) ;

  int recv ( std::string& ) ;
  bool  recv ( void* buf, unsigned int &size ) ;
  void set_non_blocking ( const bool );

  bool is_valid() const { return m_sock != -1; };
  
  // Abdelbaki: for recoonection purposes
  bool emptySocket();

   void clean();
   bool isConnected(){return connected;};
   virtual void reconnect(){};

protected: //private:

  int m_sock;
  sockaddr_in m_addr;
 
 // Abdelbaki: for recoonection purposes
  bool connected;
  std::string host;
  int port;

};


#endif
