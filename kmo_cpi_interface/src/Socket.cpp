// Implementation of the Socket class.


#include "kmo_cpi_interface/Socket.h"
#include "string.h"
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>


Socket::Socket() :
  m_sock ( -1 )
{

  memset ( &m_addr,
	   0,
	   sizeof ( m_addr ) );

	
  this->connected = false;
}

Socket::~Socket()
{
  if ( is_valid() )
    ::close ( m_sock );
}

void Socket::clean() // same as destructor, to be called by subclasses
{
	
  if ( is_valid() )   
    ::close ( m_sock );
  
  this->connected = false;
}



bool Socket::create()
{

  m_sock = socket ( AF_INET,
		    SOCK_STREAM,
		    0 );

  if ( ! is_valid() )
    return false;


  // TIME_WAIT - argh
  int on = 1;
  if ( setsockopt ( m_sock, SOL_SOCKET, SO_REUSEADDR, 
		    ( const char* ) &on, sizeof ( on ) ) == -1 )
    return false;

   
  return true;

}



bool Socket::bind ( const int port )
{

  if ( ! is_valid() )
    {
      return false;
    }



  m_addr.sin_family = AF_INET;
  m_addr.sin_addr.s_addr = INADDR_ANY;
  m_addr.sin_port = htons ( port );

  int bind_return = ::bind ( m_sock,
			     ( struct sockaddr * ) &m_addr,
			     sizeof ( m_addr ) );


  if ( bind_return == -1 )
    {
      return false;
    }

  return true;
}


bool Socket::listen() const
{
  if ( ! is_valid() )
    {
      return false;
    }

  int listen_return = ::listen ( m_sock, MAXCONNECTIONS );


  if ( listen_return == -1 )
    {
      return false;
    }

  return true;
}


bool Socket::accept ( Socket& new_socket ) const
{
  int addr_length = sizeof ( m_addr );
  new_socket.m_sock = ::accept ( m_sock, ( sockaddr * ) 
				 &m_addr, ( socklen_t * ) &addr_length );

  if ( new_socket.m_sock <= 0 )
    return false;
  else
    return true;
}


bool Socket::send ( const std::string s ) 
{
  int status = ::send ( m_sock, s.c_str(), s.size(), MSG_NOSIGNAL );
  if ( status == -1 )
    {
	    this->connected = false;
            return false;
    }
  else
    {
	 return true;
    }
}

bool Socket::send ( void* buf, unsigned int size ) 
{
     unsigned char* data = static_cast<unsigned char*>(buf);
     
     if (::send ( m_sock, buf, size, MSG_NOSIGNAL ) == -1 )
     {
	     this->connected = false;
	  return false;
     }
     return true;
}

int Socket::recv (std::string &s ) 
{
  char buf [ MAXRECV + 1 ];

  s = "";

  memset ( buf, 0, MAXRECV + 1 );

  int status = ::recv ( m_sock, buf, MAXRECV, 0 );

  if ( status == -1 )
  {
    this->connected = false;
    std::cout << "status == -1   errno == " << errno << "  in Socket::recv\n";
    return false;
  }
  else if ( status == 0 )
  {
    this->connected = false;
    return false;
  }
  else
  {
    s.assign(buf, status);
    return true;
  }
}

bool Socket::recv ( void* buf, unsigned int &size ) 
{
  int status = ::recv ( m_sock, buf, MAXRECV, 0 );

  if ( status == -1 )
    {
      this->connected = false;
      std::cout << "status == -1   errno == " << errno << "  in Socket::recv\n";
      size = 0;
      return 0;
    }
  else if ( status == 0 )
    {
      this->connected = false;
      size = 0;
      return 0;
    }
  else
    {
       size = static_cast<unsigned int>(status);
       return status;
    }
}




bool Socket::emptySocket ()
{
  char buf [ MAXRECV + 1 ];

  

  memset ( buf, 0, MAXRECV + 1 );
  int status ;
do
  {
    status = ::recv ( m_sock, buf, MAXRECV, 0 );
  }while (status > 0);
  
 return status == 0;
}


bool Socket::connect ( const std::string host, const int port )
{
  if ( ! is_valid() ) return false;

  m_addr.sin_family = AF_INET;
  m_addr.sin_port = htons ( port );

  int status = inet_pton ( AF_INET, host.c_str(), &m_addr.sin_addr );

  if ( errno == EAFNOSUPPORT ) return false;

  status = ::connect ( m_sock, ( sockaddr * ) &m_addr, sizeof ( m_addr ) );

  if ( status == 0 )
  {	  
	  this->connected = true;
    return true;
  }
  else
  {	  
	  this->connected = false;
    return false;
  } 
	  
}

void Socket::set_non_blocking ( const bool b )
{

  int opts;

  opts = fcntl ( m_sock,
		 F_GETFL );

  if ( opts < 0 )
    {
      return;
    }

  if ( b )
    opts = ( opts | O_NONBLOCK );
  else
    opts = ( opts & ~O_NONBLOCK );

  fcntl ( m_sock,
	  F_SETFL,opts );

}
