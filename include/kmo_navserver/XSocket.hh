/*
Copyright (C) 2008 Jacobs University

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public
License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef XSOCKET_H_
#define XSOCKET_H_


#if defined(__WIN32__) || defined(WIN32) || defined(_WINDOWS)
#define __IS_WINDOWS__
#endif

#ifndef __IS_WINDOWS__

#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/socket.h>

#else // windows

#include <winsock2.h>

#endif

#include <limits>
#include <string>
#include <queue>


	
/**
 * Simple asynchronous socket wrapper around berkley socket api
 * 
 * This is the berkley socket implementation, a winsock32 one also exists
 */
class XSocket
{
protected:
	
#ifndef __IS_WINDOWS__
	int sockfd;
#else	
	SOCKET sockfd;
	WSADATA wsa;
#endif

	bool remoteAddrInitialized;
	sockaddr_in remoteAddr;
	
	bool localAddrInitialized;
	sockaddr_in localAddr;
	
     /*static*/ bool sighandleInitialized;
	
	bool aIsConnected;
	
	char* buf;
	int alreadyReadBefore;
	
	std::queue< std::string > receiveQueue;
	
	std::queue< std::string > sendWhenDisconnectedQueue;
	bool queueWhenDisconnected;
	
	virtual void init();
	virtual void createSocket();
	virtual void destroySocket();
	
	bool initRemoteAddr(bool =false);
	bool initLocalAddr(bool =false);
	
	int findNextCRLF(char*, int, int, std::string delim="\r\f");
	int copyToFrontOfBuffer(int, int);
	
	bool waitForData(unsigned long);
	bool readMore();

	
public:
     static const int BUFSIZE;

#ifndef __IS_WINDOWS__
	XSocket(int);
#else
	XSocket(SOCKET);
#endif

	XSocket();
	virtual ~XSocket();
	
#ifndef __IS_WINDOWS__
	virtual int getFD() { return sockfd; }
#else
	virtual SOCKET getFD() { return sockfd; }
#endif
	
	virtual unsigned int getLocalPort();
	virtual unsigned int getRemotePort();
	virtual const std::string getRemoteIP();
	
	virtual void setQueueWhenDisconnected(bool);
	
	virtual bool connect(const std::string&, unsigned int);
	
	virtual void disconnect();
	
	virtual bool isConnected();
	
	virtual int read(char*, int, unsigned long = std::numeric_limits<unsigned long>::max());
	virtual int write(const char*, int);
	
	virtual int send(std::string, std::string delim="\r\n");
	virtual int receive(std::string&, unsigned long = std::numeric_limits<unsigned long>::max(), std::string delim="\r\n");
	
	virtual int getIncomingQueueLength();
	
	virtual std::string lastError();
};


#endif /*XSOCKET_H_*/
