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


#include <iostream>
#include <kmo_navserver/XSocket.hh>


using namespace std;

#ifndef __IS_WINDOWS__

#include <signal.h>
#include <kmo_navserver/debug.hh>

/*bool XSocket::sighandleInitialized = false;*/

/**
 * ugly unforgivable *nix C hack to prevent spontaneous exits when remote end closes socket
 */
void XSocketsighandler(int signo)
{
	if( signo == SIGPIPE )
	{
		ERR("XSocket caught SIGPIPE");
	}
}

#else
#include <errno.h>
#endif



const int XSocket::BUFSIZE = 100 * 1024; // 100kB



#ifndef __IS_WINDOWS__
XSocket::XSocket(int existingSocket)
#else
XSocket::XSocket(SOCKET existingSocket)
#endif
{
#ifdef __IS_WINDOWS__
	int error = WSAStartup (0x0202,&wsa);
	if (error)
	{
		ERR("Error:  You need WinSock 2.2!");
		exit(1);
	}
#endif
	
	sockfd = existingSocket;
	
    init();

	aIsConnected = true;
}

XSocket::XSocket()
{
     sighandleInitialized = false;
#ifdef __IS_WINDOWS__
	int error = WSAStartup (0x0202,&wsa);
	if (error)
	{
		ERR("Error:  You need WinSock 2.2!");
		exit(1);
	}
#endif

	createSocket();
    
    init();
}

XSocket::~XSocket()
{
//	cerr << "XSocket destructing" << endl;
	disconnect();
	
#ifdef __IS_WINDOWS__
	WSACleanup();
#endif

//	cerr << "XSocket destructing, disconnected" << endl;
    if(buf != NULL) {
    	delete []buf;
        buf = NULL;
    }
//	cerr << "XSocket destructing, buffer deleted" << endl;
}

void XSocket::createSocket() {
	if ((sockfd = socket(PF_INET, SOCK_STREAM, 0)) == -1) {
#ifdef __IS_WINDOWS__
		WSACleanup ();
#endif
		ERR( "Could not make a new socket! %s",  + lastError().c_str() );
		//exit(1);
    }
    aIsConnected = false;
}

/**
 * helper for constructors
 */
void XSocket::init()
{
	buf = new char[BUFSIZE]; // 100kB
    alreadyReadBefore = 0;

	localAddrInitialized = false;
	remoteAddrInitialized = false;
    
#ifndef __IS_WINDOWS__
	if(!sighandleInitialized) {
		if( signal(SIGPIPE, (sig_t)XSocketsighandler) == SIG_ERR ) {
			ERR("Couldn't register for SIGPIPE 'broken pipe'");
		} else {
			sighandleInitialized = true;
		}
	}
#endif
    queueWhenDisconnected = false;
}

/**
 * set if calls to send() should be queued if the socket isn't connected yet
 */
void XSocket::setQueueWhenDisconnected(bool queue)
{
	queueWhenDisconnected = queue;
}

/**
 * Connects to the remote host and port given
 * 
 * \arg host ip address of the host to connect to
 * \arg port port number to connect to on the remote end
 */
bool XSocket::connect(const string& host, unsigned int port)
{
	if( aIsConnected ) {
		ERR("Socket already connected!");
		return false;
	}
	
	if( sockfd == -1 )
		createSocket();
	
	remoteAddr.sin_family = AF_INET;    // host byte order 
    remoteAddr.sin_port = htons(port);  // short, network byte order 
    remoteAddr.sin_addr.s_addr = inet_addr(host.c_str());
    
    if( ::connect(sockfd, (sockaddr*)&remoteAddr, sizeof(remoteAddr)) == -1 ) {
		aIsConnected = false;
		sockfd = -1;
  		ERR( "Couldn't connect to %s:%i! (%s)", host.c_str(), port, lastError().c_str() );
		return false;
  	}

	aIsConnected = true;
	
	int sent=0;
	while( !sendWhenDisconnectedQueue.empty() )
	{
		sent = send( sendWhenDisconnectedQueue.front() );
		sendWhenDisconnectedQueue.pop();
		//cerr << "sent queued message, size: " << sent  << endl;
	}
	
	return true;
}

bool XSocket::initRemoteAddr(bool reinit) {
	if(remoteAddrInitialized && !reinit)
		return true;

#ifndef __IS_WINDOWS__
	socklen_t addrSize = sizeof(sockaddr);
#else
	int addrSize = sizeof(sockaddr);
#endif
	
	if( ::getpeername(sockfd, (sockaddr*)&remoteAddr, &addrSize) == -1 ) {
		ERR( "Couldn't initalize remote address %s", lastError().c_str() );
		remoteAddrInitialized = false;
	} else {
		remoteAddrInitialized = true;
	}
	
	return remoteAddrInitialized;
}

bool XSocket::initLocalAddr(bool reinit) {
	if(localAddrInitialized && !reinit)
		return true;
	
#ifndef __IS_WINDOWS__
	socklen_t addrSize = sizeof(sockaddr);
#else
	int addrSize = sizeof(sockaddr);
#endif
	
	if( ::getsockname(sockfd, (sockaddr*)&localAddr, &addrSize) == -1 ) {
		ERR( "Couldn't initalize local address %s", lastError().c_str() );
		localAddrInitialized = false;
	} else {
		localAddrInitialized = true;
	}
	
	return localAddrInitialized;
}

unsigned int XSocket::getLocalPort() {
	if( aIsConnected && initLocalAddr() ) {
		return ntohs(localAddr.sin_port);
	}
	ERR("local address not initialized! %i", aIsConnected);
	return 0;
}
unsigned int XSocket::getRemotePort() {
	if( aIsConnected && initRemoteAddr() ) {
		return ntohs(remoteAddr.sin_port);
	}
	ERR("remote address not initialized!");
	return 0;
}
const std::string XSocket::getRemoteIP() {
	if( aIsConnected && initRemoteAddr() ) {
		return string( inet_ntoa(remoteAddr.sin_addr) );
	}
	ERR("remote address not initialized!");
	return string();
}

/**
 * Closes the socket if it is connected
 */
void XSocket::disconnect()
{
	if( aIsConnected ) {
		destroySocket();
	}
}

void XSocket::destroySocket() {

	if( -1 ==
#ifndef __IS_WINDOWS__
		close(sockfd)
#else
		closesocket(sockfd)
#endif
	  )
		ERR( "Couldn't close socket (%s)", lastError().c_str() );
	
	ERR("XSocket::destroySocket()");
	aIsConnected = false;
	sockfd = -1;
}

int XSocket::read(char* buf, int bufLen, unsigned long timeout) {
	
	if(timeout != std::numeric_limits<unsigned long>::max() && !waitForData(timeout)) {
			return -2;
	}
	
	int numNewlyRead = 0;
	
	if( ( numNewlyRead = ::recv(sockfd, buf, bufLen, 0 ) ) == -1 )
	{
		destroySocket();
		ERR( "Couldn't read data! (%s)", lastError().c_str() );
		return -1;
	}
	if( numNewlyRead == 0 ) {
		ERR("Connection closed by peer");
		destroySocket();
	}
	
	return numNewlyRead;
}

int XSocket::write(const char* buf, int bufLen) {
	int numWritten = 0;
	int numWrittenEver = 0;
	
    if(!aIsConnected) return -1;

	while(numWrittenEver < bufLen && aIsConnected) {
		if( ( numWritten = ::send(sockfd, buf+numWrittenEver, bufLen-numWrittenEver, 0) ) == -1 ) {
			destroySocket();
			ERR("Couldn't send data! (%s)", lastError().c_str());
			return -1;
		}
		numWrittenEver += numWritten;
	}
    
	return numWrittenEver;
}

/**
 * Sends the given string over this socket
 *
 * \arg toBeSent the string to be sent
 * \returns the number of bytes sent (in ascii the length of the string that was sent)
 */
int XSocket::send(string toBeSent, string delim)
{
	if( toBeSent.size() < delim.length() || toBeSent.substr(toBeSent.size()-delim.length(),delim.length()) != delim )
	{
    	toBeSent.append(delim);
        //cout<<"XSOCKET @@@@ "<<toBeSent<<endl;
	}
	
	if( !aIsConnected && !queueWhenDisconnected ) {
		ERR("Socket not connected.");
		return 0;
	}
		
	if( !aIsConnected && queueWhenDisconnected )
	{
		sendWhenDisconnectedQueue.push(toBeSent);
		return 0;
	}
	
	int numWrittenEver = 0;
	int numWritten = 0;
	const char* charData = toBeSent.c_str();
	int charDataLen = toBeSent.size();
	
	while( numWrittenEver < charDataLen )
	{
		if( ( numWritten = ::send( sockfd, charData+numWrittenEver, charDataLen-numWrittenEver, 0) ) == -1 )
		{
			destroySocket();
			ERR( "Couldn't send data! (%s)", lastError().c_str() );
			return numWrittenEver;
		}
		numWrittenEver += numWritten;
	}
	
	return numWrittenEver;
}

/**
 * helper function for receive()
 */
int XSocket::findNextCRLF(char* buf, int len, int offset, string delim)
{
	int start = offset;
    //char
    for(int i=start; i<len; i++) {
        if(strncmp(buf+i, delim.c_str(), delim.length()) == 0) {
            return (i+delim.length());
        }
    }
    /*
	for(int i=start; i<len; i++) {
		if( previous == '\r' && buf[i] == '\n' ) // found a newline thing
		{
			return i+1;
		}
		previous = buf[i];
	}
    */

	return -1;
}

/**
 * helper function for receive()
 */
int XSocket::copyToFrontOfBuffer(int offset, int end)
{
	for(int i=offset; i<end; i++) {
		buf[i-offset] = buf[i];
	}
	buf[end-offset] = 0; //null-terminate
	return end-offset; // next byte available for writing
}

/**
 * returns if this socket is already connected or not (i.e. it is possible to call receive() or send())
 */
bool XSocket::isConnected() {
	return aIsConnected;	
}

/**
 * helper function for receive(ulong)
 */
bool XSocket::waitForData(unsigned long msec)
{
	if(!aIsConnected) {
		return false;
	}

	fd_set sock_set;
	FD_ZERO(&sock_set);
	FD_SET( sockfd, &sock_set);
	
	timeval timeout;
	timeout.tv_sec = msec/1000;
	timeout.tv_usec = msec%1000 * 1000;
	
	int sel = select( sockfd+1,  &sock_set, NULL, NULL, &timeout );
	if( sel == -1 )
	{
		destroySocket();
		ERR( "Couldn't read data, select failed! (%s)", lastError().c_str() );
		return false;
	}
        
       

	return !( sel == 0 || !FD_ISSET(sockfd, &sock_set) );
}

/**
 * helper function for receive(ulong)
 */
bool XSocket::readMore()
{
	int numNewlyRead = 0;
	if( ( numNewlyRead = ::recv(sockfd, buf+alreadyReadBefore, BUFSIZE-1-alreadyReadBefore, 0 ) ) == -1 )
	{
		destroySocket();
		ERR( "Couldn't read data! (%s)", lastError().c_str() );
                return false;
	}
	if( numNewlyRead == 0 ) {
		// other side closed socket
                destroySocket();
		return false;
	}
	
	alreadyReadBefore += numNewlyRead;
	
	buf[alreadyReadBefore] = 0;
	
	return true;
}

/**
 * returns the length of the receive queue (filled by receive(ulong))
 */
int XSocket::getIncomingQueueLength() {
	return receiveQueue.size();
}


/**
 * Receives one line (terminated by \r\f characters) from the socket.
 *
 * Manages a queue, so one read() on the socket reads as much as possible into a buffer, then
 * it parses as many complete lines into the receive queue as there are in the buffer. If
 * the queue isn't empty, it returns a line from there instead of calling read() again.
 *
 * \arg msec time in milliseconds to wait for new data if there is none in the queue
 */
int XSocket::receive(string& output, unsigned long msec, string delim)
{
	if( !aIsConnected ) {
		ERR("Socket not connected.");
		return -1;
	}
		
	if( !receiveQueue.empty() )
	{
		string ret = receiveQueue.front();
		receiveQueue.pop();
		output = ret;
		return 1;
	}	
		
	if(msec != numeric_limits<unsigned long>::max() && !waitForData(msec) ) {
	     
	     return 0; // nothing to read
	}
	
	if(!readMore()) {
		return -1;
	}
	
	// we might have gotten so much data that we read multiple lines, check!
	
	int start = 0;
	int next = 0;
	
	if( alreadyReadBefore == 0 ) {
		// nothing read so far, assume there isn't any more for now
	     return 0;
	}
	
	next = findNextCRLF(buf, alreadyReadBefore, start, delim);
	while( next == -1 ) // we have to read again!
	{
		if(!readMore()) {
			return -1;
		}
		
		next = findNextCRLF(buf, alreadyReadBefore, start, delim);
	}
	
	// we have at least one whole line in the buffer now
	
	while( next != -1 )
	{
		buf[next-2] = 0;
		receiveQueue.push(string(buf+start));
		start = next;
		
		if( next == alreadyReadBefore ) // we are done
		{
			alreadyReadBefore = 0;
			break;
		}
		
		next = findNextCRLF(buf, alreadyReadBefore, next,delim);
	}
	
	if( next == -1 ) // we have an unfinished line left in the buffer
	{
		alreadyReadBefore = copyToFrontOfBuffer(start,alreadyReadBefore);
	}
	
	// there is always at least one string in the queue now since all data sent
	// by the server ends in \r\n and we don't cut it off before
	
	string ret = receiveQueue.front();
	receiveQueue.pop();
	output = ret;
	return 1;
}

string XSocket::lastError()
{
#ifndef __IS_WINDOWS__
	return string( strerror(errno) );
#else
	switch( WSAGetLastError() )
	{
		case WSAEINTR: return string("Interrupted function call.");
		case WSAEACCES: return string("Permission denied.");
		case WSAEFAULT: return string("Bad address.");
		case WSAEINVAL: return string("Invalid argument.");
		case WSAEMFILE: return string("Too many open files.");
		case WSAEWOULDBLOCK: return string("Resource temporarily unavailable.");
		case WSAEINPROGRESS: return string("Operation now in progress.");
		case WSAEALREADY: return string("Operation already in progress.");
		case WSAENOTSOCK: return string("Socket operation on nonsocket.");
		case WSAEDESTADDRREQ: return string("Destination address required.");
		case WSAEMSGSIZE: return string("Message too long.");
		case WSAEPROTOTYPE: return string("Protocol wrong type for socket.");
		case WSAENOPROTOOPT: return string("Bad protocol option.");
		case WSAEPROTONOSUPPORT: return string("Protocol not supported.");
		case WSAESOCKTNOSUPPORT: return string("Socket type not supported.");
		case WSAEOPNOTSUPP: return string("Operation not supported.");
		case WSAEPFNOSUPPORT: return string("Protocol family not supported.");
		case WSAEAFNOSUPPORT: return string("Address family not supported by protocol family.");
		case WSAEADDRINUSE: return string("Address already in use.");
		case WSAEADDRNOTAVAIL: return string("Cannot assign requested address.");
		case WSAENETDOWN: return string("Network is down.");
		case WSAENETUNREACH: return string("Network is unreachable.");
		case WSAENETRESET: return string("Network dropped connection on reset.");
		case WSAECONNABORTED: return string("Software caused connection abort.");
		case WSAECONNRESET: return string("Connection reset by peer.");
		case WSAENOBUFS: return string("No buffer space available.");
		case WSAEISCONN: return string("Socket is already connected.");
		case WSAESHUTDOWN: return string("Cannot send after socket shutdown.");
		case WSAETIMEDOUT: return string("Connection timed out.");
		case WSAECONNREFUSED: return string("Connection refused.");
		case WSAEHOSTDOWN: return string("Host is down.");
		case WSAEHOSTUNREACH: return string("No route to host.");
		case WSAEPROCLIM: return string("Too many processes.");
		case WSASYSNOTREADY: return string("Network subsystem is unavailable.");
		case WSAVERNOTSUPPORTED: return string("Winsock.dll version out of range.");
		case WSANOTINITIALISED: return string("Successful WSAStartup not yet performed.");
		case WSAEDISCON: return string("Graceful shutdown in progress.");
		case WSATYPE_NOT_FOUND: return string("Class type not found.");
		case WSAHOST_NOT_FOUND: return string("Host not found.");
		case WSATRY_AGAIN: return string("Nonauthoritative host not found.");
		case WSANO_RECOVERY: return string("This is a nonrecoverable error.");
		case WSANO_DATA: return string("Valid name, no data record of requested type.");
		case WSA_INVALID_HANDLE: return string("Specified event object handle is invalid.");
		case WSA_INVALID_PARAMETER: return string("One or more parameters are invalid.");
		case WSA_IO_INCOMPLETE: return string("Overlapped I/O event object not in signaled state.");
		case WSA_IO_PENDING: return string("Overlapped operations will complete later.");
		case WSA_OPERATION_ABORTED: return string("Overlapped operation aborted.");
		case WSASYSCALLFAILURE: return string("System call failure.");
		default: return string("Unknown error");
	};
#endif
}
