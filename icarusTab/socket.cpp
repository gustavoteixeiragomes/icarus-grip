/* 
   Socket.cpp

   Copyright (C) René Nyffenegger

   This source code is provided 'as-is', without any express or implied
   warranty. In no event will the author be held liable for any damages
   arising from the use of this software.

   Permission is granted to anyone to use this software for any purpose,
   including commercial applications, and to alter it and redistribute it
   freely, subject to the following restrictions:

   1. The origin of this source code must not be misrepresented; you must not
      claim that you wrote the original source code. If you use this source code
      in a product, an acknowledgment in the product documentation would be
      appreciated but is not required.

   2. Altered source versions must be plainly marked as such, and must not be
      misrepresented as being the original source code.

   3. This notice may not be removed or altered from any source distribution.

   René Nyffenegger rene.nyffenegger@adp-gmbh.ch

   =======================================================================
   ==                   THIS VERSION HAS BEEN MODIFIED                  ==
   =======================================================================


   1.1	Gustavo Gomes - socket.class@guvux.com.br														2013-06-12

   * [General]							Impleented the namespace
   * [SocketInterface::ReceiveBytes]	Transfers the flow control to the requester
   * [SocketClient::SocketClient]		Modify to accepts ip adresses
   * [SocketServer::getSocket]			Implemented to return the socket and enable the verification of incoming data on the server
   removed Socket::Select, Socket::Select::Readable
   modify SocketServer add SocketServer::Create

*/
//#define _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES 1
//#define _CRT_SECURE_NO_WARNINGS

#ifndef WIN32_LEAN_AND_MEAN
	#define WIN32_LEAN_AND_MEAN
#endif
#ifndef _CRT_SECURE_NO_WARNINGS
	#define _CRT_SECURE_NO_WARNINGS 1
#endif

#include <stdio.h>
#include <sys/types.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <string>

#include "socket.h"

using namespace std;

int SocketInterface::Socket::nofSockets_= 0;

void SocketInterface::Socket::Start() {
	cout << "Start socket" << endl;
  if (!nofSockets_) {
    WSADATA info;
    if (WSAStartup(MAKEWORD(2,1), &info)) {
      throw "Could not start WSA";
    }
  }
  ++nofSockets_;
}

void SocketInterface::Socket::End() {
  WSACleanup();
}

SocketInterface::Socket::Socket() : s_(0) {
  Start();
  // UDP: use SOCK_DGRAM instead of SOCK_STREAM
  s_ = socket(AF_INET,SOCK_STREAM,0);

  if (s_ == INVALID_SOCKET) {
    throw "INVALID_SOCKET";
  }

  refCounter_ = new int(1);
}

SocketInterface::Socket::Socket(SOCKET s) : s_(s) {
  Start();
  refCounter_ = new int(1);
};

SocketInterface::Socket::~Socket() {
  if (! --(*refCounter_)) {
    Close();
    delete refCounter_;
  }

  --nofSockets_;
  if (!nofSockets_) End();
}

SocketInterface::Socket::Socket(const Socket& o) {
  refCounter_=o.refCounter_;
  (*refCounter_)++;
  s_         =o.s_;

  nofSockets_++;
}

SocketInterface::Socket& SocketInterface::Socket::operator=(Socket& o) {
  (*o.refCounter_)++;

  refCounter_=o.refCounter_;
  s_         =o.s_;

  nofSockets_++;

  return *this;
}

void SocketInterface::Socket::Close() {
  closesocket(s_);
}

std::string SocketInterface::Socket::ReceiveBytes() {
	std::string ret;
	char buf[MAXBUFFER];
	u_long arg;
	int rv;
	rv = MAXBUFFER;
    
    arg = 0;
	if (ioctlsocket(s_, FIONREAD, &arg) != 0) {
		return "";
	}
	rv = recv (s_, buf, MAXBUFFER, 0);
    if (rv <= 0) {
		return "";
	}
	std::string t;

    t.assign (buf, rv);
    ret += t;
	return ret;
}

std::string SocketInterface::Socket::ReceiveLine() {
  std::string ret;
  while (1) {
    char r;

	u_long arg;
	arg = 0;
	if (ioctlsocket(s_, FIONREAD, &arg) != 0) {
		return "";
	}
    switch(recv(s_, &r, 1, 0)) {
      case 0: // not connected anymore;
              // ... but last line sent
              // might not end in \n,
              // so return ret anyway.
        return ret;
      case -1:
        return "";
//      if (errno == EAGAIN) {
//        return ret;
//      } else {
//      // not connected anymore
//      return "";
//      }
    }
	if (r == '\n') {
		return ret;
	}
	ret += r;
  }
}

void SocketInterface::Socket::SendLine(std::string s) {
  s += '\n';
  send(s_,s.c_str(),s.length(),0);
}

void SocketInterface::Socket::SendBytes(const std::string& s, int size) {
  send(s_,s.c_str(),size,0);
}

SOCKET SocketInterface::Socket::getSocket() {
	return s_;
}

SocketInterface::SocketServer::SocketServer(int port, int connections, TypeSocket type) {
  Start();
  sockaddr_in sa;
  
  memset(&sa, 0, sizeof(sa));

  sa.sin_family = PF_INET;             
  sa.sin_port = htons(port);          
  s_ = socket(AF_INET, SOCK_STREAM, 0);
  if (s_ == INVALID_SOCKET) {
    throw "INVALID_SOCKET";
  }
  if(type==NonBlockingSocket) {
    u_long arg = 1;
    ioctlsocket(s_, FIONBIO, &arg);
  }
  /* bind the socket to the internet address */
  if (bind(s_, (sockaddr *)&sa, sizeof(sockaddr_in)) == SOCKET_ERROR) {
    closesocket(s_);
    throw "INVALID_SOCKET";
  }
  listen(s_, connections);
}

SocketInterface::Socket* SocketInterface::SocketServer::Accept() {
  SOCKET new_sock = accept(s_, 0, 0);
  if (new_sock == INVALID_SOCKET) {
    int rc = WSAGetLastError();
    if(rc==WSAEWOULDBLOCK) {
      return 0; // non-blocking call, no request pending
    }
    else {
      throw "Invalid Socket";
    }
  }

  Socket* r = new Socket(new_sock);
  return r;
}

SocketInterface::SocketClient::SocketClient(const std::string &host, int port) : Socket() {
  std::string error;

  hostent *he;
  // IP or Name
  unsigned long ip = inet_addr(host.c_str());
  if( ip != INADDR_NONE )
  {
    // ip
    he = gethostbyaddr((char *)&ip,4,AF_INET);
  }
  else
  {
    // name
    he = gethostbyname(host.c_str());
  }
  if (he == 0) {
    error = strerror(errno);
    throw error;
  }

  sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr = *((in_addr *)he->h_addr);
  memset(&(addr.sin_zero), 0, 8); 

  if (::connect(s_, (sockaddr *) &addr, sizeof(sockaddr))) {
    error = strerror(WSAGetLastError());
    throw error;
  }
}