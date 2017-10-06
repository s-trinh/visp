/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * UDP Client
 *
 *****************************************************************************/

#include <cstring>
#include <sstream>

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
#include <unistd.h>
#include <netdb.h>
#include <errno.h>
#include <arpa/inet.h>
#ifndef DWORD
#define DWORD int
#endif
#else
#include <Ws2tcpip.h>
#endif

#include <visp3/core/vpUDPClient.h>

vpUDPClient::vpUDPClient(const std::string &hostname, const int port)
  : m_serverAddress(), m_serverLength(0), m_socketFileDescriptor()
#if defined(_WIN32)
  , m_wsa()
#endif
{
  init(hostname, port);
}

vpUDPClient::~vpUDPClient() {
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  close(m_socketFileDescriptor);
#else
  closesocket(m_socketFileDescriptor);
  WSACleanup();
#endif
}

void vpUDPClient::init(const std::string &hostname, const int port) {
#if defined(_WIN32)
  if (WSAStartup(MAKEWORD(2, 2), &m_wsa) != 0) {
    std::stringstream ss;
    ss << "Failed WSAStartup for the server, error code: " << WSAGetLastError();
    throw vpException(vpException::fatalError, ss.str());
  }
#endif

  /* socket: create the socket */
  m_socketFileDescriptor = socket(AF_INET, SOCK_DGRAM, 0);
  if (m_socketFileDescriptor < 0)
    throw vpException(vpException::fatalError, "Error opening UDP socket for the client!");

  /* build the server's Internet address */
  memset(&m_serverAddress, 0, sizeof(m_serverAddress));
  std::stringstream ss;
  ss << port;
  struct addrinfo hints;
  struct addrinfo *result = NULL;
  struct addrinfo *ptr = NULL;

  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = IPPROTO_UDP;

  DWORD dwRetval = getaddrinfo(hostname.c_str(), ss.str().c_str(), &hints, &result);
  if (dwRetval != 0) {
    ss.str("");
    ss << "getaddrinfo failed with error: " << dwRetval;
    throw vpException(vpException::fatalError, ss.str());
  }

  for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {
    if (ptr->ai_family == AF_INET && ptr->ai_socktype == SOCK_DGRAM) {
      m_serverAddress = *(struct sockaddr_in *) ptr->ai_addr;
      break;
    }
  }

  freeaddrinfo(result);

  m_serverLength = sizeof(m_serverAddress);
}

int vpUDPClient::receive(std::string &msg, const int timeoutMs) {
  fd_set s;
  FD_ZERO(&s);
  FD_SET(m_socketFileDescriptor, &s);
  struct timeval timeout;
  if (timeoutMs > 0) {
    timeout.tv_sec = timeoutMs / 1000;
    timeout.tv_usec = (timeoutMs % 1000) * 1000;
  }
  int retval = select((int)m_socketFileDescriptor + 1, &s, NULL, NULL, timeoutMs > 0 ? &timeout : NULL);

  if (retval == -1) {
    std::cerr << "Error select!" << std::endl;
    return -1;
  }

  if (retval > 0) {
    /* recvfrom: receive a UDP datagram from the server */
    memset(m_buf, '\0', VP_MAX_UDP_PAYLOAD);
    int length = recvfrom(m_socketFileDescriptor, m_buf, sizeof(m_buf) - 1, 0, (struct sockaddr *) &m_serverAddress, (socklen_t *)&m_serverLength);
    if (length <= 0) {
      return length < 0 ? -1 : 0;
    }

    m_buf[length] = '\0';
    msg = m_buf;

    return length;
  }

  //Timeout
  return 0;
}

int vpUDPClient::send(const std::string &msg) {
  if (msg.size() >= VP_MAX_UDP_PAYLOAD) {
    std::cerr << "Message is too long!" << std::endl;
    return 0;
  }

  memcpy(m_buf, msg.c_str(), msg.size());
  m_buf[msg.size()] = '\0';

  /* send the message to the server */
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  return sendto(m_socketFileDescriptor, m_buf, strlen(m_buf), 0, (struct sockaddr *) &m_serverAddress, m_serverLength);
#else
  return sendto(m_socketFileDescriptor, m_buf, (int)strlen(m_buf), 0, (struct sockaddr *) &m_serverAddress, m_serverLength);
#endif
}
