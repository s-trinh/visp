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

#ifndef __vpUDPClient_h__
#define __vpUDPClient_h__

#include <visp3/core/vpConfig.h>

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#else
#include <winsock2.h>
#endif

#include <visp3/core/vpException.h>

#define VP_MAX_UDP_PAYLOAD 508

class VISP_EXPORT vpUDPClient {
public:
  vpUDPClient(const std::string &hostname, const int port);
  ~vpUDPClient();

  int receive(std::string &msg, const int timeoutMs=0);
  int send(const std::string &msg);

private:
  char m_buf[VP_MAX_UDP_PAYLOAD];
  struct sockaddr_in m_serverAddress;
  int m_serverLength;
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  int m_socketFileDescriptor;
#else
  SOCKET m_socketFileDescriptor;
  WSADATA m_wsa;
#endif

  void init(const std::string &hostname, const int port);
};

#endif
