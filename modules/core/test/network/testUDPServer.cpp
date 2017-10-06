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
 * Test for UDP server.
 *
 *****************************************************************************/

/*!
  \example testUDPServer.cpp

  Example of a UDP server.
*/

#include <cstdlib>
#include <iostream>
#include <visp3/core/vpUDPServer.h>

int main() {
  try {
    int port = 50037;
    vpUDPServer server(port);

    while (true) {
      std::string msg = "", hostInfo = "";
      int res = server.receive(msg, hostInfo, 5000);
      if (res) {
        std::cout << "Server received: " << msg << " from: " << hostInfo << std::endl;
        std::cout << "Reply to the client: Echo: " << msg << std::endl;
        server.send("Echo: " + msg);
      } else if (res == 0) {
        std::cout << "Receive timeout" << std::endl;
      } else {
        std::cerr << "Error server.receive()!" << std::endl;
      }
    }
    return EXIT_SUCCESS;
  } catch(vpException &e) {
    std::cout << "Catch an exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}
