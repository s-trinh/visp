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
 * Test for UDP client.
 *
 *****************************************************************************/

/*!
  \example testUDPClient.cpp

  Example of a UDP client.
*/

#include <cstdlib>
#include <iostream>
#include <visp3/core/vpUDPClient.h>

int main() {
  try {
    //std::string servername = "localhost";
    std::string servername = "127.0.0.1";
    unsigned int port = 50037;
    vpUDPClient client(servername, port);

    while (true) {
      std::cout << "Enter the message to send:" << std::endl;
      std::string msg = "";
      std::getline(std::cin, msg);
      if (client.send(msg) != (int) msg.size())
        std::cerr << "Error client.send()!" << std::endl;
      if (client.receive(msg))
        std::cout << "Receive from the server: " << msg << std::endl;
    }
    return EXIT_SUCCESS;
  } catch (vpException &e) {
    std::cerr << "Catch an exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}
