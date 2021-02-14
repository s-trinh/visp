/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
 * Test image I/O.
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

/*!
  \example testImageIo.cpp

  \brief Test image I/O.
*/

#if defined(VISP_HAVE_CATCH2)
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>
#include <bitset>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

#include <visp3/gui/vpDisplayX.h>

namespace
{
static std::bitset<64> dhash(const vpImage<unsigned char>& I)
{
  vpImage<unsigned char> I_resized;
  vpImageTools::resize(I, I_resized, 9, 8, vpImageTools::INTERPOLATION_LINEAR);

  vpImage<unsigned char> I_diff(8, 8);
  for (unsigned int i = 0; i < I_resized.getHeight(); i++) {
    for (unsigned int j = 0; j < I_resized.getWidth()-1; j++) {
      if (I_resized[i][j] > I_resized[i][j+1]) {
        I_diff[i][j] = 1;
      }
    }
  }

  std::cout << "I_diff:\n" << I_diff << std::endl;

  std::bitset<64> hash;
  for (unsigned int i = 0; i < I_diff.getSize(); i++) {
    if (I_diff.bitmap[i]) {
      hash.set(i, 1);
    }
  }
  return hash;
}
}

TEST_CASE("Test image I/O (JPEG)") {
  const std::string filepath = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                         "Klimt/Klimt.pgm");
  vpImage<unsigned char> I;
  vpImageIo::read(I, filepath);

  std::vector<unsigned char> jpeg_data;
  vpImageIo::writeJPEGToMemory(I, jpeg_data);
  std::cout << "jpeg_data: " << jpeg_data.size() << std::endl;

  vpImage<unsigned char> I_from_jpeg;
  vpImageIo::readJPEGFromMemory(I_from_jpeg, jpeg_data);

  std::bitset<64> hash_original = dhash(I);
  std::bitset<64> hash_from_jpeg = dhash(I_from_jpeg);

  std::cout << "Hash original image: " << hash_original << std::endl;
  std::cout << "Hash image from Jpeg: " << hash_from_jpeg << std::endl;
  std::cout << "Hamming distance: " << (hash_original ^= hash_from_jpeg).count() << std::endl;
}

TEST_CASE("Test image I/O (PNG)") {
  const std::string filepath = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                         "Klimt/Klimt.pgm");
  vpImage<unsigned char> I;
  vpImageIo::read(I, filepath);

  std::vector<unsigned char> png_data;
  vpImageIo::writePNGToMemory(I, png_data);
  std::cout << "png_data: " << png_data.size() << std::endl;

  vpImage<unsigned char> I_from_png;
  vpImageIo::readPNGFromMemory(I_from_png, png_data);

  std::bitset<64> hash_original = dhash(I);
  std::bitset<64> hash_from_jpeg = dhash(I_from_png);

  std::cout << "Hash original image: " << hash_original << std::endl;
  std::cout << "Hash image from PNG: " << hash_from_jpeg << std::endl;
  std::cout << "Hamming distance: " << (hash_original ^= hash_from_jpeg).count() << std::endl;
}

TEST_CASE("Test image I/O (STB PNG)") {
//  const std::string filepath = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
//                                                         "Klimt/Klimt.pgm");
  const std::string filepath = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                         "faces/1280px-Solvay_conference_1927.png");
  vpImage<unsigned char> I;
  vpImageIo::read(I, filepath);

  std::vector<unsigned char> png_data;
  vpImageIo::writeImageToMemory(vpImageIo::TYPE_PNG, I, png_data);
  std::cout << "png_data: " << png_data.size() << std::endl;

  {
    // Test compression = 1
    std::vector<unsigned char> png_data_compression;
    std::map<std::string, int> params;
    params["compression"] = 1;
    vpImageIo::writeImageToMemory(vpImageIo::TYPE_PNG, I, png_data_compression, params);
    std::cout << "png_data_compression (" << params["compression"] << "): " << png_data_compression.size() << std::endl;
  }
  {
    // Test compression = 9
    std::vector<unsigned char> png_data_compression;
    std::map<std::string, int> params;
    params["compression"] = 9;
    vpImageIo::writeImageToMemory(vpImageIo::TYPE_PNG, I, png_data_compression, params);
    std::cout << "png_data_compression (" << params["compression"] << "): " << png_data_compression.size() << std::endl;
  }

  vpImage<unsigned char> I_from_png;
  vpImageIo::readImageFromMemory(png_data, I_from_png);

  std::bitset<64> hash_original = dhash(I);
  std::bitset<64> hash_from_png = dhash(I_from_png);

  std::cout << "Hash original image: " << hash_original << std::endl;
  std::cout << "Hash image from PNG: " << hash_from_png << std::endl;
  std::cout << "Hamming distance: " << (hash_original ^= hash_from_png).count() << std::endl;
  std::cout << "Same images? " << (I == I_from_png) << std::endl;
}

int main(int argc, char *argv[])
{
//#if 0
//  vpImage<unsigned char> I;
//  const std::string filepath = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Klimt/Klimt.pgm");
//  vpImageIo::read(I, filepath);

//  vpImage<unsigned char> I_mem;
//#else
//  vpImage<vpRGBa> I;
//  const std::string filepath = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Klimt/Klimt.ppm");
//  vpImageIo::read(I, filepath);

//  vpImage<vpRGBa> I_mem;
//#endif

//#if 0
//  std::vector<unsigned char> data;
//  vpImageIo::writeJPEGToMemory(I, data);
//  std::cout << "data: " << data.size() << std::endl;

//  vpImageIo::readJPEGFromMemory(I_mem, data);
//  std::cout << "I_mem: " << I_mem.getWidth() << "x" << I_mem.getHeight() << std::endl;
//#else
//  std::vector<unsigned char> data;
//  vpImageIo::writeJPEGToMemory2(I, data);
//  std::cout << "data: " << data.size() << std::endl;

//  vpImageIo::readJPEGFromMemory2(I_mem, data);
//  std::cout << "I_mem: " << I_mem.getWidth() << "x" << I_mem.getHeight() << std::endl;
//#endif

//  vpDisplayX d(I_mem);

//  vpDisplay::display(I_mem);
//  vpDisplay::flush(I_mem);
//  vpDisplay::getClick(I_mem);

//  return 0;



  Catch::Session session; // There must be exactly one instance

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
}
#else
int main()
{
  return 0;
}
#endif
