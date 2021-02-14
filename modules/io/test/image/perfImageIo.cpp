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
 * Benchmark image I/O.
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpIoTools.h>

static const std::string ipath = vpIoTools::getViSPImagesDataPath();
static std::string imagePath = vpIoTools::createFilePath(ipath, "faces/1280px-Solvay_conference_1927.png");

TEST_CASE("writeJPEGToMemory", "[benchmark]") {
  SECTION("unsigned char")
  {
    vpImage<unsigned char> I;
    vpImageIo::read(I, imagePath);

    {
      std::vector<unsigned char> data;
      BENCHMARK("Benchmark stb") {
        vpImageIo::writeJPEGToMemory(I, data);
        return data;
      };
    }

    {
      std::vector<unsigned char> data;
      BENCHMARK("Benchmark toojpeg") {
        vpImageIo::writeJPEGToMemory2(I, data);
        return data;
      };
    }
  }

  SECTION("vpRGBa")
  {
    vpImage<vpRGBa> I;
    vpImageIo::read(I, imagePath);

    {
      std::vector<unsigned char> data;
      BENCHMARK("Benchmark stb") {
        vpImageIo::writeJPEGToMemory(I, data);
        return data;
      };
    }

    {
      std::vector<unsigned char> data;
      BENCHMARK("Benchmark toojpeg") {
        vpImageIo::writeJPEGToMemory2(I, data);
        return data;
      };
    }
  }
}

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
TEST_CASE("writeJPEGToMemory (OpenCV)", "[benchmark]") {
  SECTION("unsigned char")
  {
    cv::Mat img = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);

    {
      std::vector<unsigned char> data;
      BENCHMARK("Benchmark OpenCV") {
        cv::imencode(".jpg", img, data);
        return data;
      };
    }
  }

  SECTION("BGR")
  {
    cv::Mat img = cv::imread(imagePath, cv::IMREAD_COLOR);

    {
      std::vector<unsigned char> data;
      BENCHMARK("Benchmark OpenCV") {
        cv::imencode(".jpg", img, data);
        return data;
      };
    }
  }
}
#endif

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  bool runBenchmark = false;
  // Build a new parser on top of Catch's
  using namespace Catch::clara;
  auto cli = session.cli() // Get Catch's composite command line parser
    | Opt(runBenchmark)    // bind variable to a new option, with a hint string
    ["--benchmark"]        // the option names it will respond to
    ("run benchmark?")     // description string for the help output
  ;

  // Now pass the new composite back to Catch so it uses that
  session.cli(cli);

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  if (runBenchmark) {
    int numFailed = session.run();

    // numFailed is clamped to 255 as some unices only use the lower 8 bits.
    // This clamping has already been applied, so just return it here
    // You can also do any post run clean-up here
    return numFailed;
  }

  return EXIT_SUCCESS;
}
#else
#include <iostream>

int main()
{
  return 0;
}
#endif
