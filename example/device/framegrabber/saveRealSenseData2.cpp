/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 *****************************************************************************/

 /*!
   \example saveRealSenseData.cpp

   \brief Example that show how to save realsense data that can be replayed with readRealSenseData.cpp
 */

#include <iostream>

#include <visp3/core/vpConfig.h>
#if (defined(VISP_HAVE_REALSENSE) || defined(VISP_HAVE_REALSENSE2)) && defined(VISP_HAVE_THREADS) \
  && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(VISP_HAVE_PUGIXML)

#include <condition_variable>
#include <fstream>
#include <mutex>
#include <queue>
#include <thread>

#if defined(VISP_HAVE_PCL)
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#endif

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpConcurrentQueue.h>
#include <visp3/core/vpWriterExecutor.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/io/vpVideoStorageWorker.h>
#include <visp3/sensor/vpRealSense.h>
#include <visp3/sensor/vpRealSense2.h>

 // Priority to libRealSense2
#if defined(VISP_HAVE_REALSENSE2)
#define USE_REALSENSE2
#endif

#define GETOPTARGS "so:acdpiCf:bh"

namespace
{
void usage(const char *name, const char *badparam, int fps)
{
  std::cout << "\nSYNOPSIS " << std::endl
    << "  " << name
    << " [-s]"
    << " [-a]"
    << " [-c]"
    << " [-d]"
    << " [-p]"
    << " [-b]"
    << " [-i]"
    << " [-C]"
    << " [-f <fps>]"
    << " [-o <directory>]"
    << " [--help,-h]"
    << std::endl;
  std::cout << "\nOPTIONS " << std::endl
    << "  -s" << std::endl
    << "    Flag to enable data saving." << std::endl
    << std::endl
    << "  -a" << std::endl
    << "    Color and depth are aligned." << std::endl
    << std::endl
    << "  -c" << std::endl
    << "    Add color stream to saved data when -s option is enable." << std::endl
    << std::endl
    << "  -d" << std::endl
    << "    Add depth stream to saved data when -s option is enable." << std::endl
    << std::endl
    << "  -p" << std::endl
    << "    Add point cloud stream to saved data when -s option is enabled." << std::endl
    << "    By default, the point cloud is saved in Point Cloud Data file format (.PCD extension file)." << std::endl
    << "    You can also use -b option to save the point cloud in binary format." << std::endl
    << std::endl
    << "  -b" << std::endl
    << "    Point cloud stream is saved in binary format." << std::endl
    << std::endl
    << "  -i" << std::endl
    << "    Add infrared stream to saved data when -s option is enabled." << std::endl
    << std::endl
    << "  -C" << std::endl
    << "    Trigger one shot data saver after each user click." << std::endl
    << std::endl
    << "  -f <fps>" << std::endl
    << "    Set camera framerate." << std::endl
    << "    Default: " << fps << std::endl
    << std::endl
    << "  -o <directory>" << std::endl
    << "    Output directory that will host saved data." << std::endl
    << std::endl
    << "  --help, -h" << std::endl
    << "    Display this helper message." << std::endl
    << std::endl;

  if (badparam) {
    std::cout << "\nERROR: Bad parameter " << badparam << std::endl;
  }
}

bool getOptions(int argc, const char *argv[], bool &save, std::string &output_directory, bool &use_aligned_stream,
                bool &save_color, bool &save_depth, bool &save_pointcloud, bool &save_infrared, bool &click_to_save,
                int &stream_fps, bool &save_pointcloud_binary_format)
{
  const char *optarg;
  const char **argv1 = (const char **)argv;
  int c;
  while ((c = vpParseArgv::parse(argc, argv1, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 's':
      save = true;
      break;
    case 'o':
      output_directory = optarg;
      break;
    case 'a':
      use_aligned_stream = true;
      break;
    case 'c':
      save_color = true;
      break;
    case 'd':
      save_depth = true;
      break;
    case 'p':
      save_pointcloud = true;
      break;
    case 'i':
      save_infrared = true;
      break;
    case 'C':
      click_to_save = true;
      break;
    case 'f':
      stream_fps = atoi(optarg);
      break;
    case 'b':
      save_pointcloud_binary_format = true;
      break;

    case 'h':
      usage(argv[0], nullptr, stream_fps);
      return false;
      break;

    default:
      usage(argv[0], optarg, stream_fps);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], nullptr, stream_fps);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl
      << std::endl;
    return false;
  }

  return true;
}

} // Namespace

int main(int argc, const char *argv[])
{
  bool save = false;
  std::string output_directory = vpTime::getDateTime("%Y_%m_%d_%H.%M.%S");
  std::string output_directory_custom = "";
  bool use_aligned_stream = false;
  bool save_color = false;
  bool save_depth = false;
  bool save_pointcloud = false;
  bool save_infrared = false;
  bool click_to_save = false;
  int stream_fps = 30;
  bool save_pointcloud_binary_format = false;

  // Read the command line options
  if (!getOptions(argc, argv, save, output_directory_custom, use_aligned_stream, save_color, save_depth,
                  save_pointcloud, save_infrared, click_to_save, stream_fps, save_pointcloud_binary_format)) {
    return EXIT_FAILURE;
  }

  if (!output_directory_custom.empty())
    output_directory = output_directory_custom + "/" + output_directory;

#ifndef VISP_HAVE_PCL
  save_pointcloud_binary_format = true;
#endif

  std::cout << "save: " << save << std::endl;
  std::cout << "output_directory: " << output_directory << std::endl;
  std::cout << "use_aligned_stream: " << use_aligned_stream << std::endl;
  std::cout << "save_color: " << save_color << std::endl;
  std::cout << "save_depth: " << save_depth << std::endl;
  std::cout << "save_pointcloud: " << save_pointcloud << std::endl;
  std::cout << "save_infrared: " << save_infrared << std::endl;
  std::cout << "stream_fps: " << stream_fps << std::endl;
  std::cout << "save_pointcloud_binary_format: " << save_pointcloud_binary_format << std::endl;
  std::cout << "click_to_save: " << click_to_save << std::endl;

  int width = 640, height = 480;
#ifdef USE_REALSENSE2
  vpRealSense2 realsense;

  rs2::config config;
  config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, stream_fps);
  config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, stream_fps);
  config.enable_stream(RS2_STREAM_INFRARED, width, height, RS2_FORMAT_Y8, stream_fps);
  realsense.open(config);
#else
  vpRealSense realsense;
  realsense.setStreamSettings(rs::stream::color,
                              vpRealSense::vpRsStreamParams(width, height, rs::format::rgba8, stream_fps));
  realsense.setStreamSettings(rs::stream::depth,
                              vpRealSense::vpRsStreamParams(width, height, rs::format::z16, stream_fps));
  realsense.setStreamSettings(rs::stream::infrared,
                              vpRealSense::vpRsStreamParams(width, height, rs::format::y8, stream_fps));
  realsense.setStreamSettings(rs::stream::infrared2,
                              vpRealSense::vpRsStreamParams(width, height, rs::format::y8, stream_fps));

  realsense.open();
#endif

  vpImage<vpRGBa> I_color(height, width);
  vpImage<unsigned char> I_gray(height, width);
  vpImage<unsigned char> I_depth(height, width);
  vpImage<uint16_t> I_depth_raw(height, width);
  vpImage<unsigned char> I_infrared(height, width);

#ifdef VISP_HAVE_X11
  vpDisplayX d1, d2, d3;
#else
  vpDisplayGDI d1, d2, d3;
#endif
  d1.init(I_gray, 0, 0, "RealSense color stream");
  d2.init(I_depth, I_gray.getWidth() + 80, 0, "RealSense depth stream");
  d3.init(I_infrared, I_gray.getWidth() + 80, I_gray.getHeight() + 70, "RealSense infrared stream");

  while (true) {
    realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, nullptr, nullptr);
    vpImageConvert::convert(I_color, I_gray);
    vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

    vpDisplay::display(I_gray);
    vpDisplay::display(I_depth);
    vpDisplay::displayText(I_gray, 20, 20, "Click when ready.", vpColor::red);
    vpDisplay::flush(I_gray);
    vpDisplay::flush(I_depth);

    if (vpDisplay::getClick(I_gray, false)) {
      break;
    }
  }

  if (save) {
    // Create output directory
    vpIoTools::makeDirectory(output_directory);

    // Save intrinsics
#ifdef USE_REALSENSE2
    vpCameraParameters cam_color = realsense.getCameraParameters(RS2_STREAM_COLOR);
    vpXmlParserCamera xml_camera;
    xml_camera.save(cam_color, output_directory + "/camera.xml", "color_camera", width, height);

    if (use_aligned_stream) {
      xml_camera.save(cam_color, output_directory + "/camera.xml", "depth_camera", width, height);
    }
    else {
      vpCameraParameters cam_depth = realsense.getCameraParameters(RS2_STREAM_DEPTH);
      xml_camera.save(cam_depth, output_directory + "/camera.xml", "depth_camera", width, height);
    }

    vpCameraParameters cam_infrared = realsense.getCameraParameters(RS2_STREAM_INFRARED);
    xml_camera.save(cam_infrared, output_directory + "/camera.xml", "infrared_camera", width, height);
    vpHomogeneousMatrix depth_M_color;
    if (!use_aligned_stream) {
      depth_M_color = realsense.getTransformation(RS2_STREAM_COLOR, RS2_STREAM_DEPTH);
    }
#else
    vpCameraParameters cam_color = realsense.getCameraParameters(rs::stream::color);
    vpXmlParserCamera xml_camera;
    xml_camera.save(cam_color, output_directory + "/camera.xml", "color_camera", width, height);

    vpCameraParameters cam_color_rectified = realsense.getCameraParameters(rs::stream::rectified_color);
    xml_camera.save(cam_color_rectified, output_directory + "/camera.xml", "color_camera_rectified", width, height);

    if (use_aligned_stream) {
      vpCameraParameters cam_depth = realsense.getCameraParameters(rs::stream::depth);
      xml_camera.save(cam_depth, output_directory + "/camera.xml", "depth_camera", width, height);
    }
    else {
      xml_camera.save(cam_color, output_directory + "/camera.xml", "depth_camera", width, height);
    }

    vpCameraParameters cam_depth_aligned_to_rectified_color =
      realsense.getCameraParameters(rs::stream::depth_aligned_to_rectified_color);
    xml_camera.save(cam_depth_aligned_to_rectified_color, output_directory + "/camera.xml",
                    "depth_camera_aligned_to_rectified_color", width, height);

    vpCameraParameters cam_infrared = realsense.getCameraParameters(rs::stream::infrared);
    xml_camera.save(cam_infrared, output_directory + "/camera.xml", "infrared_camera", width, height);
    vpHomogeneousMatrix depth_M_color;
    if (!use_aligned_stream) {
      depth_M_color = realsense.getTransformation(rs::stream::color, rs::stream::depth);
    }
#endif
    std::ofstream file(std::string(output_directory + "/depth_M_color.txt"));
    depth_M_color.save(file);
    file.close();
  }

  // vpFrameQueue save_queue;
  // vpStorageWorker storage(std::ref(save_queue), std::cref(output_directory), save_color, save_depth, save_pointcloud,
  //                       save_infrared, save_pointcloud_binary_format, width, height);
  // std::thread storage_thread(&vpStorageWorker::run, &storage);

  // Synchronized queues for each camera stream
  vpConcurrentQueue<vpImage<vpRGBa>> rgb_queue;
  vpConcurrentQueue<vpImage<unsigned char>> infrared_queue;
  std::vector<std::shared_ptr<vpWriterWorker>> storages;
  std::vector<vpWriterExecutor> storage_threads;

  if (save) {
    std::ostringstream oss;
    oss << output_directory;
    std::cout << "Create directory: " << oss.str() << std::endl;
    vpIoTools::makeDirectory(oss.str());
    oss << "/rgb_%06d.png";
    std::string rgb_output = oss.str();

    storages.emplace_back(std::make_shared<vpVideoStorageWorker<vpRGBa>>(rgb_queue, rgb_output));

    oss.str("");
    oss.clear();
    oss << output_directory << "/ir_%06d.png";
    std::string ir_output = oss.str();
    storages.emplace_back(std::make_shared<vpVideoStorageWorker<unsigned char>>(infrared_queue, ir_output));

    storage_threads.emplace_back(storages);
  }

#ifdef USE_REALSENSE2
  rs2::align align_to(RS2_STREAM_COLOR);
  if (use_aligned_stream && save_infrared) {
    std::cerr << "Cannot use aligned streams with infrared acquisition currently."
      << "\nInfrared stream acquisition is disabled!"
      << std::endl;
  }
#endif

  int nb_saves = 0;
  bool quit = false;
#ifdef VISP_HAVE_PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
#else
  std::vector<vpColVector> pointCloud;
#endif
  while (!quit) {
    if (use_aligned_stream) {
#ifdef USE_REALSENSE2
#ifdef VISP_HAVE_PCL
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, nullptr, pointCloud, nullptr,
                        &align_to);
#else
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, &pointCloud, nullptr,
                        &align_to);
#endif
#else
#ifdef VISP_HAVE_PCL
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, nullptr, pointCloud,
                        (unsigned char *)I_infrared.bitmap, nullptr, rs::stream::rectified_color,
                        rs::stream::depth_aligned_to_rectified_color);
#else
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, &pointCloud,
                        (unsigned char *)I_infrared.bitmap, nullptr, rs::stream::rectified_color,
                        rs::stream::depth_aligned_to_rectified_color);
#endif
#endif
    }
    else {
#ifdef VISP_HAVE_PCL
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, nullptr, pointCloud,
                        (unsigned char *)I_infrared.bitmap, nullptr);
#else
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, &pointCloud,
                        (unsigned char *)I_infrared.bitmap);
#endif
    }

    vpImageConvert::convert(I_color, I_gray);
    vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

    vpDisplay::display(I_gray);
    vpDisplay::display(I_depth);
    vpDisplay::display(I_infrared);

    if (!click_to_save) {
      vpDisplay::displayText(I_gray, 20, 20, "Click to quit.", vpColor::red);
    }
    else {
      std::stringstream ss;
      ss << "Images saved: " << nb_saves;
      vpDisplay::displayText(I_gray, 20, 20, ss.str(), vpColor::red);
    }

    vpDisplay::flush(I_gray);
    vpDisplay::flush(I_depth);
    vpDisplay::flush(I_infrared);

    if (save && !click_to_save) {
#ifdef VISP_HAVE_PCL
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_copy = pointCloud->makeShared();
      save_queue.push(I_color, I_depth_raw, pointCloud_copy, I_infrared);
#else
      // save_queue.push(I_color, I_depth_raw, pointCloud, I_infrared);
      rgb_queue.push(I_color);
      infrared_queue.push(I_infrared);
#endif
    }

    vpMouseButton::vpMouseButtonType button;
    if (vpDisplay::getClick(I_gray, button, false)) {
      if (!click_to_save) {
        // save_queue.cancel();
        rgb_queue.cancel();
        infrared_queue.cancel();

        quit = true;
      }
      else {
        switch (button) {
        case vpMouseButton::button1:
          if (save) {
            nb_saves++;
#ifdef VISP_HAVE_PCL
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_copy = pointCloud->makeShared();
            save_queue.push(I_color, I_depth_raw, pointCloud_copy, I_infrared);
#else
            // save_queue.push(I_color, I_depth_raw, pointCloud, I_infrared);
            rgb_queue.push(I_color);
            infrared_queue.push(I_infrared);
#endif
          }
          break;

        case vpMouseButton::button2:
        case vpMouseButton::button3:
        default:
          // save_queue.cancel();
          rgb_queue.cancel();
          infrared_queue.cancel();

          quit = true;
          break;
        }
      }
    }
  }

  // storage_thread.join();
  // Join all the worker threads, waiting for them to finish
  for (auto &st : storage_threads) {
    st.join();
  }

  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cerr << "Need libRealSense or libRealSense2 and C++11 and displayX or displayGDI!" << std::endl;

#if !defined(VISP_HAVE_PUGIXML)
  std::cout << "pugixml built-in 3rdparty is requested." << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
