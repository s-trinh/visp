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
   \example saveRealSenseData2.cpp

   \brief Example that shows how to save realsense data using concurrent queue and NPZ format.
 */

#include <iostream>

#include <visp3/core/vpConfig.h>
#if (defined(VISP_HAVE_REALSENSE2)) && defined(VISP_HAVE_THREADS) \
  && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(VISP_HAVE_PUGIXML)

#include <condition_variable>
#include <fstream>
#include <mutex>
#include <queue>
#include <thread>

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpConcurrentQueue.h>
#include <visp3/core/vpWriterExecutor.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/io/vpFileStorageWorker.h>
#include <visp3/io/vpVideoStorageWorker.h>
#include <visp3/sensor/vpRealSense2.h>

// If set, use a single thread to save the acquisition data
#define TEST_SINGLE_THREAD 1

#define GETOPTARGS "so:acdpiCf:h"

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
                int &stream_fps)
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

template <class T, class Container = std::deque<std::vector<T>>> class vpNPZStorageWorker : public vpWriterWorker
{
public:
  vpNPZStorageWorker(const std::vector<std::reference_wrapper<vpConcurrentQueue<std::vector<T>, Container>>> &queues,
    const std::vector<std::string> &filenames, const std::vector<unsigned int> &heights,
    const std::vector<unsigned int> &widths, const std::vector<unsigned int> &channels)
    : m_filenames(filenames), m_data_vec(), m_queues(queues), m_heights(heights), m_widths(widths),
    m_channels(channels), m_iter()
  {
    assert(!m_queues.empty());
    assert(m_queues.size() == m_filenames.size());
    assert(m_heights.size() == m_filenames.size());
    assert(m_widths.size() == m_filenames.size());
    assert(m_channels.size() == m_filenames.size());
  }

  vpNPZStorageWorker(const std::reference_wrapper<vpConcurrentQueue<std::vector<T>, Container>> &queue,
    const std::string &filename, unsigned int height, unsigned int width, unsigned int channel)
    : m_filenames(), m_data_vec(), m_queues(), m_heights(), m_widths(), m_channels(), m_iter()
  {
    m_queues.push_back(queue);
    m_filenames.push_back(filename);
    m_heights.push_back(height);
    m_widths.push_back(width);
    m_channels.push_back(channel);
  }

  void init() override
  {
    m_data_vec.resize(m_queues.size());
  }

  // Thread main loop
  void run() override
  {
    init();

    while (runOnce());
  }

  bool runOnce() override
  {
    try {
      for (size_t i = 0; i < m_queues.size(); i++) {
        m_data_vec[i] = m_queues[i].get().pop();

        char filename[FILENAME_MAX];
        snprintf(filename, FILENAME_MAX, m_filenames[i].c_str(), m_iter);
        std::string str_filename = filename;

        // Write Npz headers
        std::vector<char> vec_filename(str_filename.begin(), str_filename.end());
        visp::cnpy::npz_save(filename, "filename", &vec_filename[0], { vec_filename.size() }, "w");

        std::string current_time = vpTime::getDateTime("%Y-%m-%d_%H.%M.%S");
        std::vector<char> vec_current_time(current_time.begin(), current_time.end());
        visp::cnpy::npz_save(filename, "timestamp", &vec_current_time, { vec_current_time.size() }, "a");

        int height = m_heights[i];
        int width = m_widths[i];
        int channel = m_channels[i];
        visp::cnpy::npz_save(filename, "height", &height, { 1 }, "a");
        visp::cnpy::npz_save(filename, "width", &width, { 1 }, "a");
        visp::cnpy::npz_save(filename, "channel", &channel, { 1 }, "a");

        // Write data
        const std::vector<T> &ptr_data = m_data_vec[i];
        visp::cnpy::npz_save(filename, "data", ptr_data.data(), { m_heights[i], m_widths[i], m_channels[i] }, "a");
      }

      m_iter++;
    }
    catch (typename vpConcurrentQueue<std::vector<T>, Container>::vpCancelled_t &) {
      return false;
    }

    return true;
  }

private:
  std::vector<std::string> m_filenames;
  std::vector<std::vector<T>> m_data_vec;
  std::vector< std::reference_wrapper<vpConcurrentQueue<std::vector<T>, Container>> > m_queues;
  std::vector<unsigned int> m_heights;
  std::vector<unsigned int> m_widths;
  std::vector<unsigned int> m_channels;
  int m_iter;
};

void convert(const std::vector<vpColVector> &vp_pcl, std::vector<float> &pcl)
{
  pcl.resize(3*vp_pcl.size());
  for (size_t i = 0; i < vp_pcl.size(); i++) {
    pcl[3*i + 0] = (float)vp_pcl[i][0];
    pcl[3*i + 1] = (float)vp_pcl[i][1];
    pcl[3*i + 2] = (float)vp_pcl[i][2];
  }
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

  // Read the command line options
  if (!getOptions(argc, argv, save, output_directory_custom, use_aligned_stream, save_color, save_depth,
                  save_pointcloud, save_infrared, click_to_save, stream_fps)) {
    return EXIT_FAILURE;
  }

  if (!output_directory_custom.empty())
    output_directory = output_directory_custom + "/" + output_directory;

  std::cout << "save: " << save << std::endl;
  std::cout << "output_directory: " << output_directory << std::endl;
  std::cout << "use_aligned_stream: " << use_aligned_stream << std::endl;
  std::cout << "save_color: " << save_color << std::endl;
  std::cout << "save_depth: " << save_depth << std::endl;
  std::cout << "save_pointcloud: " << save_pointcloud << std::endl;
  std::cout << "save_infrared: " << save_infrared << std::endl;
  std::cout << "stream_fps: " << stream_fps << std::endl;
  std::cout << "click_to_save: " << click_to_save << std::endl;

  const int width = 640, height = 480;
  vpRealSense2 realsense;

  rs2::config config;
  config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, stream_fps);
  config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, stream_fps);
  config.enable_stream(RS2_STREAM_INFRARED, width, height, RS2_FORMAT_Y8, stream_fps);
  realsense.open(config);

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
    std::ofstream file(std::string(output_directory + "/depth_M_color.txt"));
    depth_M_color.save(file);
    file.close();
  }

  // Synchronized queues for each camera stream
  vpConcurrentQueue<vpImage<vpRGBa>> rgb_queue;
  vpConcurrentQueue<vpImage<unsigned char>> infrared_queue;
  vpConcurrentQueue<std::vector<uint16_t>> depth_queue;
  vpConcurrentQueue<std::vector<float>> pointcloud_queue;
  std::vector<unsigned int> header_vec;
  std::vector<std::shared_ptr<vpWriterWorker>> storages;
#if TEST_SINGLE_THREAD
  std::unique_ptr<vpWriterExecutor> storage_thread_ptr;
#else
  std::vector<vpWriterExecutor> storage_threads;
#endif

  if (save) {
    std::ostringstream oss;
    oss << output_directory;
    std::cout << "Create directory: " << oss.str() << std::endl;
    vpIoTools::makeDirectory(oss.str());
    oss << "/color_image_%04d.jpg";
    std::string rgb_output = oss.str();

    storages.emplace_back(std::make_shared<vpVideoStorageWorker<vpRGBa>>(rgb_queue, rgb_output));

    oss.str("");
    oss.clear();
    oss << output_directory << "/infrared_image_%04d.jpg";
    std::string ir_output = oss.str();
    storages.emplace_back(std::make_shared<vpVideoStorageWorker<unsigned char>>(infrared_queue, ir_output));

    oss.str("");
    oss.clear();
    oss << output_directory << "/depth_image_%04d.npz";
    std::string npz_depth_output = oss.str();
    const unsigned int depth_channel = 1;
    storages.emplace_back(std::make_shared<vpNPZStorageWorker<uint16_t>>(depth_queue, npz_depth_output,
                                                                         height,
                                                                         width,
                                                                         depth_channel));

    oss.str("");
    oss.clear();
    oss << output_directory << "/pointcloud_%04d.npz";
    std::string pointcloud_output = oss.str();
    const unsigned int pcl_channel = 3;
    storages.emplace_back(std::make_shared<vpNPZStorageWorker<float>>(pointcloud_queue, pointcloud_output,
                                                                      height,
                                                                      width,
                                                                      pcl_channel));

#if TEST_SINGLE_THREAD
    storage_thread_ptr = std::make_unique<vpWriterExecutor>(storages);
#else
    storage_threads.emplace_back(storages);
#endif
  }

  rs2::align align_to(RS2_STREAM_COLOR);
  if (use_aligned_stream && save_infrared) {
    std::cerr << "Cannot use aligned streams with infrared acquisition currently."
      << "\nInfrared stream acquisition is disabled!"
      << std::endl;
  }

  int nb_saves = 0;
  bool quit = false;
  std::vector<vpColVector> pointCloud;
  std::vector<float> vec_pointCloud;
  while (!quit) {
    if (use_aligned_stream) {
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, &pointCloud, nullptr,
                        &align_to);
    }
    else {
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, &pointCloud,
                        (unsigned char *)I_infrared.bitmap);
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
      if (save_color) {
        rgb_queue.push(I_color);
      }
      if (save_infrared) {
        infrared_queue.push(I_infrared);
      }
      if (save_depth) {
        std::vector<uint16_t> I_depth_raw_vec(I_depth_raw.bitmap, I_depth_raw.bitmap + I_depth_raw.getSize());
        std::vector<uint16_t> I_depth_raw_vec_copy = I_depth_raw_vec;
        depth_queue.push(I_depth_raw_vec_copy);
      }
      if (save_pointcloud) {
        convert(pointCloud, vec_pointCloud);
        pointcloud_queue.push(vec_pointCloud);
      }
    }

    vpMouseButton::vpMouseButtonType button;
    if (vpDisplay::getClick(I_gray, button, false)) {
      if (!click_to_save) {
        rgb_queue.cancel();
        infrared_queue.cancel();
        depth_queue.cancel();
        pointcloud_queue.cancel();

        quit = true;
      }
      else {
        switch (button) {
        case vpMouseButton::button1:
          if (save) {
            nb_saves++;
            rgb_queue.push(I_color);
            infrared_queue.push(I_infrared);
            depth_queue.cancel();
            pointcloud_queue.cancel();
          }
          break;

        case vpMouseButton::button2:
        case vpMouseButton::button3:
        default:
          rgb_queue.cancel();
          infrared_queue.cancel();
          depth_queue.cancel();
          pointcloud_queue.cancel();

          quit = true;
          break;
        }
      }
    }
  }

  // Join all the worker threads, waiting for them to finish
#if TEST_SINGLE_THREAD
  if (storage_thread_ptr) {
    storage_thread_ptr->join();
  }
#else
  for (auto &st : storage_threads) {
    st.join();
  }
#endif

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
