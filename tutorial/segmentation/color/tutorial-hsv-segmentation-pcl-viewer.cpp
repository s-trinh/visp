//! \example tutorial-hsv-segmentation-pcl.cpp

#include <iostream>
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_PCL) && defined(VISP_HAVE_THREADS)
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpColorDepthConversion.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayPCL.h>
#include <visp3/sensor/vpRealSense2.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#if 1
class vpPointCloudViewer
{
public:
  explicit vpPointCloudViewer() : m_stop(false), m_flush_viewer(false) { }

  void flush()
  {
    m_flush_viewer = true;
  }

  void run(std::mutex &mutex, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());

    bool flush_viewer = false;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setPosition(640 + 80, 480 + 80);
    viewer->setCameraPosition(0, 0, -0.25, 0, -1, 0);
    viewer->setSize(640, 480);

    bool first_init = true;
    while (!m_stop) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        flush_viewer = m_flush_viewer;
        m_flush_viewer = false;
        local_pointcloud = pointcloud->makeShared();
      }

      if (flush_viewer) {
        if (first_init) {

          viewer->addPointCloud<pcl::PointXYZ>(local_pointcloud, "sample cloud");
          viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
          first_init = false;
        }
        else {
          viewer->updatePointCloud<pcl::PointXYZ>(local_pointcloud, "sample cloud");
        }
      }

      viewer->spinOnce(10);
    }

    std::cout << "End of point cloud display thread" << std::endl;
  }

  void stop()
  {
    m_stop = true;
  }

private:
  bool m_stop;
  bool m_flush_viewer;
};
#endif

int main(int argc, char **argv)
{
  std::string opt_hsv_filename = "calib/hsv-thresholds.yml";

  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--hsv-thresholds") {
      opt_hsv_filename = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "\nSYNOPSIS " << std::endl
        << argv[0]
        << " [--hsv-thresholds <filename.yml>]"
        << " [--help,-h]"
        << std::endl;
      std::cout << "\nOPTIONS " << std::endl
        << "  --hsv-thresholds <filename.yaml>" << std::endl
        << "    Path to a yaml filename that contains H <min,max>, S <min,max>, V <min,max> threshold values." << std::endl
        << "    An Example of such a file could be:" << std::endl
        << "      rows: 6" << std::endl
        << "      cols: 1" << std::endl
        << "      data:" << std::endl
        << "        - [0]" << std::endl
        << "        - [42]" << std::endl
        << "        - [177]" << std::endl
        << "        - [237]" << std::endl
        << "        - [148]" << std::endl
        << "        - [208]" << std::endl
        << std::endl
        << "  --help, -h" << std::endl
        << "    Display this helper message." << std::endl
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  vpColVector hsv_values;
  if (vpColVector::loadYAML(opt_hsv_filename, hsv_values)) {
    std::cout << "Load HSV threshold values from " << opt_hsv_filename << std::endl;
    std::cout << "HSV low/high values: " << hsv_values.t() << std::endl;
  }
  else {
    std::cout << "Warning: unable to load HSV thresholds values from " << opt_hsv_filename << std::endl;
    return EXIT_FAILURE;
  }

  int width = 848, height = 480, fps = 60;
  vpRealSense2 rs;
  rs2::config config;
  config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, fps);
  config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
  config.disable_stream(RS2_STREAM_INFRARED, 1);
  config.disable_stream(RS2_STREAM_INFRARED, 2);
  rs2::align align_to(RS2_STREAM_COLOR);

  rs.open(config);

  float depth_scale = rs.getDepthScale();
  vpCameraParameters cam_depth = rs.getCameraParameters(RS2_STREAM_DEPTH,
                                                        vpCameraParameters::perspectiveProjWithoutDistortion);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  vpImage<vpRGBa> Ic(height, width);
  vpImage<unsigned char> H(height, width);
  vpImage<unsigned char> S(height, width);
  vpImage<unsigned char> V(height, width);
  vpImage<unsigned char> Ic_segmented(height, width, 0);
  vpImage<uint16_t> depth_raw(height, width);

  vpDisplayX d_Ic(Ic, 0, 0, "Current frame");
  vpDisplayX d_Ic_segmented(Ic_segmented, Ic.getWidth()+75, 0, "HSV segmented frame");

  bool quit = false;
  double loop_time = 0., total_loop_time = 0.;
  long nb_iter = 0;
  float Z_min = 0.2;
  float Z_max = 2.5;
  int pcl_size = 0;

  vpDisplayPCL pcl_viewer;
  std::mutex pcl_viewer_mutex;
  std::thread pcl_viewer_thread(&vpDisplayPCL::run, &pcl_viewer, std::ref(pcl_viewer_mutex), pointcloud);

  while (!quit) {
    double t = vpTime::measureTimeMs();
    rs.acquire((unsigned char *)Ic.bitmap, (unsigned char *)(depth_raw.bitmap), NULL, NULL, &align_to);
    vpImageConvert::RGBaToHSV(reinterpret_cast<unsigned char *>(Ic.bitmap),
                              reinterpret_cast<unsigned char *>(H.bitmap),
                              reinterpret_cast<unsigned char *>(S.bitmap),
                              reinterpret_cast<unsigned char *>(V.bitmap), Ic.getSize());

    vpImageTools::inRange(reinterpret_cast<unsigned char *>(H.bitmap),
                          reinterpret_cast<unsigned char *>(S.bitmap),
                          reinterpret_cast<unsigned char *>(V.bitmap),
                          hsv_values,
                          reinterpret_cast<unsigned char *>(Ic_segmented.bitmap),
                          Ic_segmented.getSize());

    {
      std::lock_guard<std::mutex> lock(pcl_viewer_mutex);
      vpImageConvert::depthToPointCloud(depth_raw, depth_scale, cam_depth, pointcloud, &Ic_segmented, Z_min, Z_max);
      pcl_size = pointcloud->size();
      pcl_viewer.flush();
    }

    std::cout << "Segmented point cloud size: " << pcl_size << std::endl;

    vpDisplay::display(Ic);
    vpDisplay::display(Ic_segmented);
    vpDisplay::displayText(Ic, 20, 20, "Click to quit...", vpColor::red);

    if (vpDisplay::getClick(Ic, false)) {
      quit = true;
    }

    vpDisplay::flush(Ic);
    vpDisplay::flush(Ic_segmented);
    nb_iter++;
    loop_time = vpTime::measureTimeMs() - t;
    total_loop_time += loop_time;
  }

  pcl_viewer.stop();

  if (pcl_viewer_thread.joinable()) {
    pcl_viewer_thread.join();
  }
  std::cout << "Mean loop time: " << total_loop_time / nb_iter << std::endl;
  return EXIT_SUCCESS;
}
#else
int main()
{
#if !defined(VISP_HAVE_REALSENSE2)
  std::cout << "This tutorial needs librealsense as 3rd party." << std::endl;
#endif
#if !defined(VISP_HAVE_PCL)
  std::cout << "This tutorial needs pcl library as 3rd party." << std::endl;
#endif
  std::cout << "Install missing 3rd party, configure and rebuild ViSP." << std::endl;
  return EXIT_SUCCESS;
}
#endif
