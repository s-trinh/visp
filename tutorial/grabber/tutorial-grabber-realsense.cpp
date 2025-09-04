/*! \example tutorial-grabber-realsense.cpp */
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageStorageWorker.h>
#include <visp3/sensor/vpRealSense.h>
#include <visp3/sensor/vpRealSense2.h>

#include <opencv2/calib3d.hpp>
#include <visp3/detection/vpDetectorAprilTag.h>

void usage(const char *argv[], int error)
{
  std::cout << "SYNOPSIS" << std::endl
    << "  " << argv[0] << " [--fps <6|15|30|60>]"
    << " [--width <image width>]"
    << " [--height <image height>]"
    << " [--seqname <sequence name>]"
    << " [--record <mode>]"
    << " [--no-display]"
    << " [--help] [-h]" << std::endl
    << std::endl;
  std::cout << "DESCRIPTION" << std::endl
    << "  --fps <6|15|30|60>" << std::endl
    << "    Frames per second." << std::endl
    << "    Default: 30." << std::endl
    << std::endl
    << "  --width <image width>" << std::endl
    << "    Default: 640." << std::endl
    << std::endl
    << "  --height <image height>" << std::endl
    << "    Default: 480." << std::endl
    << std::endl
    << "  --seqname <sequence name>" << std::endl
    << "    Name of the sequence of image to create (ie: /tmp/image%04d.jpg)." << std::endl
    << "    Default: empty." << std::endl
    << std::endl
    << "  --record <mode>" << std::endl
    << "    Allowed values for mode are:" << std::endl
    << "      0: record all the captures images (continuous mode)," << std::endl
    << "      1: record only images selected by a user click (single shot mode)." << std::endl
    << "    Default mode: 0" << std::endl
    << std::endl
    << "  --no-display" << std::endl
    << "    Disable displaying captured images." << std::endl
    << "    When used and sequence name specified, record mode is internally set to 1 (continuous mode)."
    << std::endl
    << std::endl
    << "  --help, -h" << std::endl
    << "    Print this helper message." << std::endl
    << std::endl;
  std::cout << "USAGE" << std::endl
    << "  Example to visualize images:" << std::endl
    << "    " << argv[0] << std::endl
    << std::endl
    << "  Examples to record a sequence of successive images in 640x480 resolution:" << std::endl
    << "    " << argv[0] << " --seqname I%04d.png" << std::endl
    << "    " << argv[0] << " --seqname folder/I%04d.png --record 0" << std::endl
    << std::endl
    << "  Examples to record single shot 640x480 images:\n"
    << "    " << argv[0] << " --seqname I%04d.png --record 1\n"
    << "    " << argv[0] << " --seqname folder/I%04d.png --record 1" << std::endl
    << std::endl
    << "  Examples to record single shot 1280x720 images:\n"
    << "    " << argv[0] << " --seqname I%04d.png --record 1 --width 1280 --height 720" << std::endl
    << std::endl;

  if (error) {
    std::cout << "Error" << std::endl
      << "  "
      << "Unsupported parameter " << argv[error] << std::endl;
  }
}

void toImagePoints(std::vector<vpImagePoint> &imPtsIn, std::vector<cv::Point2d> &imPtsOut)
{
  imPtsOut.resize(imPtsIn.size());
  int size = imPtsIn.size();
  for (size_t i = 0; i < imPtsIn.size(); i++) {
    imPtsOut[(size - i) % size].x = imPtsIn[i].get_u();
    imPtsOut[(size - i) % size].y = imPtsIn[i].get_v();
  }
}

/*!
  Grab images from an Intel realsense camera
 */
int main(int argc, const char *argv[])
{
#if defined(VISP_HAVE_REALSENSE) || defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_THREADS)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display, display2, display_merge;
#else
  vpDisplay *display = nullptr;
#endif
  try {
    std::string opt_seqname;
    int opt_record_mode = 0;
    int opt_fps = 30;
    bool opt_display = true;
    unsigned int opt_width = 640;
    unsigned int opt_height = 480;
    int p3p_method = 0;
    bool save = false;

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--fps" && i + 1 < argc) {
        opt_fps = std::atoi(argv[++i]);
      }
      else if (std::string(argv[i]) == "--seqname" && i + 1 < argc) {
        opt_seqname = std::string(argv[++i]);
      }
      else if (std::string(argv[i]) == "--width" && i + 1 < argc) {
        opt_width = std::atoi(argv[++i]);
      }
      else if (std::string(argv[i]) == "--height" && i + 1 < argc) {
        opt_height = std::atoi(argv[++i]);
      }
      else if (std::string(argv[i]) == "--record" && i + 1 < argc) {
        opt_record_mode = std::atoi(argv[++i]);
      }
      else if (std::string(argv[i]) == "--no-display") {
        opt_display = false;
      }
      else if (std::string(argv[i]) == "--ap3p") {
        p3p_method = 1;
      }
      else if (std::string(argv[i]) == "--save") {
        save = true;
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        usage(argv, 0);
        return EXIT_SUCCESS;
      }
      else {
        usage(argv, i);
        return EXIT_FAILURE;
      }
    }

    if ((!opt_display) && (!opt_seqname.empty())) {
      opt_record_mode = 0;
    }

    if (opt_fps != 6 && opt_fps != 15 && opt_fps != 30 && opt_fps != 60) {
      opt_fps = 30; // Default
    }
    std::cout << "Resolution : " << opt_width << " x " << opt_height << std::endl;
    std::cout << "Recording  : " << (opt_seqname.empty() ? "disabled" : "enabled") << std::endl;
    std::cout << "Framerate  : " << opt_fps << std::endl;
    std::cout << "Display    : " << (opt_display ? "enabled" : "disabled") << std::endl;

    std::string text_record_mode =
      std::string("Record mode: ") + (opt_record_mode ? std::string("single") : std::string("continuous"));

    if (!opt_seqname.empty()) {
      std::cout << text_record_mode << std::endl;
      std::cout << "Record name: " << opt_seqname << std::endl;
    }
    vpImage<vpRGBa> I, I2, I_merge;

#ifdef VISP_HAVE_REALSENSE2
    std::cout << "SDK        : Realsense 2" << std::endl;
    vpRealSense2 g;
    rs2::config config;
    config.disable_stream(RS2_STREAM_DEPTH);
    config.disable_stream(RS2_STREAM_INFRARED);
    config.enable_stream(RS2_STREAM_COLOR, opt_width, opt_height, RS2_FORMAT_RGBA8, opt_fps);
    g.open(config);
#else
    std::cout << "SDK        : Realsense 1" << std::endl;
    vpRealSense g;
    g.setStreamSettings(rs::stream::color, vpRealSense::vpRsStreamParams(opt_width, opt_height, rs::format::rgba8, 60));
    g.open();
#endif
    g.acquire(I);
    I2 = I;
    I_merge.init(I.getHeight(), 2*I.getWidth());

    std::cout << "Image size : " << I.getWidth() << " " << I.getHeight() << std::endl;
    std::cout << "AP3P? " << (p3p_method != 0) << std::endl;
    std::cout << "Save? " << save << std::endl;

    // vpCameraParameters cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
    vpCameraParameters cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithDistortion);
    std::cout << "Cam:\n" << cam << std::endl;

#if defined(VISP_HAVE_PUGIXML)
    if (!opt_seqname.empty()) {
      vpXmlParserCamera p;
      std::string output_folder = vpIoTools::getParent(opt_seqname);
      if (!vpIoTools::checkDirectory(output_folder)) {
        try {
          std::cout << "Create output folder: " << output_folder << std::endl;
          vpIoTools::makeDirectory(output_folder);
        }
        catch (const vpException &e) {
          std::cout << e.getStringMessage();
          return EXIT_FAILURE;
        }
      }
      std::string cam_filename = output_folder + "/camera.xml";

      std::cout << "Save camera intrinsics in: " << cam_filename << std::endl;
      if (p.save(cam, cam_filename, "camera")) {
        std::cout << "Cannot save camera parameters in " << cam_filename << std::endl;
      }
    }
#else
    std::cout << "Warning: Unable to save camera parameters in xml since pugixml 3rdparty is not enabled" << std::endl;
#endif

    if (opt_display) {
#if !(defined(VISP_HAVE_DISPLAY))
      std::cout << "No image viewer is available..." << std::endl;
      opt_display = false;
#else
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
      display = vpDisplayFactory::createDisplay(I);
      display2 = vpDisplayFactory::createDisplay(I2);
      display_merge = vpDisplayFactory::createDisplay(I_merge);
#else
      display = vpDisplayFactory::allocateDisplay(I);
#endif
#endif
    }

    vpImageQueue<vpRGBa> image_queue(opt_seqname, opt_record_mode);
    vpImageStorageWorker<vpRGBa> image_storage_worker(std::ref(image_queue));
    std::thread image_storage_thread(&vpImageStorageWorker<vpRGBa>::run, &image_storage_worker);

    using namespace cv;

    const float markerLength = 8e-2;
    Mat objPoints(4, 1, CV_64FC3);
    objPoints.ptr<Vec3d>(0)[0] = Vec3d(-markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<Vec3d>(0)[1] = Vec3d(markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<Vec3d>(0)[2] = Vec3d(markerLength/2.f, -markerLength/2.f, 0);
    objPoints.ptr<Vec3d>(0)[3] = Vec3d(-markerLength/2.f, -markerLength/2.f, 0);

    Matx33d camMatrix = Matx33d::eye();
    camMatrix(0, 0) = cam.get_px(); camMatrix(0, 2) = cam.get_u0();
    camMatrix(1, 1) = cam.get_py(); camMatrix(1, 2) = cam.get_v0();
    Matx41d distCoeffs = Matx41d::zeros();
    // distCoeffs(0, 0) = cam.get_kud();
    // distCoeffs(1, 0) = cam.get_kdu();

    vpDetectorAprilTag::vpAprilTagFamily tag_family = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod pose_estimation_method = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;

    float quad_decimate = 1.0;
    vpDetectorAprilTag detector(tag_family);
    bool display_tag = true;

    vpImage<unsigned char> I_gray;
    std::vector<double> times1, times2;
    vpImage<vpRGBa> O1, O2;

    const std::string output_dir = "test_p3p/" + vpTime::getDateTime("%Y-%m-%d_%H.%M.%S");
    if (save) {
      vpIoTools::makeDirectory(output_dir);
    }

    int nb_images = 0;
    bool quit = false;
    int iter = 0;
    while (!quit) {
      double t = vpTime::measureTimeMs();
      g.acquire(I);
      I2 = I;

      // std::cout << "\n\n" << iter++ << ")" << std::endl;

      vpDisplay::display(I);
      vpDisplay::display(I2);

      quit = image_queue.record(I);

      vpImageConvert::convert(I, I_gray);
      detector.detect(I_gray);
      std::vector<std::vector<vpImagePoint> > tagsCorners = detector.getTagsCorners();

      // std::cout << std::endl;
      for (size_t idx = 0; idx < tagsCorners.size(); idx++) {
        Matx31d rvec, tvec;
        std::vector<cv::Point2d> tagCorner;
        toImagePoints(tagsCorners[idx], tagCorner);

        double start = vpTime::measureTimeMs();
        // bool detect_1 = solvePnP(objPoints, tagCorner, camMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_P3P);
        bool detect_1 = solvePnP(objPoints, tagCorner, camMatrix, cv::noArray(), rvec, tvec, false, cv::SOLVEPNP_P3P);
        double end = vpTime::measureTimeMs();

        vpHomogeneousMatrix cMo;
        if (detect_1) {
          cMo.buildFrom(vpTranslationVector(tvec(0), tvec(1), tvec(2)), vpThetaUVector(rvec(0), rvec(1), rvec(2)));
          vpDisplay::displayFrame(I, cMo, cam, 0.04*1.5, vpColor::none, 3);
          // std::cout << "\nUSAC P3P, estimated cMo:\n" << cMo << std::endl;
        }

        times1.push_back((end - start));
        // std::stringstream ss;
        // ss << "Computation time: " << std::setprecision(3) << (end - start) << " ms";
        // vpDisplay::displayText(I, I.getHeight() - 20, I.getWidth() - 200, ss.str(), vpColor::red);


        // 2
        start = vpTime::measureTimeMs();
        // bool detect_2 = solvePnP(objPoints, tagCorner, camMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_AP3P);
        bool detect_2 = solvePnP(objPoints, tagCorner, camMatrix, cv::noArray(), rvec, tvec, false, cv::SOLVEPNP_AP3P);
        end = vpTime::measureTimeMs();

        if (detect_2) {
          cMo.buildFrom(vpTranslationVector(tvec(0), tvec(1), tvec(2)), vpThetaUVector(rvec(0), rvec(1), rvec(2)));
          vpDisplay::displayFrame(I2, cMo, cam, 0.04*1.5, vpColor::none, 3);
          // std::cout << "AP3P:\n" << cMo << std::endl;
        }

        times2.push_back((end - start));
        // ss.str("");
        // ss << "Computation time: " << std::setprecision(3) << (end - start) << " ms";
        // vpDisplay::displayText(I2, I2.getHeight() - 20, I2.getWidth() - 200, ss.str(), vpColor::red);
      }

      std::stringstream ss;
      ss << "Acquisition time: " << std::setprecision(3) << vpTime::measureTimeMs() - t << " ms";
      vpDisplay::displayText(I, I.getHeight() - 20, 10, ss.str(), vpColor::red);
      vpDisplay::flush(I);
      vpDisplay::flush(I2);

      vpDisplay::getImage(I, O1);
      vpDisplay::getImage(I2, O2);
      I_merge.insert(O1, vpImagePoint());
      I_merge.insert(O2, vpImagePoint(0, O1.getWidth()));
      vpDisplay::display(I_merge);
      vpDisplay::flush(I_merge);

      if (save) {
        std::ostringstream oss;
        oss << output_dir << "/" << "img_%04d.jpg";

        char filepath[FILENAME_MAX];
        snprintf(filepath, FILENAME_MAX, oss.str().c_str(), nb_images);
        vpImageIo::write(I_merge, filepath);
      }

      nb_images++;
    }
    image_queue.cancel();
    image_storage_thread.join();

    std::cout << "\nNb images: " << nb_images << " ; nb data: " << times1.size() << std::endl;
    std::cout << "P3P, mean: " << vpMath::getMean(times1) << " ms ; median: " << vpMath::getMedian(times1) <<
      " ms ; std: " << vpMath::getStdev(times1) << std::endl;
    std::cout << "AP3P, mean: " << vpMath::getMean(times2) << " ms ; median: " << vpMath::getMedian(times2) <<
      " ms ; std: " << vpMath::getStdev(times2) << std::endl;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (display != nullptr) {
    delete display;
  }
#endif
#else
  (void)argc;
  (void)argv;
#if !(defined(VISP_HAVE_REALSENSE) || defined(VISP_HAVE_REALSENSE2))
  std::cout << "Install librealsense version > 2.31.0, configure and build ViSP again to use this example" << std::endl;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  std::cout << "This tutorial should be built with c++11 support" << std::endl;
#endif
#endif
}
