//! \example tutorial-apriltag-detector.cpp
//! [Include]
#include <visp3/detection/vpDetectorAprilTag.h>
//! [Include]
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/vision/vpPose.h>

#include <visp3/sensor/vpRealSense.h>

int main(int argc, const char** argv) {
  bool poseFromHomography = false;
  double tagSize = 0.02;
  bool displayPose = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--pose_homography") {
      poseFromHomography = true;
    } else if (std::string(argv[i]) == "--tag_size" && i+1 < argc) {
      tagSize = atof(argv[i+1]);
    } else if (std::string(argv[i]) == "--display_pose") {
      displayPose = true;
    }
  }

  //! [Macro defined]
#if defined(VISP_HAVE_APRILTAG) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  //! [Macro defined]
  try {
    vpRealSense realsense;
    realsense.setStreamSettings(rs::stream::color, vpRealSense::vpRsStreamParams(640, 480, rs::format::rgba8, 30));
    realsense.setEnableStream(rs::stream::depth, false);
    realsense.setEnableStream(rs::stream::infrared, false);
    realsense.open();

    vpImage<vpRGBa> I_color(480, 640);
    vpImage<unsigned char> I(480, 640);

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
#endif

    vpCameraParameters cam = realsense.getCameraParameters(rs::stream::color);
    std::cout << "cam:\n" << cam << std::endl;
    //! [Create base detector]
    vpDetectorBase *detector = new vpDetectorAprilTag;
    //! [Create base detector]

    std::vector<double> time_vec;
    while (true) {
      realsense.acquire((unsigned char *) I_color.bitmap, NULL, NULL, NULL);
      vpImageConvert::convert(I_color, I);

      vpDisplay::display(I);

      std::vector<vpHomogeneousMatrix> cMo_vec;
      dynamic_cast<vpDetectorAprilTag*>(detector)->setPoseFromHomography(poseFromHomography);

      double t = vpTime::measureTimeMs();
      dynamic_cast<vpDetectorAprilTag*>(detector)->detect(I, tagSize, cam, cMo_vec);
      t = vpTime::measureTimeMs() - t;
      time_vec.push_back(t);

      std::stringstream ss;
      ss << "Detection time: " << t << " ms for " << detector->getNbObjects() << " tags";
      vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);

      if (displayPose) {
        for (size_t i = 0; i < cMo_vec.size() ; i++) {
  //        std::cout << "cMo:\n" << cMo_vec[i] << std::endl;
          vpDisplay::displayFrame(I, cMo_vec[i], cam, tagSize, vpColor::none, 3);
        }
      } else {
        for(size_t i=0; i < detector->getNbObjects(); i++) {
          //! [Parse detected codes]
          //! [Get location]
          std::vector<vpImagePoint> p = detector->getPolygon(i);
          vpRect bbox = detector->getBBox(i);
          //! [Get location]
          vpDisplay::displayRectangle(I, bbox, vpColor::green);
          //! [Get message]
          vpDisplay::displayText(I, (int)(bbox.getTop()-10), (int)bbox.getLeft(),
                                 "Message: \"" + detector->getMessage(i) + "\"",
                                 vpColor::red);
          //! [Get message]
          for(size_t j=0; j < p.size(); j++) {
            vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
            std::ostringstream number;
            number << j;
            vpDisplay::displayText(I, p[j]+vpImagePoint(15,5), number.str(), vpColor::blue);
          }
        }
      }

      vpDisplay::flush(I);

      vpImage<vpRGBa> O;
      vpDisplay::getImage(I, O);
      vpImageIo::write(O, "AprilTag_test.jpg");


      if (vpDisplay::getClick(I, false)) {
        break;
      }
    }

    std::cout << "Mean / Median / Std detection time: " << vpMath::getMean(time_vec) << " ms ; " << vpMath::getMedian(time_vec)
              << " ms ; " << vpMath::getStdev(time_vec) << " ms" << std::endl;



//    std::ostringstream legend;
//    legend << detector->getNbObjects() << " bar code detected";
//    vpDisplay::displayText(I, (int)I.getHeight()-30, 10, legend.str(), vpColor::red);

//    //! [Parse detected codes]
//    if (status) {
//      for(size_t i=0; i < detector->getNbObjects(); i++) {
//        //! [Parse detected codes]
//        //! [Get location]
//        std::vector<vpImagePoint> p = detector->getPolygon(i);
//        vpRect bbox = detector->getBBox(i);
//        //! [Get location]
//        vpDisplay::displayRectangle(I, bbox, vpColor::green);
//        //! [Get message]
//        vpDisplay::displayText(I, (int)(bbox.getTop()-10), (int)bbox.getLeft(),
//                               "Message: \"" + detector->getMessage(i) + "\"",
//                               vpColor::red);
//        //! [Get message]
//        for(size_t j=0; j < p.size(); j++) {
//          vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
//          std::ostringstream number;
//          number << j;
//          vpDisplay::displayText(I, p[j]+vpImagePoint(15,5), number.str(), vpColor::blue);
//        }
//      }

//      vpDisplay::displayText(I, (int)I.getHeight()-15, 10, "A click to quit...", vpColor::red);
//      vpDisplay::flush(I);
//      vpDisplay::getClick(I);
//    }
    delete detector;
  }
  catch(const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
  }
#else
  (void)argc;
  (void)argv;
#endif
}
