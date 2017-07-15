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


int main(int argc, const char** argv)
{
  vpMatrix M(3,2);
  M[0][0] = 1;   M[0][1] = 6;
  M[1][0] = 2;   M[1][1] = 8;
  M[2][0] = 0.5; M[2][1] = 9;
  vpColVector w;
  vpMatrix V, Sigma, U = M;
  U.svd(w, V);
  // Construct the diagonal matrix from the singular values
  Sigma.diag(w);
  // Reconstruct the initial matrix using the decomposition
  vpMatrix Mrec =  U * Sigma * V.t();

  std::cout << "M:\n" << M << std::endl;
  std::cout << "U:\n" << U << std::endl;
  std::cout << "Mrec:\n" << Mrec << std::endl;


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
    double tagSize = 0.02;
    //! [Create base detector]
    vpDetectorBase *detector = new vpDetectorAprilTag;
    //! [Create base detector]

    bool poseFromHomography = false;
    while (true) {
      realsense.acquire((unsigned char *) I_color.bitmap, NULL, NULL, NULL);
      vpImageConvert::convert(I_color, I);

      vpDisplay::display(I);

      std::vector<vpHomogeneousMatrix> cMo_vec;
      if (poseFromHomography) {
        dynamic_cast<vpDetectorAprilTag*>(detector)->detect(I, tagSize, cam, cMo_vec);
      } else {
        std::vector<std::vector<vpPoint> > pts_vec;
        dynamic_cast<vpDetectorAprilTag*>(detector)->detect(I, tagSize, cam, pts_vec);

        for (size_t i = 0; i < pts_vec.size(); i++) {
          vpPose pose;
          pose.addPoints(pts_vec[i]);

          vpHomogeneousMatrix cMo;
          if (pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo)) {
//            vpMatrix R(3,3), U;
//            for (unsigned int i = 0; i < 3; i++) {
//              for (unsigned int j = 0; j < 3; j++) {
//                R[i][j] = cMo[i][j];
//              }
//            }
//            U = R;

//            vpColVector w;
//            vpMatrix V;
//            U.svd(w, V);
//            R = U*V.t();

//            for (unsigned int i = 0; i < 3; i++) {
//              for (unsigned int j = 0; j < 3; j++) {
//                cMo[i][j] = R[i][j];
//              }
//            }

            cMo_vec.push_back(cMo);
          }
        }
      }

      for (size_t i = 0; i < cMo_vec.size(); i++) {
//        std::cout << "cMo:\n" << cMo_vec[i] << std::endl;
        vpDisplay::displayFrame(I, cMo_vec[i], cam, tagSize, vpColor::none, 3);
      }

      vpDisplay::flush(I);

      if (vpDisplay::getClick(I, false)) {
        break;
      }
    }



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
