//! \example tutorial-mb-generic-tracker-stereo.cpp
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpIoTools.h>
//! [Include]
#include <visp3/mbt/vpMbGenericTracker.h>
//! [Include]
#include <visp3/io/vpVideoReader.h>


int main(int argc, char** argv)
{
  // OpenCV is required here by the video reader
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100)
  try {
    std::string opt_videoname_left = "teabox_left.mpg";
    std::string opt_videoname_right = "teabox_right.mpg";
    int opt_tracker = vpMbGenericTracker::EDGE_TRACKER;

    for (int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--name" && i+2 < argc) {
        opt_videoname_left = std::string(argv[i+1]);
        opt_videoname_right = std::string(argv[i+2]);
      } else if (std::string(argv[i]) == "--tracker" && i+1 < argc) {
        opt_tracker = atoi(argv[i+1]);
      } else if (std::string(argv[i]) == "--help") {
        std::cout << "\nUsage: " << argv[0] << " [--name <video name left> <video name right>] "
                     "[--tracker <1=egde|2=klt|3=hybrid>] [--help]\n" << std::endl;
        return 0;
      }
    }
    std::string parentname = vpIoTools::getParent(opt_videoname_left);
    std::string objectname_left = vpIoTools::getNameWE(opt_videoname_left);
    std::string objectname_right = vpIoTools::getNameWE(opt_videoname_right);

    if(! parentname.empty()) {
      objectname_left = parentname + "/" + objectname_left;
    }

    std::cout << "Video name: " << opt_videoname_left << " ; " << opt_videoname_right << std::endl;
    std::cout << "Tracker requested config files: " << objectname_left
              << ".[init, cao]" << " and " << objectname_right
              << ".[init, cao]" << std::endl;
    std::cout << "Tracker optional config files: " << opt_videoname_left << ".ppm"
              << " and " << opt_videoname_right << ".ppm" << std::endl;

    //! [Image]
    vpImage<unsigned char> I_left, I_right;
    //! [Image]

    vpVideoReader g_left, g_right;
    g_left.setFileName(opt_videoname_left);
    g_left.open(I_left);
    g_right.setFileName(opt_videoname_right);
    g_right.open(I_right);

    vpDisplay *display_left = NULL, *display_right = NULL;
#if defined(VISP_HAVE_X11)
    display_left = new vpDisplayX;
    display_right = new vpDisplayX;
#elif defined(VISP_HAVE_GDI)
    display_left = new vpDisplayGDI;
    display_right = new vpDisplayGDI;
#else
    display_left = new vpDisplayOpenCV;
    display_right = new vpDisplayOpenCV;
#endif
    display_left->setDownScalingFactor(vpDisplay::SCALE_AUTO);
    display_right->setDownScalingFactor(vpDisplay::SCALE_AUTO);
    display_left->init(I_left, 100, 100, "Model-based tracker (Left)");
    display_right->init(I_right, 110 + (int) I_left.getWidth(), 100, "Model-based tracker (Right)");

    //! [Constructor]
    vpMbTracker *tracker = new vpMbGenericTracker(2, opt_tracker);

#if !defined (VISP_HAVE_MODULE_KLT)
    if (opt_tracker >= 2) {
      std::cout << "klt and hybrid model-based tracker are not available since visp_klt module is missing" << std::endl;
      return 0;
    }
#endif
    //! [Constructor]

    //! [Load config file]
    dynamic_cast<vpMbGenericTracker*>(tracker)->loadConfigFile(objectname_left + ".xml", objectname_right + ".xml");
    //! [Load config file]

    //! [Get camera parameters]
    vpCameraParameters cam_left, cam_right;
    dynamic_cast<vpMbGenericTracker*>(tracker)->getCameraParameters(cam_left, cam_right);
    //! [Get camera parameters]

    //! [Load cao]
    tracker->loadModel("teabox.cao");
    //! [Load cao]
    //! [Set display features]
    tracker->setDisplayFeatures(true);
    //! [Set display features]

    vpHomogeneousMatrix cRightMcLeft;
    std::ifstream file_cRightMcLeft("cRightMcLeft.txt");
    cRightMcLeft.load(file_cRightMcLeft);

    //! [Set camera transformation matrix]
    std::map<std::string, vpHomogeneousMatrix> mapOfCameraTransformationMatrix;
    mapOfCameraTransformationMatrix["Camera1"] = vpHomogeneousMatrix();
    mapOfCameraTransformationMatrix["Camera2"] = cRightMcLeft;

    dynamic_cast<vpMbGenericTracker*>(tracker)->setCameraTransformationMatrix(mapOfCameraTransformationMatrix);
    //! [Set camera transformation matrix]

#ifndef VISP_HAVE_XML2
    std::cout << "\n**********************************************************\n"
              << "Warning: we are not able to load the tracker settings from\n"
              << "the xml config files since ViSP is not build with libxml2\n"
              << "3rd party. As a consequence, the tracking may fail !"
              << "\n**********************************************************\n"
              << std::endl;
#endif

    //! [Init]
    dynamic_cast<vpMbGenericTracker*>(tracker)->initClick(I_left, I_right, objectname_left + ".init", objectname_right + ".init", true);
    //! [Init]

    //! [cMo]
    vpHomogeneousMatrix cLeftMo, cRightMo;
    //! [cMo]
    while(!g_left.end() && !g_right.end()) {
      g_left.acquire(I_left);
      g_right.acquire(I_right);

      vpDisplay::display(I_left);
      vpDisplay::display(I_right);

      //! [Track]
      dynamic_cast<vpMbGenericTracker*>(tracker)->track(I_left, I_right);
      //! [Track]

      //! [Get pose]
      dynamic_cast<vpMbGenericTracker*>(tracker)->getPose(cLeftMo, cRightMo);
      //! [Get pose]

      //! [Display]
      dynamic_cast<vpMbGenericTracker*>(tracker)->display(I_left, I_right, cLeftMo, cRightMo, cam_left, cam_right, vpColor::red, 2);
      //! [Display]

      vpDisplay::displayFrame(I_left, cLeftMo, cam_left, 0.025, vpColor::none, 3);
      vpDisplay::displayFrame(I_right, cRightMo, cam_right, 0.025, vpColor::none, 3);
      vpDisplay::displayText(I_left, 10, 10, "A click to exit...", vpColor::red);

      vpDisplay::flush(I_left);
      vpDisplay::flush(I_right);

      if (vpDisplay::getClick(I_left, false)) {
        break;
      }
    }
    vpDisplay::getClick(I_left);

    //! [Cleanup]
    delete display_left;
    delete display_right;
    delete tracker;
    //! [Cleanup]
  }
  catch(vpException &e) {
    std::cerr << "Catch a ViSP exception: " << e.getMessage() << std::endl;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Install OpenCV and rebuild ViSP to use this example." << std::endl;
#endif
}
