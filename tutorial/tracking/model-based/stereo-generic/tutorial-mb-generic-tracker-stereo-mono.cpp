//! \example tutorial-mb-generic-tracker-stereo-mono.cpp
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
    std::string opt_videoname = "teabox.mpg";
    int opt_tracker = vpMbGenericTracker::EDGE_TRACKER;

    for (int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--name" && i+1 < argc)
        opt_videoname = std::string(argv[i+1]);
      else if (std::string(argv[i]) == "--tracker" && i+1 < argc)
        opt_tracker = atoi(argv[i+1]);
      else if (std::string(argv[i]) == "--help") {
        std::cout << "\nUsage: " << argv[0] << " [--name <video name>] [--tracker <1=egde|2=keypoint|3=hybrid>] [--help]\n" << std::endl;
        return 0;
      }
    }
    std::string parentname = vpIoTools::getParent(opt_videoname);
    std::string objectname = vpIoTools::getNameWE(opt_videoname);

    if(! parentname.empty()) {
       objectname = parentname + "/" + objectname;
    }

    std::cout << "Video name: " << opt_videoname << std::endl;
    std::cout << "Tracker requested config files: " << objectname
              << ".[init, cao]" << std::endl;
    std::cout << "Tracker optional config files: " << objectname << ".[ppm]" << std::endl;

    //! [Image]
    vpImage<unsigned char> I;
    vpCameraParameters cam;
    //! [Image]
    //! [cMo]
    vpHomogeneousMatrix cMo;
    //! [cMo]

    vpVideoReader g;
    g.setFileName(opt_videoname);
    g.open(I);

    vpDisplay *display = NULL;
#if defined(VISP_HAVE_X11)
    display = new vpDisplayX;
#elif defined(VISP_HAVE_GDI)
    display = new vpDisplayGDI;
#else
    display = new vpDisplayOpenCV;
#endif
    display->init(I, 100, 100, "Model-based tracker");

    //! [Constructor]
    vpMbTracker *tracker = new vpMbGenericTracker(1, opt_tracker);

#if !defined (VISP_HAVE_MODULE_KLT)
    if (opt_tracker >= 2) {
      std::cout << "klt and hybrid model-based tracker are not available since visp_klt module is missing" << std::endl;
      return 0;
    }
#endif
    //! [Constructor]

    //! [Set parameters]
    if (opt_tracker == 1 || opt_tracker == 3) {
      vpMe me;
      me.setMaskSize(5);
      me.setMaskNumber(180);
      me.setRange(8);
      me.setThreshold(10000);
      me.setMu1(0.5);
      me.setMu2(0.5);
      me.setSampleStep(4);
      dynamic_cast<vpMbGenericTracker*>(tracker)->setMovingEdge(me);
    }

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
    if (opt_tracker == 2 || opt_tracker == 3) {
      vpKltOpencv klt_settings;
      klt_settings.setMaxFeatures(300);
      klt_settings.setWindowSize(5);
      klt_settings.setQuality(0.015);
      klt_settings.setMinDistance(8);
      klt_settings.setHarrisFreeParameter(0.01);
      klt_settings.setBlockSize(3);
      klt_settings.setPyramidLevels(3);
      dynamic_cast<vpMbGenericTracker*>(tracker)->setKltOpencv(klt_settings);
      dynamic_cast<vpMbGenericTracker*>(tracker)->setMaskBorder(5);
    }
#endif

    //! [Set camera parameters]
    cam.initPersProjWithoutDistortion(839, 839, 325, 243);
    //! [Set camera parameters]
    tracker->setCameraParameters(cam);
    //! [Set parameters]

    //! [Load cao]
    tracker->loadModel(objectname + ".cao");
    //! [Load cao]
    //! [Set display]
    tracker->setDisplayFeatures(true);
    //! [Set display]
    //! [Init]
    tracker->initClick(I, objectname + ".init", true);
    //! [Init]

    while(! g.end()){
      g.acquire(I);
      vpDisplay::display(I);
      //! [Track]
      tracker->track(I);
      //! [Track]
      //! [Get pose]
      tracker->getPose(cMo);
      //! [Get pose]
      //! [Display]
      tracker->getCameraParameters(cam);
      tracker->display(I, cMo, cam, vpColor::red, 2, true);
      //! [Display]
      vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
      vpDisplay::displayText(I, 10, 10, "A click to exit...", vpColor::red);
      vpDisplay::flush(I);

      if (vpDisplay::getClick(I, false)) {
        break;
      }
    }
    vpDisplay::getClick(I);
    //! [Cleanup]
    delete display;
    delete tracker;
    //! [Cleanup]
  }
  catch(vpException &e) {
    std::cerr << "Catch a ViSP exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Install OpenCV and rebuild ViSP to use this example." << std::endl;
#endif
}
