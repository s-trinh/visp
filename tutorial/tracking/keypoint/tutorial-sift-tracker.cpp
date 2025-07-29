//! \example tutorial-klt-tracker.cpp
#include <iostream>

#include <visp3/core/vpConfig.h>

//! [Check 3rd party]
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO) && defined(HAVE_OPENCV_VIDEOIO)
//! [Check 3rd party]

//! [Include]
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/klt/vpSiftOpencv.h>
//! [Include]

int main(int argc, const char *argv[])
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  try {
    std::string opt_videoname = "video-postcard.mp4";
    unsigned int opt_subsample = 1;
    bool click = false;
    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--videoname") {
        opt_videoname = std::string(argv[++i]);
      }
      else if (std::string(argv[i]) == "--subsample") {
        opt_subsample = static_cast<unsigned int>(std::atoi(argv[++i]));
      }
      else if (std::string(argv[i]) == "--click") {
        click = true;
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "Usage: " << argv[0]
          << " [--videoname <video name>] [--subsample <scale factor>] [--click]"
          << " [--help] [-h]" << std::endl;
        return EXIT_SUCCESS;
      }
    }

    //! [Create reader]
    vpVideoReader reader;
    reader.setFileName(opt_videoname);
    //! [Create reader]

    //! [Acquire]
    vpImage<unsigned char> I, I_prev, Iacq;
    reader.acquire(Iacq);
    Iacq.subsample(opt_subsample, opt_subsample, I);
    //! [Acquire]
    I_prev = I;
    vpImage<unsigned char> I_match(I.getRows(), 2*I.getCols());

    //! [Convert to OpenCV image]
    cv::Mat cvI;
    vpImageConvert::convert(I, cvI);
    //! [Convert to OpenCV image]

    //! [Init display]
    vpDisplayOpenCV d(I, 0, 0, "SIFT tracking");
    vpDisplay::display(I);
    vpDisplay::flush(I);

    vpDisplayOpenCV d_match(I_match, 0, 0, "Matching");
    vpDisplay::display(I_match);
    vpDisplay::flush(I_match);
    //! [Init display]

    //! [Create tracker]
    vpSiftOpencv tracker;

    //! [Init tracker]
    tracker.initTracking(cvI);
    //! [Init tracker]

    //! [How many features]
    std::cout << "Tracker initialized with " << tracker.getNbFeatures() << " features" << std::endl;
    //! [How many features]

    //! [While loop]
    while (!reader.end()) {
      std::cout << "\n" << reader.getFrameIndex() << ")" << std::endl;

      double t = vpTime::measureTimeMs();
      reader.acquire(Iacq);
      Iacq.subsample(opt_subsample, opt_subsample, I);

      I_match.insert(I_prev, vpImagePoint(0, 0));
      I_match.insert(I, vpImagePoint(0, I_prev.getCols()));

      vpDisplay::display(I);
      vpDisplay::display(I_match);

      vpImageConvert::convert(I, cvI);

      tracker.track(cvI);
      tracker.display(I, vpColor::red);

      std::vector<cv::Point2f> features_prev = tracker.getPrevFeatures();
      std::vector<cv::Point2f> features_cur = tracker.getFeatures();
      for (size_t i = 0; i < features_prev.size(); i++) {
        vpImagePoint ip1(features_prev[i].y, features_prev[i].x);
        vpImagePoint ip2(features_cur[i].y, I_prev.getCols() + features_cur[i].x);
        vpDisplay::displayLine(I_match, ip1, ip2, vpColor::green);
      }

      vpDisplay::displayText(I, 10, 10, "Click to quit", vpColor::red);
      if (vpDisplay::getClick(I, false))
        break;

      vpDisplay::flush(I);
      vpDisplay::flush(I_match);
      if (!reader.isVideoFormat()) {
        vpTime::wait(t, 40);
      }
      vpDisplay::getClick(I, click);

      I_prev = I;
    }
    //! [While loop]

    //! [Wait click]
    vpDisplay::getClick(I);
    //! [Wait click]
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

#else

int main()
{
#if !defined(HAVE_OPENCV_HIGHGUI)
  std::cout << "This tutorial needs OpenCV highgui module that is missing." << std::endl;
#endif
#if !defined(HAVE_OPENCV_IMGPROC)
  std::cout << "This tutorial needs OpenCV imgproc module that is missing." << std::endl;
#endif
#if !defined(HAVE_OPENCV_VIDEO)
  std::cout << "This tutorial needs OpenCV video module that is missing." << std::endl;
#endif
#if !defined(HAVE_OPENCV_VIDEOIO)
  std::cout << "This tutorial needs OpenCV videoio module that is missing." << std::endl;
#endif
}

#endif
