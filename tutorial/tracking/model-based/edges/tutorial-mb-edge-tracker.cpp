#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/mbt/vpMbEdgeKltTracker.h>
#include <visp3/io/vpVideoReader.h>

int main(int argc, char **argv)
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100)
  try {
    std::string videoname = "teabox.mpg";
    int tracker_type = 0;

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--name")
        videoname = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--help") {
        std::cout << "\nUsage: " << argv[0] << " [--name <video name>] [--help]\n" << std::endl;
        return 0;
      } else if (std::string(argv[i]) == "--tracker_type" && i+1 < argc) {
        tracker_type = atoi(argv[i+1]);
      }
    }
    std::string parentname = vpIoTools::getParent(videoname);
    std::string objectname = vpIoTools::getNameWE(videoname);

    if (!parentname.empty())
      objectname = parentname + "/" + objectname;

    std::cout << "Video name: " << videoname << std::endl;
    std::cout << "Tracker requested config files: " << objectname << ".[init,"
#ifdef VISP_HAVE_XML2
              << "xml,"
#endif
              << "cao or wrl]" << std::endl;
    std::cout << "Tracker optional config files: " << objectname << ".[ppm]" << std::endl;
    std::cout << "tracker_type: " << tracker_type << std::endl;

    vpImage<unsigned char> I, I2;

    vpVideoReader g;
    g.setFileName(videoname);
    g.open(I);
    I2 = I;

    vpDisplayX display, display2;
    display.init(I, 100, 100, "Model-based edge tracker");
    display2.init(I2, 700, 100, "Generic");

    vpMbTracker *tracker;
    std::vector<int> tracker_types = {vpMbGenericTracker::EDGE_TRACKER};
    if (tracker_type == 0) {
      tracker = new vpMbEdgeTracker;
    } else if (tracker_type == 1) {
      tracker = new vpMbKltTracker;
      tracker_types[0] = vpMbGenericTracker::KLT_TRACKER;
    } else {
      tracker = new vpMbEdgeKltTracker;
      tracker_types[0] = vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER;
    }
    vpMbGenericTracker tracker_generic(tracker_types);

    tracker->loadConfigFile(objectname + ".xml");
    tracker->loadModel(objectname + ".cao");
    tracker->setDisplayFeatures(true);
    tracker->setCovarianceComputation(true);
    tracker->initClick(I, objectname + ".init", true);

    tracker_generic.loadConfigFile(objectname + ".xml");
    tracker_generic.loadModel(objectname + ".cao");
    tracker_generic.setDisplayFeatures(true);
    tracker_generic.setCovarianceComputation(true);
    tracker_generic.initClick(I2, objectname + ".init", true);

    vpPlot plot1(2, 700, 700, 0, 0, "MBT");
    plot1.initGraph(0, 3);
    plot1.initGraph(1, 3);

    vpPlot plot2(2, 700, 700, 700, 0, "Generic");
    plot2.initGraph(0, 3);
    plot2.initGraph(1, 3);

    int iter = 0;
    std::vector<double> errors_cov_trans, errors_cov_rot;
    while (!g.end()) {
      g.acquire(I);
      I2 = I;

      vpDisplay::display(I);
      vpDisplay::display(I2);

      vpColVector cov_translation1(3), cov_rotation1(3);
      vpColVector cov_translation2(3), cov_rotation2(3);
      {
        tracker->track(I);

        vpHomogeneousMatrix cMo = tracker->getPose();
        vpCameraParameters cam;
        tracker->getCameraParameters(cam);
        tracker->display(I, cMo, cam, vpColor::red, 2);

        vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
        vpDisplay::displayText(I, 10, 10, "A click to exit...", vpColor::red);

        vpMatrix cov = tracker->getCovarianceMatrix();
        for (int i = 0; i < 3; i++) {
          cov_translation1[i] = cov[i][i];
          cov_rotation1[i] = cov[i+3][i+3];
        }

        plot1.plot(0, iter, cov_translation1);
        plot1.plot(1, iter, cov_rotation1);
      }

      {
        tracker_generic.track(I2);

        vpHomogeneousMatrix cMo = tracker_generic.getPose();
        vpCameraParameters cam;
        tracker_generic.getCameraParameters(cam);
        tracker_generic.display(I2, cMo, cam, vpColor::red, 2);

        vpDisplay::displayFrame(I2, cMo, cam, 0.025, vpColor::none, 3);
        vpDisplay::displayText(I2, 10, 10, "A click to exit...", vpColor::red);

        vpMatrix cov = tracker_generic.getCovarianceMatrix();
        for (int i = 0; i < 3; i++) {
          cov_translation2[i] = cov[i][i];
          cov_rotation2[i] = cov[i+3][i+3];
        }

        plot2.plot(0, iter, cov_translation2);
        plot2.plot(1, iter, cov_rotation2);
      }

      for (int i = 0; i < 3; i++) {
        errors_cov_trans.push_back( cov_translation1[i] - cov_translation2[i] );
        errors_cov_rot.push_back( cov_rotation1[i] - cov_rotation2[i] );
      }

      vpDisplay::flush(I);
      vpDisplay::flush(I2);

      if (vpDisplay::getClick(I, false) || vpDisplay::getClick(I2, false))
        break;

      iter++;
    }

    std::cout << "Mean error translation: " << vpMath::getMean(errors_cov_trans) << " ; Median: " << vpMath::getMedian(errors_cov_trans) << std::endl;
    std::cout << "Mean error rotation: " << vpMath::getMean(errors_cov_rot) << " ; Median: " << vpMath::getMedian(errors_cov_rot) << std::endl;

    vpDisplay::getClick(I);

    delete tracker;
  } catch (vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Install OpenCV and rebuild ViSP to use this example." << std::endl;
#endif
}
