/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
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
 * Description:
 * Test keypoint matching and pose estimation.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020301)

#include <visp3/vision/vpKeyPoint.h>
#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/mbt/vpMbEdgeTracker.h>
#include <visp3/io/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"cdt:h"

void usage(const char *name, const char *badparam);
bool getOptions(int argc, const char **argv, bool &click_allowed, bool &display, int &testNb);

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Test keypoints matching.\n\
\n\
SYNOPSIS\n\
  %s [-c] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               \n\
\n\
  -c\n\
     Disable the mouse click. Useful to automaze the \n\
     execution of this program without humain intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -t <test> \n\
     Regression test number.\n\
\n\
  -h\n\
     Print the help.\n");

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param click_allowed : Mouse click activation.
  \param display : Display activation.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, bool &click_allowed, bool &display, int &testNb)
{
  const char *optarg_;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'c': click_allowed = false; break;
    case 'd': display = false; break;
    case 't': testNb = atoi(optarg_); break;

    case 'h': usage(argv[0], NULL); return false; break;

    default:
      usage(argv[0], optarg_);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

void learn_on_images(const std::string &dirname, const std::string &filename, const vpHomogeneousMatrix &cMo,
                     vpMbEdgeTracker &tracker, vpKeyPoint &keypoints, const int id) {
  vpImage<unsigned char> I;

  //Read image
  std::string filenameRef = vpIoTools::createFilePath(dirname, filename);
  vpImageIo::read(I, filenameRef);

  //Init pose at image
  tracker.initFromPose(I, cMo);

  //Detect keypoints on the image
  std::vector<cv::KeyPoint> trainKeyPoints;
  double elapsedTime;
  keypoints.detect(I, trainKeyPoints, elapsedTime);

  //Keep only keypoints on the cube
  std::vector<vpPolygon> polygons;
  std::vector<std::vector<vpPoint> > roisPt;
  std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair = tracker.getPolygonFaces(true, true, true); //To detect an issue with CI
  polygons = pair.first;
  roisPt = pair.second;

  vpCameraParameters cam;
  tracker.getCameraParameters(cam);

  //Compute the 3D coordinates
  std::vector<cv::Point3f> points3f;
  vpKeyPoint::compute3DForPointsInPolygons(cMo, cam, trainKeyPoints, polygons, roisPt, points3f);

  //Build the reference keypoints
  keypoints.buildReference(I, trainKeyPoints, points3f, true, id);
}

/*!
  \example testKeyPoint-2.cpp

  \brief   Test keypoint matching and pose estimation.
*/
int main(int argc, const char ** argv) {
  try {
    std::string env_ipath;
    bool opt_click_allowed = true;
    bool opt_display = true;
    int test = 0;

    // Read the command line options
    if (getOptions(argc, argv, opt_click_allowed, opt_display, test) == false) {
      exit (-1);
    }

    //Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    if(env_ipath.empty()) {
      std::cerr << "Please set the VISP_INPUT_IMAGE_PATH environment variable value." << std::endl;
      return -1;
    }

    vpImage<unsigned char> I;

    //Set the path location of the image sequence
    std::string dirname = vpIoTools::createFilePath(env_ipath, "ViSP-images/mbt/cube");

    //Build the name of the image files
    std::string filenameRef = vpIoTools::createFilePath(dirname, "image0000.pgm");
    vpImageIo::read(I, filenameRef);
    std::string filenameCur = vpIoTools::createFilePath(dirname, "image%04d.pgm");

#if defined VISP_HAVE_X11
    vpDisplayX display;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display;
#else
    vpDisplayOpenCV display;
#endif

    if (opt_display) {
      display.setDownScalingFactor(vpDisplay::SCALE_AUTO);
      display.init(I, 0, 0, "ORB keypoints matching and pose estimation");
    }

    vpCameraParameters cam;
    vpMbEdgeTracker tracker;
    //Load config for tracker
    std::string tracker_config_file = vpIoTools::createFilePath(env_ipath, "ViSP-images/mbt/cube.xml");

    bool usexml = false;
#ifdef VISP_HAVE_XML2
    tracker.loadConfigFile(tracker_config_file);
    tracker.getCameraParameters(cam);

    usexml = true;
#endif
    if (! usexml) {
      vpMe me;
      me.setMaskSize(5);
      me.setMaskNumber(180);
      me.setRange(8);
      me.setThreshold(10000);
      me.setMu1(0.5);
      me.setMu2(0.5);
      me.setSampleStep(4);
      me.setNbTotalSample(250);
      tracker.setMovingEdge(me);
      cam.initPersProjWithoutDistortion(547.7367575, 542.0744058, 338.7036994, 234.5083345);
      tracker.setCameraParameters(cam);
      tracker.setNearClippingDistance(0.01);
      tracker.setFarClippingDistance(100.0);
      tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
    }

    tracker.setAngleAppear(vpMath::rad(89));
    tracker.setAngleDisappear(vpMath::rad(89));

    //Load CAO model
    std::string cao_model_file = vpIoTools::createFilePath(env_ipath, "ViSP-images/mbt/cube.cao");
    tracker.loadModel(cao_model_file);

    //Get the init pose
    vpHomogeneousMatrix cMo;

    //Init keypoints
    vpKeyPoint keypoints("ORB", "ORB", "BruteForce-Hamming");
#if (VISP_HAVE_OPENCV_VERSION >= 0x020400)
    //Bug when using LSH index with FLANN and OpenCV 2.3.1.
    //see http://code.opencv.org/issues/1741 (Bug #1741)
    keypoints.setMatcher("FlannBased");
#if (VISP_HAVE_OPENCV_VERSION < 0x030000)
      keypoints.setDetectorParameter("ORB", "nLevels", 1);
#else
      cv::Ptr<cv::ORB> orb_detector = keypoints.getDetector("ORB").dynamicCast<cv::ORB>();
      if(orb_detector != NULL) {
        orb_detector->setNLevels(1);
      }
#endif
#endif

    switch (test) {
      case 0:
        keypoints.loadLearningData("testKeyPoint-2_regression_ORB_xml/testKeyPoint-2_regression_ORB.xml", false);
        break;

      case 1:
        keypoints.loadLearningData("testKeyPoint-2_regression_ORB_bin/testKeyPoint-2_regression_ORB.bin", true);
        break;

      case 2:
        keypoints.loadLearningData("testKeyPoint-2_regression_ORB_no_img_xml/testKeyPoint-2_regression_ORB_no_img.xml", false);
        break;

      case 3:
        keypoints.loadLearningData("testKeyPoint-2_regression_ORB_no_img_bin/testKeyPoint-2_regression_ORB_no_img.bin", true);
        break;

      case 4:
        keypoints.loadLearningData("testKeyPoint-2_regression_ORB_part1_xml/testKeyPoint-2_regression_ORB_part1.xml", false, true);
        keypoints.loadLearningData("testKeyPoint-2_regression_ORB_part2_xml/testKeyPoint-2_regression_ORB_part2.xml", false, true);
        break;

      case 5:
        keypoints.loadLearningData("testKeyPoint-2_regression_ORB_part1_bin/testKeyPoint-2_regression_ORB_part1.bin", true, false);
        keypoints.loadLearningData("testKeyPoint-2_regression_ORB_part2_bin/testKeyPoint-2_regression_ORB_part2.bin", true, true);
        break;

      case 6:
        keypoints.loadLearningData("testKeyPoint-2_regression_ORB_part2_xml/testKeyPoint-2_regression_ORB_part2.xml", false, true);
        keypoints.loadLearningData("testKeyPoint-2_regression_ORB_part1_xml/testKeyPoint-2_regression_ORB_part1.xml", false, true);
        break;

      case 7:
        keypoints.loadLearningData("testKeyPoint-2_regression_ORB_part2_bin/testKeyPoint-2_regression_ORB_part2.bin", true, true);
        keypoints.loadLearningData("testKeyPoint-2_regression_ORB_part1_bin/testKeyPoint-2_regression_ORB_part1.bin", true, true);
        break;

      case 8:
        {
          keypoints.loadLearningData("testKeyPoint-2_regression_ORB_part1_xml/testKeyPoint-2_regression_ORB_part1.xml", false);

          vpHomogeneousMatrix cMo;
          cMo.buildFrom(0.02651282185, -0.03713587374, 0.6873765919, 2.314744454, 0.3492296488, -0.1226054828);
          learn_on_images(dirname, "image0150.pgm", cMo, tracker, keypoints, 2);

          cMo.buildFrom(0.02965448956, -0.07283091786, 0.7253526051, 2.300529617, -0.4286674806, 0.1788761025);
          learn_on_images(dirname, "image0200.pgm", cMo, tracker, keypoints, 3);
        }
        break;

      case 9:
        {
          keypoints.loadLearningData("testKeyPoint-2_regression_ORB_part1_bin/testKeyPoint-2_regression_ORB_part1.bin", true);

          vpHomogeneousMatrix cMo;
          cMo.buildFrom(0.02651282185, -0.03713587374, 0.6873765919, 2.314744454, 0.3492296488, -0.1226054828);
          learn_on_images(dirname, "image0150.pgm", cMo, tracker, keypoints, 2);

          cMo.buildFrom(0.02965448956, -0.07283091786, 0.7253526051, 2.300529617, -0.4286674806, 0.1788761025);
          learn_on_images(dirname, "image0200.pgm", cMo, tracker, keypoints, 3);
        }
        break;

      case 10:
        {
          keypoints.loadLearningData("testKeyPoint-2_regression_ORB_part2_xml/testKeyPoint-2_regression_ORB_part2.xml", false);

          vpHomogeneousMatrix cMo;
          cMo.buildFrom(0.02044769891, 0.1101505452, 0.5078963719, 2.063603907, 1.110231561, -0.4392789872);
          learn_on_images(dirname, "image0000.pgm", cMo, tracker, keypoints, 1);
        }
        break;

      case 11:
        {
          keypoints.loadLearningData("testKeyPoint-2_regression_ORB_part2_bin/testKeyPoint-2_regression_ORB_part2.bin", true);

          vpHomogeneousMatrix cMo;
          cMo.buildFrom(0.02044769891, 0.1101505452, 0.5078963719, 2.063603907, 1.110231561, -0.4392789872);
          learn_on_images(dirname, "image0000.pgm", cMo, tracker, keypoints, 1);
        }
        break;

      default:
        std::cerr << "Bad test number!" << std::endl;
        break;
    }



    //Init reader for getting the input image sequence
    vpVideoReader g;
    g.setFileName(filenameCur);
    g.open(I);
    g.acquire(I);

#if defined VISP_HAVE_X11
    vpDisplayX display2;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display2;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display2;
#else
    vpDisplayOpenCV display2;
#endif

    vpImage<unsigned char> IMatching;
    keypoints.createImageMatching(I, IMatching);

    if (opt_display) {
      display2.setDownScalingFactor(vpDisplay::SCALE_AUTO);
      display2.init(IMatching, 0, (int)I.getHeight()/vpDisplay::getDownScalingFactor(I) + 80, "IMatching");
    }

    bool opt_click = false;
    vpMouseButton::vpMouseButtonType button;
    while((opt_display && !g.end()) || (!opt_display && g.getFrameIndex() < 30)) {
      g.acquire(I);

      if(opt_display) {
        vpDisplay::display(I);

        //Display image matching
        keypoints.insertImageMatching(I, IMatching);

        vpDisplay::display(IMatching);
      }

      //Match keypoints and estimate the pose
      if(keypoints.matchPoint(I, cam, cMo)) {
        tracker.setCameraParameters(cam);
        tracker.setPose(I, cMo);

        if(opt_display) {
          tracker.display(I, cMo, cam, vpColor::red, 2);
          vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);

          std::vector<vpImagePoint> ransacInliers = keypoints.getRansacInliers();
          std::vector<vpImagePoint> ransacOutliers = keypoints.getRansacOutliers();

          for(std::vector<vpImagePoint>::const_iterator it = ransacInliers.begin(); it != ransacInliers.end(); ++it) {
            vpDisplay::displayCircle(I, *it, 4, vpColor::green);
            vpImagePoint imPt(*it);
            imPt.set_u(imPt.get_u() + I.getWidth());
            imPt.set_v(imPt.get_v() + I.getHeight());
            vpDisplay::displayCircle(IMatching, imPt, 4, vpColor::green);
          }

          for(std::vector<vpImagePoint>::const_iterator it = ransacOutliers.begin(); it != ransacOutliers.end(); ++it) {
            vpDisplay::displayCircle(I, *it, 4, vpColor::red);
            vpImagePoint imPt(*it);
            imPt.set_u(imPt.get_u() + I.getWidth());
            imPt.set_v(imPt.get_v() + I.getHeight());
            vpDisplay::displayCircle(IMatching, imPt, 4, vpColor::red);
          }

          keypoints.displayMatching(I, IMatching);

          //Display model in the correct sub-image in IMatching
          vpCameraParameters cam2;
          cam2.initPersProjWithoutDistortion(cam.get_px(), cam.get_py(),
              cam.get_u0() + I.getWidth(), cam.get_v0() + I.getHeight());
          tracker.setCameraParameters(cam2);
          tracker.setPose(IMatching, cMo);
          tracker.display(IMatching, cMo, cam2, vpColor::red, 2);
          vpDisplay::displayFrame(IMatching, cMo, cam2, 0.025, vpColor::none, 3);
        }
      }

      if(opt_display) {
        vpDisplay::flush(I);
        vpDisplay::flush(IMatching);
      }

      if (opt_click_allowed && opt_display) {
        //Click requested to process next image
        if(opt_click) {
          vpDisplay::getClick(I, button, true);
          if(button == vpMouseButton::button3) {
            opt_click = false;
          }
        } else {
          //Use right click to enable/disable step by step tracking
          if(vpDisplay::getClick(I, button, false)) {
            if (button == vpMouseButton::button3) {
              opt_click = true;
            }
            else if(button == vpMouseButton::button1) {
              break;
            }
          }
        }
      }
    }

  } catch(vpException &e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  std::cout << "testKeyPoint-2 is ok !" << std::endl;
  return 0;
}
#else
int main() {
  std::cerr << "You need OpenCV library." << std::endl;

  return 0;
}

#endif
