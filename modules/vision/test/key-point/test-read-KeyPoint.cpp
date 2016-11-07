/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
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
 * Test saving / loading learning files for vpKeyPoint class.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

#include <iostream>
#include <iomanip>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020301)

#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/vision/vpKeyPoint.h>
#include <visp3/core/vpException.h>

// List of allowed command line options
#define GETOPTARGS	"cdo:hb:x:"

void usage(const char *name, const char *badparam, std::string opath, std::string user);
bool getOptions(int argc, const char **argv, std::string &opath, std::string user,
    std::string &bin_file, std::string &xml_file);

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(const char *name, const char *badparam, std::string opath, std::string user)
{
  fprintf(stdout, "\n\
Test save / load learning files for vpKeyPoint class.\n\
\n\
SYNOPSIS\n\
  %s [-c] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               \n\
\n\
  -o <output image path>                               %s\n\
     Set image output path.\n\
     From this directory, creates the \"%s\"\n\
     subdirectory depending on the username, where \n\
     learning files will be written.\n\
\n\
  -h\n\
     Print the help.\n",
     opath.c_str(), user.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param opath : Output image path.
  \param user : Username.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &opath, std::string user, std::string &bin_file, std::string &xml_file)
{
  const char *optarg_;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'c': break; //not used, to avoid error with default arguments ctest
    case 'd': break; //not used, to avoid error with default arguments ctest
    case 'o': opath = optarg_; break;
    case 'h': usage(argv[0], NULL, opath, user); return false; break;
    case 'b': bin_file = optarg_; break;
    case 'x': xml_file = optarg_; break;

    default:
      usage(argv[0], optarg_, opath, user); return false; break;
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, opath, user);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

/*!
  Compare two vectors of cv::KeyPoint.

  \param keypoints1 : First vectors of cv::KeyPoint.
  \param keypoints2 : Second vectors of cv::KeyPoint.

  \return True if the two vectors are identical, false otherwise.
*/
bool compareKeyPoints(const std::vector<cv::KeyPoint> &keypoints1, const std::vector<cv::KeyPoint> &keypoints2) {
  if(keypoints1.size() != keypoints2.size()) {
    return false;
  }

  for(size_t cpt = 0; cpt < keypoints1.size(); cpt++) {
    if(!vpMath::equal(keypoints1[cpt].angle, keypoints2[cpt].angle, std::numeric_limits<float>::epsilon())) {
      std::cerr << std::fixed << std::setprecision(9) << "keypoints1[cpt].angle=" << keypoints1[cpt].angle <<
          " ; keypoints2[cpt].angle=" << keypoints2[cpt].angle << std::endl;
      return false;
    }

    if(keypoints1[cpt].class_id != keypoints2[cpt].class_id) {
      std::cerr << "keypoints1[cpt].class_id=" << keypoints1[cpt].class_id << " ; keypoints2[cpt].class_id=" <<
          keypoints2[cpt].class_id << std::endl;
      return false;
    }

    if(keypoints1[cpt].octave != keypoints2[cpt].octave) {
      std::cerr << "keypoints1[cpt].octave=" << keypoints1[cpt].octave << " ; keypoints2[cpt].octave=" <<
          keypoints2[cpt].octave << std::endl;
      return false;
    }

    if(!vpMath::equal(keypoints1[cpt].pt.x, keypoints2[cpt].pt.x, std::numeric_limits<float>::epsilon())) {
      std::cerr << std::fixed << std::setprecision(9) << "keypoints1[cpt].pt.x=" << keypoints1[cpt].pt.x <<
          " ; keypoints2[cpt].pt.x=" << keypoints2[cpt].pt.x << std::endl;
      return false;
    }

    if(!vpMath::equal(keypoints1[cpt].pt.y, keypoints2[cpt].pt.y, std::numeric_limits<float>::epsilon())) {
      std::cerr << std::fixed << std::setprecision(9) << "keypoints1[cpt].pt.y=" << keypoints1[cpt].pt.y <<
          " ; keypoints2[cpt].pt.y=" << keypoints2[cpt].pt.y << std::endl;
      return false;
    }

    if(!vpMath::equal(keypoints1[cpt].response, keypoints2[cpt].response, std::numeric_limits<float>::epsilon())) {
      std::cerr << std::fixed << std::setprecision(9) << "keypoints1[cpt].response=" << keypoints1[cpt].response <<
          " ; keypoints2[cpt].response=" << keypoints2[cpt].response << std::endl;
      return false;
    }

    if(!vpMath::equal(keypoints1[cpt].size, keypoints2[cpt].size, std::numeric_limits<float>::epsilon())) {
      std::cerr << std::fixed << std::setprecision(9) << "keypoints1[cpt].size=" << keypoints1[cpt].size <<
          " ; keypoints2[cpt].size=" << keypoints2[cpt].size << std::endl;
      return false;
    }
  }

  return true;
}

/*!
  Compare two descriptors.

  \param descriptors1 : First descriptor.
  \param descriptors2 : Second descriptor.

  \return True if the two vectors are identical, false otherwise.
*/
bool compareDescriptors(const cv::Mat &descriptors1, const cv::Mat &descriptors2) {
  if(descriptors1.rows != descriptors2.rows || descriptors1.cols != descriptors2.cols ||
      descriptors1.type() != descriptors2.type()) {
    return false;
  }

  for(int i = 0; i < descriptors1.rows; i++) {
    for(int j = 0; j < descriptors1.cols; j++) {
      switch(descriptors1.type()) {
      case CV_8U:
        if(descriptors1.at<unsigned char>(i,j) != descriptors2.at<unsigned char>(i,j)) {
          std::cerr << "descriptors1.at<unsigned char>(i,j)=" << descriptors1.at<unsigned char>(i,j) <<
              " ; descriptors2.at<unsigned char>(i,j)=" << descriptors2.at<unsigned char>(i,j) << std::endl;
          return false;
        }
        break;

      case CV_8S:
        if(descriptors1.at<char>(i,j) != descriptors2.at<char>(i,j)) {
          std::cerr << "descriptors1.at<char>(i,j)=" << descriptors1.at<char>(i,j) <<
              " ; descriptors2.at<char>(i,j)=" << descriptors2.at<char>(i,j) << std::endl;
          return false;
        }
        break;

      case CV_16U:
        if(descriptors1.at<unsigned short>(i,j) != descriptors2.at<unsigned short>(i,j)) {
          std::cerr << "descriptors1.at<unsigned short>(i,j)=" << descriptors1.at<unsigned short>(i,j) <<
              " ; descriptors2.at<unsigned short>(i,j)=" << descriptors2.at<unsigned short>(i,j) << std::endl;
          return false;
        }
        break;

      case CV_16S:
        if(descriptors1.at<short>(i,j) != descriptors2.at<short>(i,j)) {
          std::cerr << "descriptors1.at<short>(i,j)=" << descriptors1.at<short>(i,j) <<
              " ; descriptors2.at<short>(i,j)=" << descriptors2.at<short>(i,j) << std::endl;
          return false;
        }
        break;

      case CV_32S:
        if(descriptors1.at<int>(i,j) != descriptors2.at<int>(i,j)) {
          std::cerr << "descriptors1.at<int>(i,j)=" << descriptors1.at<int>(i,j) <<
              " ; descriptors2.at<int>(i,j)=" << descriptors2.at<int>(i,j) << std::endl;
          return false;
        }
        break;

      case CV_32F:
        if(!vpMath::equal(descriptors1.at<float>(i,j), descriptors2.at<float>(i,j), std::numeric_limits<float>::epsilon())) {
          std::cerr << std::fixed << std::setprecision(9) << "descriptors1.at<float>(i,j)=" << descriptors1.at<float>(i,j)
              << " ; descriptors2.at<float>(i,j)=" << descriptors2.at<float>(i,j) << std::endl;
          return false;
        }
        break;

      case CV_64F:
        if(!vpMath::equal(descriptors1.at<double>(i,j), descriptors2.at<double>(i,j), std::numeric_limits<double>::epsilon())) {
          std::cerr << std::fixed << std::setprecision(17) << "descriptors1.at<double>(i,j)=" << descriptors1.at<double>(i,j)
              << " ; descriptors2.at<double>(i,j)=" << descriptors2.at<double>(i,j) << std::endl;
          return false;
        }
        break;

      default:
        return false;
        break;
      }
    }
  }

  return true;
}

int main(int argc, const char ** argv) {
  try {
    std::string env_ipath;
    std::string opt_opath;
    std::string username;
    std::string opath;
    std::string filename;
    std::string bin_file = "", xml_file = "";

    //Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    if(env_ipath.empty()) {
      throw vpException(vpException::ioError, "Please set the VISP_INPUT_IMAGE_PATH environment variable value.");
    }

    // Set the default output path
#if defined(_WIN32)
    opt_opath = "C:/temp";
#else
    opt_opath = "/tmp";
#endif

    // Get the user login name
    vpIoTools::getUserName(username);

    // Read the command line options
    if (getOptions(argc, argv, opt_opath, username, bin_file, xml_file) == false) {
      throw vpException(vpException::fatalError, "getOptions(argc, argv, opt_opath, username) == false");
    }

    // Get the option values
    if (!opt_opath.empty()) {
      opath = opt_opath;
    }

    // Append to the output path string, the login name of the user
    opath = vpIoTools::createFilePath(opath, username);

    // Test if the output path exist. If no try to create it
    if (vpIoTools::checkDirectory(opath) == false) {
      try {
        // Create the dirname
        vpIoTools::makeDirectory(opath);
      }
      catch (...) {
        usage(argv[0], NULL, opt_opath, username);
        std::stringstream ss;
        ss << std::endl << "ERROR:" << std::endl;
        ss << "  Cannot create " << opath << std::endl;
        ss << "  Check your -o " << opt_opath << " option " << std::endl;
        throw vpException(vpException::ioError, ss.str().c_str());
      }
    }

    vpImage<unsigned char> I;

    //Set the path location of the image sequence
    std::string dirname = vpIoTools::createFilePath(env_ipath, "ViSP-images/Klimt");

    //Build the name of the image files
    std::string img_filename = vpIoTools::createFilePath(dirname, "/Klimt.ppm");
    vpImageIo::read(I, img_filename);


#if defined(VISP_HAVE_XML2)
    if(!bin_file.empty() && !xml_file.empty()) {
      //Check if read(binary_file) == read(xml_file)
      vpKeyPoint keyPoints_binary, keyPoints_xml;

      keyPoints_binary.loadLearningData(bin_file, true);
      keyPoints_xml.loadLearningData(xml_file, false);

      std::vector<cv::KeyPoint> trainKeyPoints_read_binary, trainKeyPoints_read_xml;
      keyPoints_binary.getTrainKeyPoints(trainKeyPoints_read_binary);
      keyPoints_xml.getTrainKeyPoints(trainKeyPoints_read_xml);

      if(!compareKeyPoints(trainKeyPoints_read_binary, trainKeyPoints_read_xml)) {
        throw vpException(vpException::fatalError, "Problem with trainKeyPoints when reading learning file saved "
            "in binary with train images saved !");
      }

      cv::Mat trainDescriptors_read_binary = keyPoints_binary.getTrainDescriptors();
      cv::Mat trainDescriptors_read_xml = keyPoints_xml.getTrainDescriptors();

      if(!compareDescriptors(trainDescriptors_read_binary, trainDescriptors_read_xml)) {
        throw vpException(vpException::fatalError, "Problem with trainDescriptors when reading learning file saved in "
            "binary with train images saved !");
      }

      std::cout << "Read(xml_file) == Read(binary_file) / OK !" << std::endl;
    }

    if(!xml_file.empty()) {
      //Ckeck if all keypoints are inside the image
      vpKeyPoint keyPoints_xml;
      std::vector<cv::KeyPoint> trainKeyPoints_read_xml;
      keyPoints_xml.getTrainKeyPoints(trainKeyPoints_read_xml);

      for(std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints_read_xml.begin();
          it != trainKeyPoints_read_xml.end(); ++it) {
        if(it->pt.x < 0 || it->pt.y < 0 || it->pt.x >= I.getWidth() || it->pt.y >= I.getHeight()) {
          std::cerr << "x=" << it->pt.x << " ; y=" << it->pt.y << std::endl;
          throw vpException(vpException::fatalError, "Problem with trainKeyPoints_read_xml ! "
              "The keypoint is outside the image !");
        }
      }

      std::cout << "Read(xml_file) is OK !" << std::endl;
    }
#endif

    if(!bin_file.empty()) {
      //Ckeck if all keypoints are inside the image
      vpKeyPoint keyPoints_binary;
      std::vector<cv::KeyPoint> trainKeyPoints_read_binary;
      keyPoints_binary.getTrainKeyPoints(trainKeyPoints_read_binary);

      for(std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints_read_binary.begin();
          it != trainKeyPoints_read_binary.end(); ++it) {
        if(it->pt.x < 0 || it->pt.y < 0 || it->pt.x >= I.getWidth() || it->pt.y >= I.getHeight()) {
          std::cerr << "x=" << it->pt.x << " ; y=" << it->pt.y << std::endl;
          throw vpException(vpException::fatalError, "Problem with trainKeyPoints_read_binary ! "
              "The keypoint is outside the image !");
        }
      }

      std::cout << "Read(binary_file) is OK !" << std::endl;
    }

  } catch(vpException &e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  return 0;
}
#else
int main() {
  std::cerr << "You need OpenCV library." << std::endl;

  return 0;
}

#endif
