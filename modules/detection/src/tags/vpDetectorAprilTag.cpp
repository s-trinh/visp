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
 * Base class for April Tag detection.
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_APRILTAG

#include <apriltag.h>
#include <tag36h11.h>
#include <tag36h10.h>
#include <tag36artoolkit.h>
#include <tag25h9.h>
#include <tag25h7.h>
#include <common/homography.h>

#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpPixelMeterConversion.h>

/*!
   Default constructor.
 */
vpDetectorAprilTag::vpDetectorAprilTag(const vpAprilTagFamily &tagFamily) : /*m_tf(NULL), m_td(NULL),*/ m_tagFamily(tagFamily) {
//  switch (m_tagFamily) {
//    case TAG_36h11:
//      m_tf = tag36h11_create();
//      break;

//    case TAG_36h10:
//      m_tf = tag36h10_create();
//      break;

//    case TAG_36ARTOOLKIT:
//      m_tf = tag36artoolkit_create();
//      break;

//    case TAG_25h9:
//      m_tf = tag25h9_create();
//      break;

//    case TAG_25h7:
//      m_tf = tag25h7_create();
//      break;

//    default:
//      throw vpException(vpException::fatalError, "Unknow April Tag family!");
//      break;
//  }

//  m_td = apriltag_detector_create();
//  apriltag_detector_add_family(m_td, m_tf);
}

vpDetectorAprilTag::~vpDetectorAprilTag() {
//  apriltag_detector_destroy(m_td);

//  switch (m_tagFamily) {
//    case TAG_36h11:
//      tag36h11_destroy(m_tf);
//      break;

//    case TAG_36h10:
//      tag36h10_destroy(m_tf);
//      break;

//    case TAG_36ARTOOLKIT:
//      tag36artoolkit_destroy(m_tf);
//      break;

//    case TAG_25h9:
//      tag25h9_destroy(m_tf);
//      break;

//    case TAG_25h7:
//      tag25h7_destroy(m_tf);
//      break;

//    default:
//      break;
//  }
}

/*!
  Detect AprilTag tags in the image. Return true if a tag is detected, false otherwise.

  \param I : Input image.
 */
bool vpDetectorAprilTag::detect(const vpImage<unsigned char> &I) {
  apriltag_family_t *tf = NULL;
  switch (m_tagFamily) {
    case TAG_36h11:
      tf = tag36h11_create();
      break;

    case TAG_36h10:
      tf = tag36h10_create();
      break;

    case TAG_36ARTOOLKIT:
      tf = tag36artoolkit_create();
      break;

    case TAG_25h9:
      tf = tag25h9_create();
      break;

    case TAG_25h7:
      tf = tag25h7_create();
      break;

    default:
      throw vpException(vpException::fatalError, "Unknow April Tag family!");
      break;
  }

  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);


  // Make an image_u8_t header for the Mat data
  image_u8_t im = { .width = I.getWidth(),
      .height = (int32_t) I.getHeight(),
      .stride = (int32_t) I.getWidth(),
      .buf = I.bitmap
  };

  zarray_t *detections = apriltag_detector_detect(td, &im);
  bool detected = zarray_size(detections) > 0;
  std::cout << zarray_size(detections) << " tags detected" << std::endl;

  // Draw detection outlines
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);

    vpDisplay::displayLine(I, det->p[0][1], det->p[0][0], det->p[1][1], det->p[1][0], vpColor::green, 2);
    vpDisplay::displayLine(I, det->p[0][1], det->p[0][0], det->p[3][1], det->p[3][0], vpColor::green, 2);
    vpDisplay::displayLine(I, det->p[1][1], det->p[1][0], det->p[2][1], det->p[2][0], vpColor::green, 2);
    vpDisplay::displayLine(I, det->p[2][1], det->p[2][0], det->p[3][1], det->p[3][0], vpColor::green, 2);

    std::stringstream ss;
    ss << det->id;
    vpDisplay::displayText(I, det->c[1], det->c[0], ss.str(), vpColor::red);
  }


  zarray_destroy(detections);
  apriltag_detector_destroy(td);

  switch (m_tagFamily) {
    case TAG_36h11:
      tag36h11_destroy(tf);
      break;

    case TAG_36h10:
      tag36h10_destroy(tf);
      break;

    case TAG_36ARTOOLKIT:
      tag36artoolkit_destroy(tf);
      break;

    case TAG_25h9:
      tag25h9_destroy(tf);
      break;

    case TAG_25h7:
      tag25h7_destroy(tf);
      break;

    default:
      break;
  }

  return detected;
}

void vpDetectorAprilTag::detect(const vpImage<unsigned char> &I, const double tagSize, const vpCameraParameters &cam, std::vector<vpHomogeneousMatrix> &cMo_vec) {
  apriltag_family_t *tf = NULL;
  switch (m_tagFamily) {
    case TAG_36h11:
      tf = tag36h11_create();
      break;

    case TAG_36h10:
      tf = tag36h10_create();
      break;

    case TAG_36ARTOOLKIT:
      tf = tag36artoolkit_create();
      break;

    case TAG_25h9:
      tf = tag25h9_create();
      break;

    case TAG_25h7:
      tf = tag25h7_create();
      break;

    default:
      throw vpException(vpException::fatalError, "Unknow April Tag family!");
      break;
  }

  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);


  // Make an image_u8_t header for the Mat data
  image_u8_t im = { .width = I.getWidth(),
      .height = (int32_t) I.getHeight(),
      .stride = (int32_t) I.getWidth(),
      .buf = I.bitmap
  };

  double t = vpTime::measureTimeMs();
  zarray_t *detections = apriltag_detector_detect(td, &im);
  t = vpTime::measureTimeMs() - t;

  std::cout << zarray_size(detections) << " tags detected in " << t << " ms" << std::endl;

  double fx = cam.get_px(), fy = cam.get_py();
  double cx = cam.get_u0(), cy = cam.get_v0();

  // Draw detection outlines
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);

    vpDisplay::displayLine(I, det->p[0][1], det->p[0][0], det->p[1][1], det->p[1][0], vpColor::red, 2);
    vpDisplay::displayLine(I, det->p[0][1], det->p[0][0], det->p[3][1], det->p[3][0], vpColor::green, 2);
    vpDisplay::displayLine(I, det->p[1][1], det->p[1][0], det->p[2][1], det->p[2][0], vpColor::green, 2);
    vpDisplay::displayLine(I, det->p[2][1], det->p[2][0], det->p[3][1], det->p[3][0], vpColor::green, 2);

    std::stringstream ss;
    ss << det->id;
    vpDisplay::displayText(I, det->c[1], det->c[0], ss.str(), vpColor::red);

    matd_t *M = homography_to_pose(det->H, fx, fy, cx, cy, tagSize, 0);

    vpHomogeneousMatrix cMo;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        cMo[i][j] = MATD_EL(M, i, j);
      }
      cMo[i][3] = MATD_EL(M, i, 3);
    }
    cMo_vec.push_back(cMo);

    matd_destroy(M);
  }


  zarray_destroy(detections);
  apriltag_detector_destroy(td);

  switch (m_tagFamily) {
    case TAG_36h11:
      tag36h11_destroy(tf);
      break;

    case TAG_36h10:
      tag36h10_destroy(tf);
      break;

    case TAG_36ARTOOLKIT:
      tag36artoolkit_destroy(tf);
      break;

    case TAG_25h9:
      tag25h9_destroy(tf);
      break;

    case TAG_25h7:
      tag25h7_destroy(tf);
      break;

    default:
      break;
  }
}

void vpDetectorAprilTag::detect(const vpImage<unsigned char> &I, const double tagSize, const vpCameraParameters &cam, std::vector<std::vector<vpPoint> > &pts_vec) {
  apriltag_family_t *tf = NULL;
  switch (m_tagFamily) {
    case TAG_36h11:
      tf = tag36h11_create();
      break;

    case TAG_36h10:
      tf = tag36h10_create();
      break;

    case TAG_36ARTOOLKIT:
      tf = tag36artoolkit_create();
      break;

    case TAG_25h9:
      tf = tag25h9_create();
      break;

    case TAG_25h7:
      tf = tag25h7_create();
      break;

    default:
      throw vpException(vpException::fatalError, "Unknow April Tag family!");
      break;
  }

  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);


  // Make an image_u8_t header for the Mat data
  image_u8_t im = { .width = I.getWidth(),
      .height = (int32_t) I.getHeight(),
      .stride = (int32_t) I.getWidth(),
      .buf = I.bitmap
  };

  double t = vpTime::measureTimeMs();
  zarray_t *detections = apriltag_detector_detect(td, &im);
  t = vpTime::measureTimeMs() - t;

  std::stringstream ss;
  ss << zarray_size(detections) << " tags detected in " << t << " ms";
  vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);

  // Draw detection outlines
  pts_vec.reserve( (size_t) zarray_size(detections) );
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);

    vpDisplay::displayLine(I, det->p[0][1], det->p[0][0], det->p[1][1], det->p[1][0], vpColor::red, 2);
    vpDisplay::displayLine(I, det->p[0][1], det->p[0][0], det->p[3][1], det->p[3][0], vpColor::green, 2);
    vpDisplay::displayLine(I, det->p[1][1], det->p[1][0], det->p[2][1], det->p[2][0], vpColor::green, 2);
    vpDisplay::displayLine(I, det->p[2][1], det->p[2][0], det->p[3][1], det->p[3][0], vpColor::green, 2);

    std::stringstream ss2;
    ss2 << det->id;
    vpDisplay::displayText(I, det->c[1], det->c[0], ss2.str(), vpColor::red);

    vpPoint pt;
    vpImagePoint imPt;
    double x = 0.0, y = 0.0;
    std::vector<vpPoint> pts(4);
    pt.setWorldCoordinates(-tagSize/2.0, -tagSize/2.0, 0.0);
    imPt.set_uv(det->p[0][0], det->p[0][1]);
    vpPixelMeterConversion::convertPoint(cam, imPt, x, y);
    pt.set_x(x);
    pt.set_y(y);
    pts[0] = pt;

    pt.setWorldCoordinates(tagSize/2.0, -tagSize/2.0, 0.0);
    imPt.set_uv(det->p[1][0], det->p[1][1]);
    vpPixelMeterConversion::convertPoint(cam, imPt, x, y);
    pt.set_x(x);
    pt.set_y(y);
    pts[1] = pt;

    pt.setWorldCoordinates(tagSize/2.0, tagSize/2.0, 0.0);
    imPt.set_uv(det->p[2][0], det->p[2][1]);
    vpPixelMeterConversion::convertPoint(cam, imPt, x, y);
    pt.set_x(x);
    pt.set_y(y);
    pts[2] = pt;

    pt.setWorldCoordinates(-tagSize/2.0, tagSize/2.0, 0.0);
    imPt.set_uv(det->p[3][0], det->p[3][1]);
    vpPixelMeterConversion::convertPoint(cam, imPt, x, y);
    pt.set_x(x);
    pt.set_y(y);
    pts[3] = pt;

    pts_vec.push_back(pts);
  }


  zarray_destroy(detections);
  apriltag_detector_destroy(td);

  switch (m_tagFamily) {
    case TAG_36h11:
      tag36h11_destroy(tf);
      break;

    case TAG_36h10:
      tag36h10_destroy(tf);
      break;

    case TAG_36ARTOOLKIT:
      tag36artoolkit_destroy(tf);
      break;

    case TAG_25h9:
      tag25h9_destroy(tf);
      break;

    case TAG_25h7:
      tag25h7_destroy(tf);
      break;

    default:
      break;
  }
}
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpDetectorAprilTag.cpp.o) has no symbols
void dummy_vpDetectorAprilTag() {};
#endif
