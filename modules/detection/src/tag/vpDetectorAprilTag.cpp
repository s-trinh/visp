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
#include <visp3/vision/vpPose.h>

class vpDetectorAprilTag::Impl {
public:
  Impl(const vpAprilTagFamily &tagFamily) : m_cam(), m_poseFromHomography(false), m_tagFamily(tagFamily), m_tagPoses(), m_tagSize(1.0), m_td(NULL), m_tf(NULL) {
    switch (m_tagFamily) {
      case TAG_36h11:
        m_tf = tag36h11_create();
        break;

      case TAG_36h10:
        m_tf = tag36h10_create();
        break;

      case TAG_36ARTOOLKIT:
        m_tf = tag36artoolkit_create();
        break;

      case TAG_25h9:
        m_tf = tag25h9_create();
        break;

      case TAG_25h7:
        m_tf = tag25h7_create();
        break;

      default:
        throw vpException(vpException::fatalError, "Unknow Tag family!");
        break;
    }

    m_td = apriltag_detector_create();
    apriltag_detector_add_family(m_td, m_tf);
  }

  ~Impl() {
    apriltag_detector_destroy(m_td);

    switch (m_tagFamily) {
      case TAG_36h11:
        tag36h11_destroy(m_tf);
        break;

      case TAG_36h10:
        tag36h10_destroy(m_tf);
        break;

      case TAG_36ARTOOLKIT:
        tag36artoolkit_destroy(m_tf);
        break;

      case TAG_25h9:
        tag25h9_destroy(m_tf);
        break;

      case TAG_25h7:
        tag25h7_destroy(m_tf);
        break;

      default:
        break;
    }
  }

  bool detect(const vpImage<unsigned char> &I, std::vector<std::vector<vpImagePoint> > &polygons, std::vector<std::string> &messages, const bool computePose) {
    m_tagPoses.clear();

    image_u8_t im = { .width = (int32_t) I.getWidth(),
                      .height = (int32_t) I.getHeight(),
                      .stride = (int32_t) I.getWidth(),
                      .buf = I.bitmap
                    };

    zarray_t *detections = apriltag_detector_detect(m_td, &im);
    int nb_detections = zarray_size(detections);
    bool detected = nb_detections > 0;

    polygons.resize( (size_t) nb_detections);
    messages.resize( (size_t) nb_detections);

    std::string tag_family_name = "";
    switch (m_tagFamily) {
      case TAG_36h11:
        tag_family_name = "36h11";
        break;

      case TAG_36h10:
        tag_family_name = "36h10";
        break;

      case TAG_36ARTOOLKIT:
        tag_family_name = "36artoolkit";
        break;

      case TAG_25h9:
        tag_family_name = "25h9";
        break;

      case TAG_25h7:
        tag_family_name = "25h7";
        break;

      default:
        break;
    }

    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);

      std::vector<vpImagePoint> polygon;
      for (int j = 0; j < 4; j++) {
        polygon.push_back( vpImagePoint(det->p[j][1], det->p[j][0]) );
      }
      polygons[i] = polygon;
      std::stringstream ss;
      ss << tag_family_name << " id: " << det->id;
      messages[i] = ss.str();

      if (computePose) {
        if (m_poseFromHomography) {
          double fx = m_cam.get_px(), fy = m_cam.get_py();
          double cx = m_cam.get_u0(), cy = m_cam.get_v0();

          matd_t *M = homography_to_pose(det->H, fx, fy, cx, cy, m_tagSize, 0);

          vpHomogeneousMatrix cMo;
          for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
              cMo[i][j] = MATD_EL(M, i, j);
            }
            cMo[i][3] = MATD_EL(M, i, 3);
          }
          m_tagPoses.push_back(cMo);

          matd_destroy(M);
        } else {
          vpPoint pt;
          vpImagePoint imPt;
          double x = 0.0, y = 0.0;
          std::vector<vpPoint> pts(4);
          pt.setWorldCoordinates(-m_tagSize/2.0, -m_tagSize/2.0, 0.0);
          imPt.set_uv(det->p[0][0], det->p[0][1]);
          vpPixelMeterConversion::convertPoint(m_cam, imPt, x, y);
          pt.set_x(x);
          pt.set_y(y);
          pts[0] = pt;

          pt.setWorldCoordinates(m_tagSize/2.0, -m_tagSize/2.0, 0.0);
          imPt.set_uv(det->p[1][0], det->p[1][1]);
          vpPixelMeterConversion::convertPoint(m_cam, imPt, x, y);
          pt.set_x(x);
          pt.set_y(y);
          pts[1] = pt;

          pt.setWorldCoordinates(m_tagSize/2.0, m_tagSize/2.0, 0.0);
          imPt.set_uv(det->p[2][0], det->p[2][1]);
          vpPixelMeterConversion::convertPoint(m_cam, imPt, x, y);
          pt.set_x(x);
          pt.set_y(y);
          pts[2] = pt;

          pt.setWorldCoordinates(-m_tagSize/2.0, m_tagSize/2.0, 0.0);
          imPt.set_uv(det->p[3][0], det->p[3][1]);
          vpPixelMeterConversion::convertPoint(m_cam, imPt, x, y);
          pt.set_x(x);
          pt.set_y(y);
          pts[3] = pt;

          vpPose pose;
          pose.addPoints(pts);
          vpHomogeneousMatrix cMo_dementhon, cMo_lagrange;
          double residual_dementhon = std::numeric_limits<double>::max(), residual_lagrange = std::numeric_limits<double>::max();

          if (pose.computePose(vpPose::DEMENTHON, cMo_dementhon)) {
            residual_dementhon = pose.computeResidual(cMo_dementhon);
          }

          if (pose.computePose(vpPose::LAGRANGE, cMo_lagrange)) {
            residual_lagrange = pose.computeResidual(cMo_lagrange);
          }

          vpHomogeneousMatrix cMo = residual_dementhon < residual_lagrange ? cMo_dementhon : cMo_lagrange;
          pose.computePose(vpPose::VIRTUAL_VS, cMo);

          m_tagPoses.push_back(cMo);
        }
      }
    }

    zarray_destroy(detections);

    return detected;
  }

  void getTagPoses(std::vector<vpHomogeneousMatrix> &tagPoses) const {
    tagPoses = m_tagPoses;
  }

  void setCameraParameters(const vpCameraParameters &cam) {
    m_cam = cam;
  }

  void setTagSize(const double tagSize) {
    m_tagSize = tagSize;
  }

  void setUsePoseFromHomography(const bool use) {
    m_poseFromHomography = use;
  }

protected:
  vpCameraParameters m_cam;
  bool m_poseFromHomography;
  vpAprilTagFamily m_tagFamily;
  std::vector<vpHomogeneousMatrix> m_tagPoses;
  double m_tagSize;
  apriltag_detector_t *m_td;
  apriltag_family_t *m_tf;
};

/*!
   Default constructor.
*/
vpDetectorAprilTag::vpDetectorAprilTag(const vpAprilTagFamily &tagFamily) : m_poseFromHomography(false), m_tagFamily(tagFamily), m_impl(new Impl(tagFamily)) {
}

vpDetectorAprilTag::~vpDetectorAprilTag() {
  delete m_impl;
}

/*!
  Detect AprilTag tags in the image. Return true if at least one tag is detected, false otherwise.

  \param I : Input image.
*/
bool vpDetectorAprilTag::detect(const vpImage<unsigned char> &I) {
  m_message.clear();
  m_polygon.clear();
  m_nb_objects = 0;

  bool detected =  m_impl->detect(I, m_polygon, m_message, false);
  m_nb_objects = m_message.size();

  return detected;
}

void vpDetectorAprilTag::detect(const vpImage<unsigned char> &I, const double tagSize, const vpCameraParameters &cam, std::vector<vpHomogeneousMatrix> &cMo_vec) {
  m_message.clear();
  m_polygon.clear();
  m_nb_objects = 0;

  m_impl->setTagSize(tagSize);
  m_impl->setCameraParameters(cam);
  m_impl->detect(I, m_polygon, m_message, true);
  m_nb_objects = m_message.size();
  m_impl->getTagPoses(cMo_vec);
}
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpDetectorAprilTag.cpp.o) has no symbols
void dummy_vpDetectorAprilTag() {}
#endif
