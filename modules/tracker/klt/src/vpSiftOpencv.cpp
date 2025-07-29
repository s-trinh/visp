/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
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
 * Wrapper for the KLT (Kanade-Lucas-Tomasi) feature tracker implemented
 * with opencv.
 */

/*!
  \file vpSiftOpencv.cpp

  \brief Wrapper for the KLT (Kanade-Lucas-Tomasi) feature tracker
  implemented with opencv.
*/

#include <visp3/core/vpConfig.h>

#if defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)

#include <string>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/klt/vpSiftOpencv.h>

BEGIN_VISP_NAMESPACE
vpSiftOpencv::vpSiftOpencv()
  : m_gray(), m_points_id(),
  m_next_points_id(0),
  m_siftDetector(),
  m_keyPointsRef(), m_keyPointsCur(),
  m_descriptorsRef(), m_descriptorsCur(),
  m_descriptorsMatcher(), m_knnMatches(), m_matches01(), m_matches10()
{
  m_siftDetector = cv::SiftFeatureDetector::create();
  const bool cross_check = true;
  m_descriptorsMatcher = cv::BFMatcher::create(cv::NORM_L2, cross_check);
}

vpSiftOpencv::vpSiftOpencv(const vpSiftOpencv &copy)
  : m_gray(), m_points_id(),
  m_next_points_id(0),
  m_siftDetector(),
  m_keyPointsRef(), m_keyPointsCur(),
  m_descriptorsRef(), m_descriptorsCur(),
  m_descriptorsMatcher(), m_knnMatches(), m_matches01(), m_matches10()
{
  *this = copy;
}

vpSiftOpencv &vpSiftOpencv::operator=(const vpSiftOpencv &copy)
{
  m_gray = copy.m_gray;
  m_points[0] = copy.m_points[0];
  m_points[1] = copy.m_points[1];
  m_points_id = copy.m_points_id;
  m_next_points_id = copy.m_next_points_id;

  m_siftDetector = copy.m_siftDetector;

  m_keyPointsRef = copy.m_keyPointsRef;
  m_keyPointsCur = copy.m_keyPointsCur;

  m_descriptorsRef = copy.m_descriptorsRef;
  m_descriptorsCur = copy.m_descriptorsCur;

  m_descriptorsMatcher = copy.m_descriptorsMatcher;
  m_knnMatches = copy.m_knnMatches;
  m_matches01 = copy.m_matches01;
  m_matches10 = copy.m_matches10;

  return *this;
}

vpSiftOpencv::~vpSiftOpencv() { }

void vpSiftOpencv::initTracking(const cv::Mat &I, const cv::Mat &mask)
{
  m_next_points_id = 0;

  I.copyTo(m_gray);

  for (size_t i = 0; i < 2; i++) {
    m_points[i].clear();
  }

  m_points_id.clear();

  m_keyPointsRef.clear();
  m_siftDetector->detectAndCompute(m_gray, mask, m_keyPointsRef, m_descriptorsRef);

  for (size_t i = 0; i < m_keyPointsRef.size(); i++) {
    m_points[0].push_back(m_keyPointsRef[i].pt);
  }
  for (size_t i = 0; i < m_points[0].size(); i++) {
    m_points_id.push_back(m_next_points_id++);
  }

  m_points[1] = m_points[0];
  std::cout << "m_points[0]=" << m_points[0].size() << " ; m_points_id=" << m_points_id.size() << std::endl;


  // cv::goodFeaturesToTrack(m_gray, m_points[1], m_maxCount, m_qualityLevel, m_minDistance, mask, m_blockSize, false,
  //                         m_harris_k);

  // if (m_points[1].size() > 0) {
  //   cv::cornerSubPix(m_gray, m_points[1], cv::Size(m_winSize, m_winSize), cv::Size(-1, -1), m_termcrit);

  //   for (size_t i = 0; i < m_points[1].size(); i++)
  //     m_points_id.push_back(m_next_points_id++);
  // }
}

void vpSiftOpencv::track(const cv::Mat &I)
{
  if (m_points[1].size() == 0) {
    throw vpTrackingException(vpTrackingException::fatalError, "Not enough key points to track.");
  }

  if (!m_keyPointsCur.empty()) {
    cv::swap(m_descriptorsRef, m_descriptorsCur);
    cv::swap(m_keyPointsRef, m_keyPointsCur);
  }

  m_keyPointsCur.clear();
  m_siftDetector->detectAndCompute(I, cv::noArray(), m_keyPointsCur, m_descriptorsCur);

  m_matches01.clear();
  m_matches10.clear();
  std::cout << "m_keyPointsCur=" << m_keyPointsCur.size() << " ; m_keyPointsRef=" << m_keyPointsRef.size() << std::endl;
  std::cout << "m_descriptorsCur=" << m_descriptorsCur.rows << "x" << m_descriptorsCur.cols << std::endl;
  std::cout << "m_descriptorsRef=" << m_descriptorsRef.rows << "x" << m_descriptorsRef.cols << std::endl;
  // m_descriptorsMatcher->match(m_descriptorsRef, m_descriptorsCur, m_matches01);
  m_descriptorsMatcher->match(m_descriptorsCur, m_descriptorsRef, m_matches10);
  std::cout << "m_matches01=" << m_matches01.size() << " ; m_matches10=" << m_matches10.size() << std::endl;

  m_points[0].clear();
  m_points[1].clear();
  // m_points_id.clear();
  std::vector<long> points_id;
  if (!m_points_id.empty()) {
    points_id = m_points_id;
  }
  m_points_id = std::vector<long>(m_keyPointsCur.size(), -1);

  std::vector<uchar> status0(m_keyPointsRef.size(), 0);
  std::vector<uchar> status1(m_keyPointsCur.size(), 0);
  int nb_correct0 = 0;
  int nb_correct1 = 0;

  for (size_t i = 0; i < m_matches10.size(); i++) {
    const cv::DMatch &match = m_matches10[i];
    // m_points[0].push_back(m_keyPointsRef[match.queryIdx].pt);
    // m_points[1].push_back(m_keyPointsCur[match.trainIdx].pt);
    m_points[0].push_back(m_keyPointsRef[match.trainIdx].pt);
    m_points[1].push_back(m_keyPointsCur[match.queryIdx].pt);

    // m_points_id.push_back(match.queryIdx);
    // m_points_id.push_back(match.trainIdx);
    if (!points_id.empty()) {
      m_points_id[match.queryIdx] = points_id[match.trainIdx];
    }
    else {
      m_points_id[match.queryIdx] = match.trainIdx;
    }
    // m_points_id.push_back(i);
    status0[match.trainIdx] = 1;
    status1[match.queryIdx] = 1;
    nb_correct0++;
    nb_correct1++;
  }

  // // Remove points that are lost
  // for (int i = static_cast<int>(status0.size()) - 1; i >= 0; i--) {
  //   if (status0[static_cast<size_t>(i)] == 0) {
  //     m_keyPointsRef.erase(m_keyPointsRef.begin() + i);
  //   }
  // }
  for (int i = static_cast<int>(status1.size()) - 1; i >= 0; i--) {
    if (status1[static_cast<size_t>(i)] == 0) {
      m_keyPointsCur.erase(m_keyPointsCur.begin() + i);
      m_points_id.erase(m_points_id.begin() + i);
    }
  }
  std::cout << "After filter, m_keyPointsCur=" << m_keyPointsCur.size() << std::endl;

  // cv::Mat descriptorsRef(nb_correct0, m_descriptorsRef.cols, m_descriptorsRef.type());
  // for (int i = 0, idx = 0; i < m_descriptorsRef.rows; i++) {
  //   if (status0[static_cast<size_t>(i)] == 1) {
  //     descriptorsRef(cv::Range(idx, idx+1), cv::Range::all()) = m_descriptorsRef(cv::Range(i, i+1), cv::Range::all()).clone();
  //     idx++;
  //   }
  // }
  // m_descriptorsRef = descriptorsRef.clone();

  cv::Mat descriptorsCur(nb_correct1, m_descriptorsCur.cols, m_descriptorsCur.type());
  std::cout << "Before filter, nb_correct1=" << nb_correct1 << " ; descriptorsCur=" << descriptorsCur.rows << "x" << descriptorsCur.cols << std::endl;
  for (int i = 0, idx = 0; i < m_descriptorsCur.rows; i++) {
    if (status1[static_cast<size_t>(i)] == 1) {
      // descriptorsCur(cv::Range(idx, idx+1), cv::Range::all()) = m_descriptorsCur(cv::Range(i, i+1), cv::Range::all()).clone();
      // descriptorsCur.rowRange(idx, idx+1).colRange(cv::Range::all()) = m_descriptorsCur(cv::Range(i, i+1), cv::Range::all()).clone();
      // descriptorsCur.rowRange(idx, idx+1).colRange(cv::Range::all()) = m_descriptorsCur.rowRange(i, i+1).colRange(cv::Range::all()).clone();
      // cv::Mat tmp_cur = m_descriptorsCur(cv::Range(i, i+1), cv::Range::all()).clone();
      cv::Mat dest = descriptorsCur(cv::Range(idx, idx+1), cv::Range::all());
      // tmp_cur.copyTo(dest);
      m_descriptorsCur(cv::Range(i, i+1), cv::Range::all()).copyTo(dest);

      // for (int col0 = 0; col0 < 128; col0++) {
        // descriptorsCur.at<float>(idx, col0) = m_descriptorsCur.at<float>(i, col0);
      // }
      idx++;
    }
  }
  std::cout << "After filter, nb_correct1=" << nb_correct1 << " ; descriptorsCur=" << descriptorsCur.rows << "x" << descriptorsCur.cols << std::endl;
  std::cout << "m_descriptorsCur[0,0]=" << m_descriptorsCur.at<float>(0, 0) << " ; m_descriptorsCur[1,0]=" << m_descriptorsCur.at<float>(1, 0) << std::endl;
  std::cout << "descriptorsCur[0,0]=" << descriptorsCur.at<float>(0, 0) << " ; descriptorsCur[1,0]=" << descriptorsCur.at<float>(1, 0) << std::endl;
  std::cout << "status1[0]=" << int(status1[0]) << " ; status1[1]=" << int(status1[1]) << std::endl;
  m_descriptorsCur = descriptorsCur.clone();
  std::cout << "After filter, m_descriptorsCur=" << m_descriptorsCur.rows << "x" << m_descriptorsCur.cols << std::endl;





  // std::vector<float> err;
  // int flags = 0;

  // cv::swap(m_prevGray, m_gray);

  // if (m_initial_guess) {
  //   flags |= cv::OPTFLOW_USE_INITIAL_FLOW;
  //   m_initial_guess = false;
  // }
  // else {
  //   std::swap(m_points[1], m_points[0]);
  // }

  // // cvtColor(I, m_gray, cv::COLOR_BGR2GRAY);
  // I.copyTo(m_gray);

  // if (m_prevGray.empty()) {
  //   m_gray.copyTo(m_prevGray);
  // }

  // std::vector<uchar> status;

  // cv::calcOpticalFlowPyrLK(m_prevGray, m_gray, m_points[0], m_points[1], status, err, cv::Size(m_winSize, m_winSize),
  //                          m_pyrMaxLevel, m_termcrit, flags, m_minEigThreshold);

  // // Remove points that are lost
  // for (int i = static_cast<int>(status.size()) - 1; i >= 0; i--) {
  //   if (status[static_cast<size_t>(i)] == 0) { // point is lost
  //     m_points[0].erase(m_points[0].begin() + i);
  //     m_points[1].erase(m_points[1].begin() + i);
  //     m_points_id.erase(m_points_id.begin() + i);
  //   }
  // }
}

void vpSiftOpencv::getFeature(const int &index, long &id, float &x, float &y) const
{
  if (static_cast<size_t>(index) >= m_points[1].size()) {
    throw(vpException(vpException::badValue, "Feature [%d] doesn't exist", index));
  }

  x = m_points[1][static_cast<size_t>(index)].x;
  y = m_points[1][static_cast<size_t>(index)].y;
  id = m_points_id[static_cast<size_t>(index)];
}

void vpSiftOpencv::display(const vpImage<unsigned char> &I, const vpColor &color, unsigned int thickness) const
{
  vpSiftOpencv::display(I, m_points[1], m_points_id, color, thickness);
}

void vpSiftOpencv::display(const vpImage<unsigned char> &I, const std::vector<cv::Point2f> &features,
                          const vpColor &color, unsigned int thickness)
{
  vpImagePoint ip;
  for (size_t i = 0; i < features.size(); i++) {
    ip.set_u(vpMath::round(features[i].x));
    ip.set_v(vpMath::round(features[i].y));
    vpDisplay::displayCross(I, ip, 10 + thickness, color, thickness);
  }
}

void vpSiftOpencv::display(const vpImage<vpRGBa> &I, const std::vector<cv::Point2f> &features, const vpColor &color,
                          unsigned int thickness)
{
  vpImagePoint ip;
  for (size_t i = 0; i < features.size(); i++) {
    ip.set_u(vpMath::round(features[i].x));
    ip.set_v(vpMath::round(features[i].y));
    vpDisplay::displayCross(I, ip, 10 + thickness, color, thickness);
  }
}

void vpSiftOpencv::display(const vpImage<unsigned char> &I, const std::vector<cv::Point2f> &features,
                          const std::vector<long> &featuresid, const vpColor &color, unsigned int thickness)
{
  vpImagePoint ip;
  for (size_t i = 0; i < features.size(); i++) {
    ip.set_u(vpMath::round(features[i].x));
    ip.set_v(vpMath::round(features[i].y));
    vpDisplay::displayCross(I, ip, 10, color, thickness);

    std::ostringstream id;
    id << featuresid[i];
    ip.set_u(vpMath::round(features[i].x + 5));
    vpDisplay::displayText(I, ip, id.str(), color);
  }
}

void vpSiftOpencv::display(const vpImage<vpRGBa> &I, const std::vector<cv::Point2f> &features,
                          const std::vector<long> &featuresid, const vpColor &color, unsigned int thickness)
{
  vpImagePoint ip;
  for (size_t i = 0; i < features.size(); i++) {
    ip.set_u(vpMath::round(features[i].x));
    ip.set_v(vpMath::round(features[i].y));
    vpDisplay::displayCross(I, ip, 10, color, thickness);

    std::ostringstream id;
    id << featuresid[i];
    ip.set_u(vpMath::round(features[i].x + 5));
    vpDisplay::displayText(I, ip, id.str(), color);
  }
}

// void vpSiftOpencv::initTracking(const cv::Mat &I, const std::vector<cv::Point2f> &pts)
// {
//   m_points[1] = pts;
//   m_next_points_id = 0;
//   m_points_id.clear();
//   for (size_t i = 0; i < m_points[1].size(); i++) {
//     m_points_id.push_back(m_next_points_id++);
//   }

//   I.copyTo(m_gray);
// }

// void vpSiftOpencv::initTracking(const cv::Mat &I, const std::vector<cv::Point2f> &pts, const std::vector<long> &ids)
// {
//   m_points[1] = pts;
//   m_points_id.clear();

//   if (ids.size() != pts.size()) {
//     m_next_points_id = 0;
//     for (size_t i = 0; i < m_points[1].size(); i++)
//       m_points_id.push_back(m_next_points_id++);
//   }
//   else {
//     long max = 0;
//     for (size_t i = 0; i < m_points[1].size(); i++) {
//       m_points_id.push_back(ids[i]);
//       if (ids[i] > max)
//         max = ids[i];
//     }
//     m_next_points_id = max + 1;
//   }

//   I.copyTo(m_gray);
// }

void vpSiftOpencv::addFeature(const float &x, const float &y)
{
  cv::Point2f f(x, y);
  m_points[1].push_back(f);
  m_points_id.push_back(m_next_points_id++);
}

void vpSiftOpencv::addFeature(const long &id, const float &x, const float &y)
{
  cv::Point2f f(x, y);
  m_points[1].push_back(f);
  m_points_id.push_back(id);
  if (id >= m_next_points_id)
    m_next_points_id = id + 1;
}

void vpSiftOpencv::addFeature(const cv::Point2f &f)
{
  m_points[1].push_back(f);
  m_points_id.push_back(m_next_points_id++);
}

void vpSiftOpencv::suppressFeature(const int &index)
{
  if (static_cast<size_t>(index) >= m_points[1].size()) {
    throw(vpException(vpException::badValue, "Feature [%d] doesn't exist", index));
  }

  m_points[1].erase(m_points[1].begin() + index);
  m_points_id.erase(m_points_id.begin() + index);
}
END_VISP_NAMESPACE
#else

// Work around to avoid visp_klt library empty when OpenCV is not installed or used
class VISP_EXPORT dummy_vpSiftOpencv
{
public:
  dummy_vpSiftOpencv() { }
};

#if !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_klt.a(vpSiftOpencv.cpp.o) has no symbols
void dummy_vpSiftOpencv_fct() { }
#endif

#endif
