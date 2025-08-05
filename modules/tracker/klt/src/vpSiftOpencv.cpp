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

#include <opencv2/video/tracking.hpp>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/klt/vpSiftOpencv.h>

#define USE_GFTT 0

static const bool debug_print = false;
static const bool debug_display = true;
static const bool use_SIFT_KLT = true;

BEGIN_VISP_NAMESPACE
vpSiftOpencv::vpSiftOpencv(bool useAKAZE, const MatchingFilterType &type)
  : m_gray(), m_points_id(),
  m_next_points_id(0),
  m_siftDetector(),
  m_keyPointsRef(), m_keyPointsCur(),
  m_descriptorsRef(), m_descriptorsCur(),
  m_descriptorsMatcher(), m_knnMatches(), m_ratioThreshold(0.65), m_matches01(), m_matches10(),
  m_filterType(type), m_AKAZE(useAKAZE), m_history(0)
{
  cv::NormTypes normType = cv::NORM_L2;
  if (m_AKAZE) {
    normType = cv::NORM_HAMMING;
    m_siftDetector = cv::AKAZE::create();
    // m_siftDetector = cv::BRISK::create();
    // m_siftDetector = cv::ORB::create();

    // const int nfeatures = 100;
    // m_siftDetector = cv::ORB::create(nfeatures);
  }
  else {
    m_siftDetector = cv::SiftFeatureDetector::create();
  }

  if (m_filterType == RatioTest) {
    if (m_AKAZE) {
      m_descriptorsMatcher = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
    }
    else {
      m_descriptorsMatcher = cv::FlannBasedMatcher::create();
    }
  }
  else {
    const bool cross_check = m_filterType == CrossCheck;
    m_descriptorsMatcher = cv::BFMatcher::create(normType, cross_check);
  }
}

vpSiftOpencv::vpSiftOpencv(const vpSiftOpencv &copy)
  : m_gray(), m_points_id(),
  m_next_points_id(0),
  m_siftDetector(),
  m_keyPointsRef(), m_keyPointsCur(),
  m_descriptorsRef(), m_descriptorsCur(),
  m_descriptorsMatcher(), m_knnMatches(), m_ratioThreshold(0.65), m_matches01(), m_matches10(),
  m_filterType(MatchingFilterType::CrossCheck), m_AKAZE(false), m_history(0)
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
  m_ratioThreshold = copy.m_ratioThreshold;
  m_matches01 = copy.m_matches01;
  m_matches10 = copy.m_matches10;
  m_filterType = copy.m_filterType;
  m_AKAZE = copy.m_AKAZE;

  m_history = copy.m_history;

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

  m_keyPointsCur.clear();
#if 1 // !USE_GFTT
  // m_siftDetector->detectAndCompute(I, cv::noArray(), m_keyPointsCur, m_descriptorsCur);
  m_siftDetector->detectAndCompute(m_gray, mask, m_keyPointsCur, m_descriptorsCur);

  for (size_t i = 0; i < m_keyPointsCur.size(); i++) {
    m_points[1].push_back(m_keyPointsCur[i].pt);
  }
#else // This does not work, need some attributes and need to use GFTT
  int maxCount = 10000;
  double qualityLevel = 0.01;
  double minDistance = 15;
  int blockSize = 3;
  double harris_k = 0.04;

  cv::goodFeaturesToTrack(m_gray, m_points[1], maxCount, qualityLevel, minDistance, mask, blockSize, false,
                          harris_k);

  if (m_points[1].size() > 0) {
    std::cout << "[goodFeaturesToTrack] m_points[1]=" << m_points[1].size() << std::endl;

    int winSize = 11;
    cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);

    cv::cornerSubPix(m_gray, m_points[1], cv::Size(winSize, winSize), cv::Size(-1, -1), termcrit);

    m_keyPointsCur.reserve(m_points[1].size());
    for (size_t i = 0; i < m_points[1].size(); i++) {
      m_keyPointsCur.push_back(cv::KeyPoint(cv::Point2f(m_points[1][i].x, m_points[1][i].y), 4.800000191));
    }

    m_siftDetector->compute(m_gray, m_keyPointsCur, m_descriptorsCur);
  }
#endif

  for (size_t i = 0; i < m_points[1].size(); i++) {
    m_points_id.push_back(m_next_points_id++);
  }

  if (debug_print) {
    std::cout << "[initTracking] m_points[1]=" << m_points[1].size() << " ; m_points_id=" << m_points_id.size() << std::endl;
  }

  if (debug_display) {
    // Debug
    cv::cvtColor(I, m_leftMat, cv::COLOR_GRAY2BGR);
    m_displayMat = cv::Mat3b(I.rows, 2*I.cols);
  }




  // m_next_points_id = 0;

  // I.copyTo(m_gray);

  // for (size_t i = 0; i < 2; i++) {
  //   m_points[i].clear();
  // }

  // m_points_id.clear();

  // cv::goodFeaturesToTrack(m_gray, m_points[1], m_maxCount, m_qualityLevel, m_minDistance, mask, m_blockSize, false,
  //                         m_harris_k);

  // if (m_points[1].size() > 0) {
  //   cv::cornerSubPix(m_gray, m_points[1], cv::Size(m_winSize, m_winSize), cv::Size(-1, -1), m_termcrit);

  //   for (size_t i = 0; i < m_points[1].size(); i++)
  //     m_points_id.push_back(m_next_points_id++);
  // }
}

template<typename type>
void mat2vec(const cv::Mat &mat, std::vector<std::vector<type>> &vec_of_vec)
{
  std::vector<type> vec;
  vec.resize(mat.cols);

  for (int i = 0; i < mat.rows; i++) {
    for (int j = 0; j < mat.cols; j++) {
      vec[j] = mat.at<type>(i, j);
    }

    vec_of_vec[i] = vec;
  }
}

void trimMat(const cv::Mat &matRef, const std::vector<int> &vec_idx, cv::Mat &mat)
{
  mat = cv::Mat(vec_idx.size(), matRef.cols, matRef.type());

  for (size_t i = 0; i < vec_idx.size(); i++) {
    cv::Mat dest = mat(cv::Range(i, i+1), cv::Range::all());
    matRef(cv::Range(vec_idx[i], vec_idx[i]+1), cv::Range::all()).copyTo(dest);
  }
}

void trimMatInv(const cv::Mat &matRef, const std::vector<int> &vec_idx, cv::Mat &mat)
{
  std::vector<int> vec_idx_correct;
  vec_idx_correct.reserve(matRef.rows - static_cast<int>(vec_idx.size()));

  for (int i = 0; i < matRef.rows; i++) {
    if (static_cast<int>(i) != vec_idx[i]) {
      vec_idx_correct.push_back(i);
    }
  }

  trimMat(matRef, vec_idx_correct, mat);


  // mat = cv::Mat(matRef.rows - vec_idx.size(), matRef.cols, matRef.type());

  // for (size_t i = 0; i < vec_idx.size(); i++) {
  //   if (static_cast<int>(i) != vec_idx[i]) {
  //     cv::Mat dest = mat(cv::Range(i, i+1), cv::Range::all());
  //     matRef(cv::Range(vec_idx[i], vec_idx[i]+1), cv::Range::all()).copyTo(dest);
  //   }
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
#if 1 // !USE_GFTT
  m_siftDetector->detectAndCompute(I, cv::noArray(), m_keyPointsCur, m_descriptorsCur);
  // std::cout << "GFTT size=" << m_keyPointsCur[0].size << " ; class_id=" << m_keyPointsCur[0].class_id << " ; response=" << m_keyPointsCur[0].response << std::endl;
#else // This does not work, need some attributes and need to use GFTT
  int maxCount = 10000;
  double qualityLevel = 0.01;
  double minDistance = 15;
  int blockSize = 3;
  double harris_k = 0.04;

  m_points[1].clear();
  cv::goodFeaturesToTrack(I, m_points[1], maxCount, qualityLevel, minDistance, cv::noArray(), blockSize, false,
                          harris_k);

  if (m_points[1].size() > 0) {
    // std::cout << "[goodFeaturesToTrack] m_points[1]=" << m_points[1].size() << std::endl;

    int winSize = 11;
    cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);

    cv::cornerSubPix(m_gray, m_points[1], cv::Size(winSize, winSize), cv::Size(-1, -1), termcrit);

    m_keyPointsCur.reserve(m_points[1].size());
    for (size_t i = 0; i < m_points[1].size(); i++) {
      m_keyPointsCur.push_back(cv::KeyPoint(cv::Point2f(m_points[1][i].x, m_points[1][i].y), 4.800000191));
    }

    m_siftDetector->compute(I, m_keyPointsCur, m_descriptorsCur);
  }
#endif

  m_matches01.clear();
  m_matches10.clear();
  if (debug_print) {
    std::cout << "[track] m_keyPointsCur=" << m_keyPointsCur.size() << " ; m_keyPointsRef=" << m_keyPointsRef.size() << std::endl;
    std::cout << "[track] m_descriptorsCur=" << m_descriptorsCur.rows << "x" << m_descriptorsCur.cols << " ; type=" << m_descriptorsCur.type() << " ; CV_8U=" << CV_8U << std::endl;
    std::cout << "[track] m_descriptorsRef=" << m_descriptorsRef.rows << "x" << m_descriptorsRef.cols << " ; type=" << m_descriptorsRef.type() << " ; CV_32F=" << CV_32F << std::endl;
  }
  if (m_filterType == RatioTest) {
    m_knnMatches.clear();
    m_descriptorsMatcher->clear();

    // Match train to query
    m_descriptorsMatcher->add(std::vector<cv::Mat>(1, m_descriptorsCur));
    m_descriptorsMatcher->knnMatch(m_descriptorsRef, m_knnMatches, 2);

    for (size_t idx = 0; idx < m_knnMatches.size(); idx++) {
      if (m_knnMatches[idx].size() >= 2) {
        float ratio = m_knnMatches[idx][0].distance / m_knnMatches[idx][1].distance;

        if (ratio <= m_ratioThreshold) {
          m_matches10.push_back(cv::DMatch(m_knnMatches[idx][0].trainIdx, m_knnMatches[idx][0].queryIdx, m_knnMatches[idx][0].distance));
        }
      }
    }
  }
  else {
    // Match train to query
    m_descriptorsMatcher->match(m_descriptorsRef, m_descriptorsCur, m_matches01);
    m_matches10.reserve(m_matches01.size());

    for (size_t i = 0; i < m_matches01.size(); i++) {
      const cv::DMatch &m01 = m_matches01[i];
      m_matches10.push_back(cv::DMatch(m01.trainIdx, m01.queryIdx, m01.distance));
    }

    if (use_SIFT_KLT) {
      bool initial_guess = true;
      if (m_prevGray.empty()) {
        initial_guess = false;
      }

      cv::swap(m_prevGray, m_gray);
      I.copyTo(m_gray);
      if (m_prevGray.empty()) {
        m_gray.copyTo(m_prevGray);
      }

      if (initial_guess) {
        int flags = cv::OPTFLOW_USE_INITIAL_FLOW;
        int pyrMaxLevel = 3;
        cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
        int winSize = 11;
        double minEigThreshold = 1e-4;
        std::vector<float> err;

        // std::swap(m_points[1], m_points[0]);
        m_points[0].clear();
        m_points[1].clear();
        for (size_t i = 0; i < m_matches10.size(); i++) {
          const cv::DMatch &match = m_matches10[i];
          m_points[0].push_back(m_keyPointsRef[match.trainIdx].pt);
          m_points[1].push_back(m_keyPointsCur[match.queryIdx].pt);
        }

        std::vector<uchar> status;
        cv::calcOpticalFlowPyrLK(m_prevGray, m_gray, m_points[0], m_points[1], status, err, cv::Size(winSize, winSize),
                                 pyrMaxLevel, termcrit, flags, minEigThreshold);
        // std::cout << "[calcOpticalFlowPyrLK] m_matches10=" << m_matches10.size() << " ; status=" << status.size() << std::endl;

        // Remove points that are lost
        for (int i = static_cast<int>(status.size()) - 1; i >= 0; i--) {
          if (status[static_cast<size_t>(i)] == 0) { // point is lost
            m_matches10.erase(m_matches10.begin() + i);
          }
          else {
            m_keyPointsCur[m_matches10[i].queryIdx].pt = m_points[1][i];
          }
        }
      }
    }
  }
  if (debug_print) {
    std::cout << "[track] m_filterType=" << m_filterType << std::endl;
    std::cout << "[track] m_matches01=" << m_matches01.size() << " ; m_matches10=" << m_matches10.size() << " ; m_points_id=" << m_points_id.size() << std::endl;
  }

  // Remove query points that are matches to the same train points
  if (m_filterType != CrossCheck && m_matches10.size() > 1) {
    // std::vector<int> vec_idx;

    // for (int idx1 = static_cast<int>(m_matches10.size())-1; idx1 >= 0; idx1--) {
    //   bool same_train = false;
    //   int train_idx = m_matches10[idx1].trainIdx;

    //   for (int idx2 = 0; idx2 < idx1 && !same_train; idx2++) {
    //     if (m_matches10[idx2].trainIdx == train_idx) {
    //       same_train = true;
    //     }
    //   }

    //   if (same_train) {
    //     // m_points_id.erase(m_points_id.begin() + idx1);
    //     m_matches10.erase(m_matches10.begin() + idx1);
    //   }
    //   else {
    //     vec_idx.push_back(m_matches10[idx1].queryIdx);
    //   }
    // }

    // if (debug_print) {
    //   std::cout << "[track] m_matches01=" << m_matches01.size() << " ; m_matches10=" << m_matches10.size() << " ; vec_idx=" << vec_idx.size() << std::endl;
    // }

    // cv::Mat descriptorsCur_trim;
    // trimMat(m_descriptorsCur, vec_idx, descriptorsCur_trim);
    // // m_descriptorsCur = descriptorsCur_trim.clone();
    // m_descriptorsCur = descriptorsCur_trim;
  }
  if (debug_print) {
    std::cout << "[track] m_matches01=" << m_matches01.size() << " ; m_matches10=" << m_matches10.size() << " ; m_points_id=" << m_points_id.size() << std::endl;
  }

  m_points[0].clear();
  m_points[1].clear();
  std::vector<long> points_id = m_points_id;
  // m_points_id = std::vector<long>(m_keyPointsCur.size(), -1);
  m_points_id.clear();

  if (m_matches10.empty()) {
    std::cerr << "No match!" << std::endl;
    return;
  }

  cv::Mat descriptorsCur(m_matches10.size(), m_descriptorsCur.cols, m_descriptorsCur.type());
  std::vector<cv::KeyPoint> keyPointsCur;
  keyPointsCur.reserve(m_matches10.size());

  for (size_t i = 0; i < m_matches10.size(); i++) {
    const cv::DMatch &match = m_matches10[i];
    m_points[0].push_back(m_keyPointsRef[match.trainIdx].pt);
    m_points[1].push_back(m_keyPointsCur[match.queryIdx].pt);
    m_points_id.push_back(points_id[match.trainIdx]);

    cv::Mat dest = descriptorsCur(cv::Range(i, i+1), cv::Range::all());
    m_descriptorsCur(cv::Range(match.queryIdx, match.queryIdx+1), cv::Range::all()).copyTo(dest);
    keyPointsCur.push_back(m_keyPointsCur[match.queryIdx]);
  }
  m_descriptorsCur = descriptorsCur;
  m_keyPointsCur = keyPointsCur;

  if (debug_print) {
    std::cout << "[track] After filter, m_keyPointsCur=" << m_keyPointsCur.size() << std::endl;
    std::cout << "[track] After filter, m_descriptorsCur=" << m_descriptorsCur.rows << "x" << m_descriptorsCur.cols << std::endl;
  }

  if (debug_display) {
    // Debug
    cv::Mat I_color;
    cv::cvtColor(I, I_color, cv::COLOR_GRAY2BGR);

    m_leftMat.copyTo(m_displayMat(cv::Rect(0, 0, m_leftMat.cols, m_leftMat.rows)));
    I_color.copyTo(m_displayMat(cv::Rect(m_leftMat.cols, 0, I_color.cols, I_color.rows)));
    if (debug_print) {
      std::cout << "[track] m_points[0]=" << m_points[0].size() << " ; m_points[1]=" << m_points[1].size() << " ; m_points_id=" << m_points_id.size() << std::endl;
    }

    for (size_t i = 0; i < m_points[0].size(); i++) {
      cv::line(m_displayMat, m_points[0][i], cv::Point(m_points[1][i].x + I_color.cols, m_points[1][i].y), cv::Scalar(0, 255, 0));

      cv::drawMarker(m_displayMat, m_points[0][i], cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 8);
      cv::drawMarker(m_displayMat, cv::Point(m_points[1][i].x + I_color.cols, m_points[1][i].y), cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 8);

      std::ostringstream id;
      id << m_points_id[i];
      cv::putText(m_displayMat, id.str(), cv::Point(m_points[0][i].x + 5, m_points[0][i].y), 0, 0.4, cv::Scalar(0, 0, 255));
    }

    cv::imshow("DEBUG", m_displayMat);
    cv::waitKey(1);

    m_leftMat = I_color;
  }



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
  // TODO:
  throw std::runtime_error("DISABLE vpSiftOpencv::addFeature");

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
    throw(vpException(vpException::badValue, "Feature [%d] doesn't exist ; m_points_id.size=%d", index, static_cast<int>(m_points_id.size())));
  }

  // std::cout << "m_keyPointsCur=" << m_keyPointsCur.size() << " ; m_descriptorsCur=" << m_descriptorsCur.rows << std::endl;

  m_points[1].erase(m_points[1].begin() + index);
  m_points_id.erase(m_points_id.begin() + index);
  m_keyPointsCur.erase(m_keyPointsCur.begin() + index);

  cv::Mat descriptorsCur;
  std::vector<int> correct_idx;
  correct_idx.push_back(index);
  trimMatInv(m_descriptorsCur, correct_idx, descriptorsCur);
  m_descriptorsCur = descriptorsCur;
}

void vpSiftOpencv::suppressFeatures(const cv::Mat &mask)
{
  std::vector<int> correct_idx;
  for (size_t i = 0; i < m_keyPointsCur.size(); i++) {
    if (mask.at<uchar>(m_keyPointsCur[i].pt) != 0) {
      correct_idx.push_back(i);
    }
  }

  cv::Mat descriptorsCur;
  trimMat(m_descriptorsCur, correct_idx, descriptorsCur);
  m_descriptorsCur = descriptorsCur;

  for (int i = static_cast<int>(m_keyPointsCur.size()) - 1; i >= 0; i--) {
    if (mask.at<uchar>(m_keyPointsCur[i].pt) == 0) {
      m_points[0].erase(m_points[0].begin() + i);
      m_points[1].erase(m_points[1].begin() + i);
      m_points_id.erase(m_points_id.begin() + i);

      m_keyPointsCur.erase(m_keyPointsCur.begin() + i);
    }
  }
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
