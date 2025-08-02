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
  \file vpSiftOpencv.h

  \brief Wrapper for the KLT (Kanade-Lucas-Tomasi) feature tracker
  implemented with opencv.
*/

#ifndef VP_SIFT_OPENCV_H
#define VP_SIFT_OPENCV_H

#include <visp3/core/vpConfig.h>

#if defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)

#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json.hpp)
#endif

BEGIN_VISP_NAMESPACE
/*!
 * \class vpSiftOpencv
 *
 * \ingroup module_klt
 *
 * \brief Wrapper for the KLT (Kanade-Lucas-Tomasi) feature tracker
 * implemented in OpenCV. Thus to enable this class OpenCV should be installed.
 * Installation instructions are provided here
 * https://visp.inria.fr/3rd_opencv.
 *
 * The following example available in tutorial-klt-tracker.cpp shows how to use
 * the main functions of the class.
 *
 * \include tutorial-klt-tracker.cpp
 *
 * A line by line explanation is provided in \ref tutorial-tracking-keypoint.
*/
class VISP_EXPORT vpSiftOpencv
{
public:
  enum MatchingFilterType
  {
    None,
    CrossCheck,
    RatioTest
  };

  /*!
   * Default constructor.
   */
  // vpSiftOpencv(bool useAKAZE = true, const MatchingFilterType &type = None);
  // vpSiftOpencv(bool useAKAZE = true, const MatchingFilterType &type = CrossCheck);
  // vpSiftOpencv(bool useAKAZE = false, const MatchingFilterType &type = RatioTest);
  vpSiftOpencv(bool useAKAZE = false, const MatchingFilterType &type = CrossCheck);
  /*!
   * Copy constructor.
   */
  vpSiftOpencv(const vpSiftOpencv &copy);
  /*!
   * Destructor.
   */
  virtual ~vpSiftOpencv();

  /*!
   * Add a keypoint at the end of the feature list. The id of the feature is set
   * to ensure that it is unique.
   *
   * \param x : Coordinates along x-axis of the feature in the image.
   * \param y : Coordinates along y-axis of the feature in the image.
   */
  void addFeature(const float &x, const float &y);

  /*!
   * Add a keypoint at the end of the feature list.
   *
   * \warning This function doesn't ensure that the id of the feature is unique.
   * You should rather use addFeature(const float &, const float &) or
   * addFeature(const cv::Point2f &).
   *
   * \param id : Feature id. Should be unique
   * \param x : Coordinates along x-axis of the feature in the image.
   * \param y : Coordinates along y-axis of the feature in the image.
   */
  void addFeature(const long &id, const float &x, const float &y);

  /*!
   * Add a keypoint at the end of the feature list. The id of the feature is set
   * to ensure that it is unique.
   * \param f : Coordinates of the feature in the image.
   */
  void addFeature(const cv::Point2f &f);

  /*!
   * Display features position and id.
   *
   * \param I : Image used as background. Display should be initialized on it.
   * \param color : Color used to display the features.
   * \param thickness : Thickness of the drawings.
   */
  void display(const vpImage<unsigned char> &I, const vpColor &color = vpColor::red, unsigned int thickness = 1) const;
  /*!
   * Display features list.
   *
   * \param I : The image used as background.
   * \param features : Vector of features.
   * \param color : Color used to display the points.
   * \param thickness : Thickness of the points.
   */
  static void display(const vpImage<unsigned char> &I, const std::vector<cv::Point2f> &features,
                      const vpColor &color = vpColor::green, unsigned int thickness = 1);
  /*!
   * Display features list.
   *
   * \param I : The image used as background.
   * \param features : Vector of features.
   * \param color : Color used to display the points.
   * \param thickness : Thickness of the points.
   */
  static void display(const vpImage<vpRGBa> &I, const std::vector<cv::Point2f> &features,
                      const vpColor &color = vpColor::green, unsigned int thickness = 1);
  /*!
   * Display features list with ids.
   *
   * \param I : The image used as background.
   * \param features : Vector of features.
   * \param featuresid : Vector of ids corresponding to the features.
   * \param color : Color used to display the points.
   * \param thickness : Thickness of the points
   */
  static void display(const vpImage<unsigned char> &I, const std::vector<cv::Point2f> &features,
                        const std::vector<long> &featuresid, const vpColor &color = vpColor::green,
                        unsigned int thickness = 1);
  /*!
   * Display features list with ids.
   *
   * \param I : The image used as background.
   * \param features : Vector of features.
   * \param featuresid : Vector of ids corresponding to the features.
   * \param color : Color used to display the points.
   * \param thickness : Thickness of the points
   */
  static void display(const vpImage<vpRGBa> &I, const std::vector<cv::Point2f> &features,
                      const std::vector<long> &featuresid, const vpColor &color = vpColor::green,
                      unsigned int thickness = 1);

  /*!
   * Get the 'index'th feature image coordinates.  Beware that
   * getFeature(i,...) may not represent the same feature before and
   * after a tracking iteration (if a feature is lost, features are
   * shifted in the array).
   *
   * \param index : Index of feature.
   * \param id : id of the feature.
   * \param x : x coordinate.
   * \param y : y coordinate.
   */
  void getFeature(const int &index, long &id, float &x, float &y) const;
  //! Get the list of current features.
  std::vector<cv::Point2f> getFeatures() const { return m_points[1]; }
  // CvPoint2D32f* getFeatures() const {return features;}
  //! Get the unique id of each feature.
  std::vector<long> getFeaturesId() const { return m_points_id; }
  //! Get the number of current features
  int getNbFeatures() const { return static_cast<int>(m_points[1].size()); }
  //! Get the number of previous features.
  int getNbPrevFeatures() const { return static_cast<int>(m_points[0].size()); }
  // void getPrevFeature(int index, int &id, float &x, float &y) const;
  //! Get the list of previous features
  std::vector<cv::Point2f> getPrevFeatures() const { return m_points[0]; }

  /*!
   * Initialise the tracking by extracting KLT keypoints on the provided image.
   *
   * \param I : Grey level image used as input. This image should have only 1 channel.
   * \param mask : Image mask used to restrict the keypoint detection
   * area. If mask is nullptr, all the image will be considered.
   *
   * \exception vpTrackingException::initializationError : If the image I is not
   * initialized, or if the image or the mask have bad coding format.
   */
  void initTracking(const cv::Mat &I, const cv::Mat &mask = cv::Mat());

  // /*!
  //  * Set the points that will be used as initialization during the next call to
  //  * track().
  //  *
  //  * \param I : Input image.
  //  * \param pts : Vector of points that should be tracked.
  //  */
  // void initTracking(const cv::Mat &I, const std::vector<cv::Point2f> &pts);

  // /*!
  //  * Set the points that will be used as initialization during the next call to
  //  * track().
  //  *
  //  * \param I : Input image.
  //  * \param pts : Vector of points that should be tracked.
  //  * \param ids : Corresponding point ids.
  //  */
  // void initTracking(const cv::Mat &I, const std::vector<cv::Point2f> &pts, const std::vector<long> &ids);

  /*!
   * Copy operator.
   */
  vpSiftOpencv &operator=(const vpSiftOpencv &copy);

  /*!
   * Track KLT keypoints using the iterative Lucas-Kanade method with pyramids.
   *
   * \param I : Input image.
   */
  void track(const cv::Mat &I);

  //! Does nothing. Just here for compat with previous releases that use
  //! OpenCV C api to do the tracking.
  void setTrackerId(int tid) { (void)tid; }

  /*!
   * Remove the feature with the given index as parameter.
   *
   * \param index : Index of the feature to remove.
   */
  void suppressFeature(const int &index);

  float getRatioThreshold() const
  {
    return m_ratioThreshold;
  }

  void setRatioThreshold(float ratio)
  {
    m_ratioThreshold = ratio;
  }

#ifdef VISP_HAVE_NLOHMANN_JSON
  friend void to_json(nlohmann::json &j, const vpSiftOpencv &array);
  friend void from_json(const nlohmann::json &j, vpSiftOpencv &array);
#endif

protected:
  cv::Mat m_gray; //!< Gray image
  std::vector<cv::Point2f> m_points[2]; //!< Previous [0] and current [1] keypoint location
  std::vector<long> m_points_id;        //!< Keypoint id
  long m_next_points_id; //!< Id for the newt keypoint
  cv::Ptr<cv::FeatureDetector> m_siftDetector;
  std::vector<cv::KeyPoint> m_keyPointsRef;
  std::vector<cv::KeyPoint> m_keyPointsCur;
  cv::Mat m_descriptorsRef;
  cv::Mat m_descriptorsCur;
  cv::Ptr<cv::DescriptorMatcher> m_descriptorsMatcher;
  std::vector<std::vector<cv::DMatch> > m_knnMatches;
  float m_ratioThreshold;
  std::vector<cv::DMatch> m_matches01;
  std::vector<cv::DMatch> m_matches10;
  MatchingFilterType m_filterType;
  bool m_AKAZE;
  int m_history;

  cv::Mat m_leftMat;
  cv::Mat m_displayMat;
};

#ifdef VISP_HAVE_NLOHMANN_JSON
inline void to_json(nlohmann::json &j, const vpSiftOpencv &klt)
{ }

inline void from_json(const nlohmann::json &j, vpSiftOpencv &klt)
{ }
#endif

END_VISP_NAMESPACE
#endif
#endif
