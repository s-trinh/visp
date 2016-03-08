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
 * Detect an object using a cascade of classifiers with the possibility to
 * to track the lost objects with the ViSP template tracker.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

#include <visp3/detection/vpDetectorObject.h>
#include <visp3/core/vpHistogram.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImageTools.h>


#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)

vpDetectorObject::vpDetectorObject() : m_classifierDetector(), m_detectionFilterType(MATCHING),
    m_mapOfDetectedObjects(), m_mapOfLostObjects(), m_maxCooldownTime(5), m_maxLifeTime(5),
    m_messageObjectName("Object"), m_minOverlappingPercentage(25.0),
    m_objectBoundingBoxes(), m_prevI(),
#if defined(VISP_HAVE_MODULE_TT)
    m_recoveryStrategyType(TEMPLATE_TRACKING),
#else
    m_recoveryStrategyType(PREDICTION),
#endif
    m_usePredictionForMatching(true), m_vectorOfDetectedObjects()
#if defined(VISP_HAVE_MODULE_TT)
    , m_trackerParamIterationMax(100), m_trackerParamLambda(0.001), m_trackerParamPyramidLevels(2),
    m_trackerParamPyramidLevelToStop(1), m_trackerParamSample_i(2), m_trackerParamSample_j(2),
    m_trackerParamThresholdGradient(60.0), m_trackerType(vpObjectDetection::TRACKER_SSD_INVERSE_COMPOSITIONAL),
    m_warpType(vpObjectDetection::WARP_AFFINE)
#endif
{

}

vpDetectorObject::vpDetectorObject(const std::string &classifierDataFile) : m_classifierDetector(classifierDataFile),
    m_detectionFilterType(MATCHING), m_mapOfDetectedObjects(), m_mapOfLostObjects(), m_maxCooldownTime(5),
    m_maxLifeTime(5), m_messageObjectName("Object"), m_minOverlappingPercentage(25.0), m_objectBoundingBoxes(),
    m_prevI(),
#if defined(VISP_HAVE_MODULE_TT)
    m_recoveryStrategyType(TEMPLATE_TRACKING),
#else
    m_recoveryStrategyType(PREDICTION),
#endif
    m_usePredictionForMatching(true),
    m_vectorOfDetectedObjects()
#if defined(VISP_HAVE_MODULE_TT)
    , m_trackerParamIterationMax(100), m_trackerParamLambda(0.001), m_trackerParamPyramidLevels(2),
    m_trackerParamPyramidLevelToStop(1), m_trackerParamSample_i(2), m_trackerParamSample_j(2),
    m_trackerParamThresholdGradient(60.0), m_trackerType(vpObjectDetection::TRACKER_SSD_INVERSE_COMPOSITIONAL),
    m_warpType(vpObjectDetection::WARP_AFFINE)
#endif
{

}

/*!
  Compute the detection and the brightness filtering (if set).

  \param I: Image.
  \param scaleFactor: Parameter specifying how much the image size is reduced at each image scale.
  \param minNeighbors: Parameter specifying how many neighbors each candidate rectangle should have to retain it.
  \param minSize: Minimum possible object size. Objects smaller than that are ignored.
  \param maxSize: Maximum possible object size. Objects larger than that are ignored.
*/
void vpDetectorObject::computeDetection(const vpImage<unsigned char> &I, const double scaleFactor, const int minNeighbors,
    const cv::Size &minSize, const cv::Size &maxSize) {
  m_objectBoundingBoxes.clear();
  m_vectorOfDetectedObjects.clear();

  cv::Mat matImg;
  vpImageConvert::convert(I, matImg);

  std::vector<cv::Rect> objects;
  m_classifierDetector.detectMultiScale(matImg, objects, scaleFactor, minNeighbors, 0, minSize, maxSize);

  for(std::vector<cv::Rect>::const_iterator it = objects.begin(); it != objects.end(); ++it) {
    m_objectBoundingBoxes.push_back(vpRect(it->x, it->y, it->width, it->height));
  }
}

/*!
  Compute the matching between detected bounding box and the current list of objects.

  \param I: Image.
*/
void vpDetectorObject::computeMatching(const vpImage<unsigned char> &I) {
  switch(m_detectionFilterType) {
  case NO_DETECTION_FILTER:
  //Add all the current detections in the order returned by the Cascade Classifier
  {
      m_mapOfDetectedObjects.clear();

      int cpt = 1;
      for(std::vector<vpRect>::const_iterator it = m_objectBoundingBoxes.begin();
          it != m_objectBoundingBoxes.end(); ++it, cpt++) {
        m_mapOfDetectedObjects[cpt] = vpObjectDetection(*it, vpObjectDetection::DETECTION_STATUS
#if defined(VISP_HAVE_MODULE_TT)
            , m_trackerType, m_warpType, m_trackerParamIterationMax, m_trackerParamLambda,
            m_trackerParamPyramidLevels, m_trackerParamPyramidLevelToStop, m_trackerParamSample_i,
            m_trackerParamSample_j, m_trackerParamThresholdGradient
#endif
          );
      }
  }
  break;

  case BIGGEST_OBJECT:
  //Add only the biggest detection, thus there is only one detected object
  {
    vpRect max_detection = getBiggestDetection(m_objectBoundingBoxes);

    if(m_mapOfDetectedObjects.empty()) {
      int id = getId();
      m_mapOfDetectedObjects[id] = vpObjectDetection(max_detection, vpObjectDetection::DETECTION_STATUS
#if defined(VISP_HAVE_MODULE_TT)
          , m_trackerType, m_warpType, m_trackerParamIterationMax, m_trackerParamLambda,
          m_trackerParamPyramidLevels, m_trackerParamPyramidLevelToStop, m_trackerParamSample_i,
          m_trackerParamSample_j, m_trackerParamThresholdGradient
#endif
        );
    } else {
      m_mapOfDetectedObjects.begin()->second.addDetection(max_detection, vpObjectDetection::DETECTION_STATUS);
    }
  }
  break;

  case MATCHING:
    //Track and match the detections
    match(I, m_objectBoundingBoxes);
    break;

  default:
    break;
  }

  for(std::map<int, vpObjectDetection>::const_iterator it = m_mapOfDetectedObjects.begin();
      it != m_mapOfDetectedObjects.end(); ++it) {
    m_vectorOfDetectedObjects.push_back(it->second);
  }
}

bool vpDetectorObject::detect(const vpImage<unsigned char> &I) {
  detect2(I);

  return !m_mapOfDetectedObjects.empty();
}

/*!
  Detect using the cascade classifier.

  \param I: RGBa image.
  \param func: Function pointer to eliminate detections according to specific criterion
  \param scaleFactor: Parameter specifying how much the image size is reduced at each image scale.
  \param minNeighbors: Parameter specifying how many neighbors each candidate rectangle should have to retain it.
  \param minSize: Minimum possible object size. Objects smaller than that are ignored.
  \param maxSize: Maximum possible object size. Objects larger than that are ignored.
*/
void vpDetectorObject::detect2(const vpImage<vpRGBa> &I,
    bool (*func)(const vpImage<vpRGBa> &I, const vpRect &boundingBox),
    const double scaleFactor, const int minNeighbors, const cv::Size &minSize, const cv::Size &maxSize) {
  //vpDetectorBase
  m_message.clear();
  m_polygon.clear();
  m_nb_objects = 0;

  if (m_classifierDetector.empty()) {
    clear();
    throw vpException(vpException::fatalError, "Empty classifier !");
  }

  vpImage<unsigned char> grayI;
  vpImageConvert::convert(I, grayI);

  if(m_prevI.getWidth() == 0 || m_prevI.getHeight() == 0) {
    m_prevI = grayI;
  }

  computeDetection(grayI, scaleFactor, minNeighbors, minSize, maxSize);

  //Custom supplied filtering
  if(func != NULL) {
    for(std::vector<vpRect>::iterator it = m_objectBoundingBoxes.begin(); it != m_objectBoundingBoxes.end();) {
      if(!func(I, *it)) {
        m_vectorOfDetectedObjects.push_back(vpObjectDetection(*it, vpObjectDetection::FILTERED_STATUS));
        it = m_objectBoundingBoxes.erase(it);
      } else {
        ++it;
      }
    }
  }

  computeMatching(grayI);

  m_prevI = grayI;

  //vpDetectorBase
  m_nb_objects = m_mapOfDetectedObjects.size();
  for(std::map<int, vpObjectDetection>::const_iterator it = m_mapOfDetectedObjects.begin();
      it != m_mapOfDetectedObjects.end(); ++it) {
    std::stringstream ss_message;
    ss_message << m_messageObjectName << " " << it->first;
    m_message.push_back( ss_message.str() );

    std::vector<vpImagePoint> polygon;
    double x = it->second.m_boundingBox.getLeft();
    double y = it->second.m_boundingBox.getTop();
    double w = it->second.m_boundingBox.getWidth();
    double h = it->second.m_boundingBox.getHeight();

    polygon.push_back(vpImagePoint(y  , x  ));
    polygon.push_back(vpImagePoint(y+h, x  ));
    polygon.push_back(vpImagePoint(y+h, x+w));
    polygon.push_back(vpImagePoint(y  , x+w));

    m_polygon.push_back(polygon);
  }
}

/*!
  Detect using the cascade classifier.

  \param I: Grayscale image.
  \param func: Function pointer to eliminate detections according to specific criterion
  \param scaleFactor: Parameter specifying how much the image size is reduced at each image scale.
  \param minNeighbors: Parameter specifying how many neighbors each candidate rectangle should have to retain it.
  \param minSize: Minimum possible object size. Objects smaller than that are ignored.
  \param maxSize: Maximum possible object size. Objects larger than that are ignored.
*/
void vpDetectorObject::detect2(const vpImage<unsigned char> &I,
    bool (*func)(const vpImage<unsigned char> &I, const vpRect &boundingBox),
    const double scaleFactor, const int minNeighbors, const cv::Size &minSize, const cv::Size &maxSize) {
  //vpDetectorBase
  m_message.clear();
  m_polygon.clear();
  m_nb_objects = 0;

  if (m_classifierDetector.empty()) {
    clear();
    throw vpException(vpException::fatalError, "Empty classifier !");
  }

  if(m_prevI.getWidth() == 0 || m_prevI.getHeight() == 0) {
    m_prevI = I;
  }

  computeDetection(I, scaleFactor, minNeighbors, minSize, maxSize);

  //Custom supplied filtering
  if(func != NULL) {
    for(std::vector<vpRect>::iterator it = m_objectBoundingBoxes.begin(); it != m_objectBoundingBoxes.end();) {
      if(!func(I, *it)) {
        m_vectorOfDetectedObjects.push_back(vpObjectDetection(*it, vpObjectDetection::FILTERED_STATUS));
        it = m_objectBoundingBoxes.erase(it);
      } else {
        ++it;
      }
    }
  }

  computeMatching(I);

  m_prevI = I;

  //vpDetectorBase
  m_nb_objects = m_mapOfDetectedObjects.size();
  for(std::map<int, vpObjectDetection>::const_iterator it = m_mapOfDetectedObjects.begin();
      it != m_mapOfDetectedObjects.end(); ++it) {
    std::stringstream ss_message;
    ss_message << m_messageObjectName << " " << it->first;
    m_message.push_back( ss_message.str() );

    std::vector<vpImagePoint> polygon;
    double x = it->second.m_boundingBox.getLeft();
    double y = it->second.m_boundingBox.getTop();
    double w = it->second.m_boundingBox.getWidth();
    double h = it->second.m_boundingBox.getHeight();

    polygon.push_back(vpImagePoint(y  , x  ));
    polygon.push_back(vpImagePoint(y+h, x  ));
    polygon.push_back(vpImagePoint(y+h, x+w));
    polygon.push_back(vpImagePoint(y  , x+w));

    m_polygon.push_back(polygon);
  }
}

/*!
  Display all the detections.

  \param I: Grayscale image.
  \param thickness: Thickness for the displayed rectangle.
*/
void vpDetectorObject::displayAllDetections(const vpImage<unsigned char> &I, const unsigned int thickness) {
  for(std::vector<vpObjectDetection>::const_iterator it = m_vectorOfDetectedObjects.begin();
      it != m_vectorOfDetectedObjects.end(); ++it) {
    vpColor color = vpColor::red;
    switch(it->m_statusHistory.back()) {
    case vpObjectDetection::DETECTION_STATUS:
      color = vpColor::red;
      break;

    case vpObjectDetection::PREDICTION_STATUS:
      color = vpColor::orange;
      break;

#if defined(VISP_HAVE_MODULE_TT)
    case vpObjectDetection::TRACKING_STATUS:
      color = vpColor::green;
      break;
#endif

    case vpObjectDetection::FILTERED_STATUS:
      color = vpColor::purple;
      break;

    default:
      break;
    }

    vpDisplay::displayRectangle(I, it->m_boundingBox, color, false, thickness);
  }
}

/*!
  Display all the detections.

  \param I: RGBa image.
  \param thickness: Thickness for the displayed rectangle.
*/
void vpDetectorObject::displayAllDetections(const vpImage<vpRGBa> &I, const unsigned int thickness) {
  for(std::vector<vpObjectDetection>::const_iterator it = m_vectorOfDetectedObjects.begin();
      it != m_vectorOfDetectedObjects.end(); ++it) {
    vpColor color = vpColor::red;
    switch(it->m_statusHistory.back()) {
    case vpObjectDetection::DETECTION_STATUS:
      color = vpColor::red;
      break;

    case vpObjectDetection::PREDICTION_STATUS:
      color = vpColor::orange;
      break;

#if defined(VISP_HAVE_MODULE_TT)
    case vpObjectDetection::TRACKING_STATUS:
      color = vpColor::green;
      break;
#endif

    case vpObjectDetection::FILTERED_STATUS:
      color = vpColor::purple;
      break;

    default:
      color = vpColor::black;
      break;
    }

    vpDisplay::displayRectangle(I, it->m_boundingBox, color, false, thickness);
  }
}

/*!
  Display the current list of objects (without filtered detections).

  \param I: Grayscale image.
  \param thickness: Thickness for the displayed rectangle.
*/
void vpDetectorObject::displayDetectedObjects(const vpImage<unsigned char> &I, const unsigned int thickness) {
  for(std::map<int, vpObjectDetection>::const_iterator it = m_mapOfDetectedObjects.begin();
      it != m_mapOfDetectedObjects.end(); ++it) {
    int id = it->first;
    vpColor color = vpColor::red;

    switch(it->second.m_statusHistory.back()) {
    case vpObjectDetection::DETECTION_STATUS:
      color = vpColor::red;
      break;

    case vpObjectDetection::PREDICTION_STATUS:
      color = vpColor::orange;
      break;

#if defined(VISP_HAVE_MODULE_TT)
    case vpObjectDetection::TRACKING_STATUS:
      color = vpColor::green;
      break;
#endif

    case vpObjectDetection::FILTERED_STATUS:
      color = vpColor::purple;
      break;

    default:
      color = vpColor::black;
      break;
    }

    vpDisplay::displayRectangle(I, it->second.m_boundingBox, color, false, thickness);
    std::stringstream ss;
    ss << "[" << id << "], " << it->second.m_boundingBox.getWidth() << "x" << it->second.m_boundingBox.getHeight();
    vpImagePoint topLeft(it->second.m_boundingBox.getTop()-10, it->second.m_boundingBox.getLeft()+10);
    vpDisplay::displayText(I, topLeft, ss.str(), color);
  }
}

/*!
  Display the current list of objects (without filtered detections).

  \param I: RGBa image.
  \param thickness: Thickness for the displayed rectangle.
*/
void vpDetectorObject::displayDetectedObjects(const vpImage<vpRGBa> &I, const unsigned int thickness) {
  for(std::map<int, vpObjectDetection>::const_iterator it = m_mapOfDetectedObjects.begin();
      it != m_mapOfDetectedObjects.end(); ++it) {
    int id = it->first;
    vpColor color = vpColor::red;

    switch(it->second.m_statusHistory.back()) {
    case vpObjectDetection::DETECTION_STATUS:
      color = vpColor::red;
      break;

    case vpObjectDetection::PREDICTION_STATUS:
      color = vpColor::orange;
      break;

#if defined(VISP_HAVE_MODULE_TT)
    case vpObjectDetection::TRACKING_STATUS:
      color = vpColor::green;
      break;
#endif

    case vpObjectDetection::FILTERED_STATUS:
      color = vpColor::purple;
      break;

    default:
      color = vpColor::black;
      break;
    }

    vpDisplay::displayRectangle(I, it->second.m_boundingBox, color, false, thickness);
    std::stringstream ss;
    ss << "[" << id << "], " << it->second.m_boundingBox.getWidth() << "x" << it->second.m_boundingBox.getHeight();
    vpImagePoint topLeft(it->second.m_boundingBox.getTop()-10, it->second.m_boundingBox.getLeft()+10);
    vpDisplay::displayText(I, topLeft, ss.str(), color);
  }
}

void vpDetectorObject::eraseLostObjects() {
  //Update the list of detected objects
  for(std::map<int, vpObjectDetection>::iterator it1 = m_mapOfDetectedObjects.begin();
          it1 != m_mapOfDetectedObjects.end();) {
    //Update vectors in vpObjectDetection to keep a maximum size
    it1->second.clean();

#if defined(VISP_HAVE_MODULE_TT)
    if(it1->second.m_trackingHasDiverged) {
      if(m_mapOfLostObjects.find(it1->first) == m_mapOfLostObjects.end()) {
        m_mapOfLostObjects[it1->first] = m_maxCooldownTime + 1;
      }

      m_mapOfDetectedObjects.erase(it1++);
    } else
#endif

    if(it1->second.m_statusHistory.size() > m_maxLifeTime) {
      bool erase = true;
      for(size_t i = it1->second.m_statusHistory.size()-1; i > it1->second.m_statusHistory.size()-1-m_maxLifeTime; i--) {
        if(it1->second.m_statusHistory[i] == vpObjectDetection::DETECTION_STATUS) {
          //Do not erase if there is at least one detection in the last m_maxLifeTime frames
          erase = false;
          break;
        }
      }

      if(erase) {
        if(m_mapOfLostObjects.find(it1->first) == m_mapOfLostObjects.end()) {
          m_mapOfLostObjects[it1->first] = m_maxCooldownTime + 1;
        }

        m_mapOfDetectedObjects.erase(it1++);
      } else {
        ++it1;
      }
    } else {
      ++it1;
    }
  }

  for(std::map<int, int>::iterator it = m_mapOfLostObjects.begin(); it != m_mapOfLostObjects.end();) {
    //Decrement the lifetime
    it->second--;

    if(it->second <= 0) {
      m_mapOfLostObjects.erase(it++);
    } else {
      ++it;
    }
  }
}

/*!
  Return the biggest bounding box.

  \param objectBoundingBoxes: Vector of bounding boxes corresponding to the current detections.
  \return The biggest bounding box rectangle.
*/
vpRect vpDetectorObject::getBiggestDetection(const std::vector<vpRect> &objectBoundingBoxes) {
  vpRect bbMax;

  for(std::vector<vpRect>::const_iterator it = objectBoundingBoxes.begin(); it != objectBoundingBoxes.end(); ++it) {
    if(it->getWidth()*it->getHeight() > bbMax.getWidth()*bbMax.getHeight()) {
      bbMax = *it;
    }
  }

  return bbMax;
}

/*!
  Get the first available identifier.

  \return The id.
*/
int vpDetectorObject::getId() {
  int cpt = 1;
  for(std::map<int, vpObjectDetection>::const_iterator it = m_mapOfDetectedObjects.begin();
      it != m_mapOfDetectedObjects.end(); ++it, cpt++) {
    //Get the first available id and with a cooldown time finished
    if(m_mapOfDetectedObjects.find(cpt) == m_mapOfDetectedObjects.end() &&
        m_mapOfLostObjects.find(cpt) == m_mapOfLostObjects.end()) {
      return cpt;
    }
  }

  while(m_mapOfDetectedObjects.find(cpt) != m_mapOfDetectedObjects.end() ||
        m_mapOfLostObjects.find(cpt) != m_mapOfLostObjects.end()) {
    cpt++;
  }

  return cpt;
}

/*!
  Return the percentage of overlapping for two rectangles.

  \param r1: First rectangle.
  \param r2: Second rectangle.
  \return The percentage of overlapping or -1.0 if they don't overlap.
*/
double vpDetectorObject::getPercentageOfOverlap(const vpRect &r1, const vpRect &r2) {
  double left = (std::max)(r1.getLeft(), r2.getLeft());
  double right = (std::min)(r1.getRight(), r2.getRight());
  double bottom = (std::min)(r1.getBottom(), r2.getBottom());
  double top = (std::max)(r1.getTop(), r2.getTop());

  if(left < right && bottom > top) {
    double overlap_area = (right-left)*(bottom-top);
    double r1_area = r1.getWidth()*r1.getHeight();
    double r2_area = r2.getWidth()*r2.getHeight();
    double percentage = overlap_area / (r1_area + r2_area - overlap_area) * 100.0;
    return percentage;
  }

  return -1;
}

/*!
  Load the classifier with the supplied training file.

  \param classifierDataFile: Filepath to the training file.
  \return True if the loading is ok, false otherwise.
*/
bool vpDetectorObject::load(const std::string &classifierDataFile) {
  return m_classifierDetector.load(classifierDataFile);
}

/*!
  Match the current detections with the list of objects.

  \param I: The grayscale image.
  \param objectBoundingBoxes: The vector of bounding boxes for the current detections.
*/
void vpDetectorObject::match(const vpImage<unsigned char> &
#if defined(VISP_HAVE_MODULE_TT)
    I
#endif
    , const std::vector<vpRect> &objectBoundingBoxes) {

  if(m_mapOfDetectedObjects.empty()) {
    int cpt = 1;
    //Add all the detections
    for(std::vector<vpRect>::const_iterator it = objectBoundingBoxes.begin(); it != objectBoundingBoxes.end(); ++it, cpt++) {
      m_mapOfDetectedObjects[cpt] = vpObjectDetection(*it, vpObjectDetection::DETECTION_STATUS
#if defined(VISP_HAVE_MODULE_TT)
          , m_trackerType, m_warpType, m_trackerParamIterationMax, m_trackerParamLambda,
          m_trackerParamPyramidLevels, m_trackerParamPyramidLevelToStop, m_trackerParamSample_i,
          m_trackerParamSample_j, m_trackerParamThresholdGradient
#endif
        );
    }
  } else {
    //Set the state of the objects
    for(std::map<int, vpObjectDetection>::iterator it = m_mapOfDetectedObjects.begin();
        it != m_mapOfDetectedObjects.end(); ++it) {
      it->second.m_update = false;
    }

    //Iterate over the detections
    for(std::vector<vpRect>::const_iterator it1 = objectBoundingBoxes.begin(); it1 != objectBoundingBoxes.end(); ++it1) {
      double max_overlapping_percentage = 0;
      std::map<int, vpObjectDetection>::iterator it_closest = m_mapOfDetectedObjects.end();
      vpRect closest_rect;

      //Find the closest object
      for(std::map<int, vpObjectDetection>::iterator it2 = m_mapOfDetectedObjects.begin();
          it2 != m_mapOfDetectedObjects.end(); ++it2) {
        vpRect prevBoundingBox(it2->second.m_boundingBoxHistory.back());
        if(m_usePredictionForMatching) {
          if(it2->second.m_boundingBoxHistory.size() > 1) {
            size_t index = it2->second.m_boundingBoxHistory.size()-1;
            vpImagePoint shift = it2->second.m_boundingBoxHistory[index].getCenter() -
                it2->second.m_boundingBoxHistory[index-1].getCenter();

            vpImagePoint top_left = it2->second.m_boundingBoxHistory[index].getTopLeft() + shift;
            vpRect rect_prediction(top_left, it2->second.m_boundingBoxHistory[index].getWidth(),
                it2->second.m_boundingBoxHistory[index].getHeight());

            prevBoundingBox = rect_prediction;
          }
        }

        double percentage_of_overlap = getPercentageOfOverlap(prevBoundingBox, *it1);
        if(percentage_of_overlap > max_overlapping_percentage) {
          max_overlapping_percentage = percentage_of_overlap;
          it_closest = it2;
          closest_rect = *it1;
        }
      }

      if(max_overlapping_percentage >= m_minOverlappingPercentage) {
        //Found a match
        it_closest->second.addDetection(closest_rect, vpObjectDetection::DETECTION_STATUS);
        it_closest->second.m_update = true;
      } else {
        //Add a new object
        int id = getId();
        m_mapOfDetectedObjects[id] = vpObjectDetection(*it1, vpObjectDetection::DETECTION_STATUS
#if defined(VISP_HAVE_MODULE_TT)
            , m_trackerType, m_warpType, m_trackerParamIterationMax, m_trackerParamLambda,
            m_trackerParamPyramidLevels, m_trackerParamPyramidLevelToStop, m_trackerParamSample_i,
            m_trackerParamSample_j, m_trackerParamThresholdGradient
#endif
          );
      }
    }

    //Predict or track the position of non matched objects
    for(std::map<int, vpObjectDetection>::iterator it = m_mapOfDetectedObjects.begin();
        it != m_mapOfDetectedObjects.end(); ++it) {
      if(!it->second.m_update) {
        switch(m_recoveryStrategyType) {
        case NO_RECOVER:
          //Add the last detected bounding box
          it->second.addDetection(it->second.m_boundingBoxHistory.back(), vpObjectDetection::PREDICTION_STATUS);
          break;

        case PREDICTION:
        //Predict the position for lost objects using a constant velocity model
        {
          if(it->second.m_boundingBoxHistory.size() > 1) {
            size_t index = it->second.m_boundingBoxHistory.size()-1;
            vpImagePoint shift = it->second.m_boundingBoxHistory[index].getCenter() -
                it->second.m_boundingBoxHistory[index-1].getCenter();

            vpImagePoint top_left = it->second.m_boundingBoxHistory[index].getTopLeft() + shift;
            vpRect rect_prediction(top_left, it->second.m_boundingBoxHistory[index].getWidth(),
                it->second.m_boundingBoxHistory[index].getHeight());

            it->second.addDetection(rect_prediction, vpObjectDetection::PREDICTION_STATUS);
          } else {
            it->second.addDetection(it->second.m_boundingBoxHistory.back(), vpObjectDetection::PREDICTION_STATUS);
          }
        }
        break;

#if defined(VISP_HAVE_MODULE_TT)
        case TEMPLATE_TRACKING:
        //Use the template tracker to track an object that was not detected by the Cascade Classifier
        {
          if(it->second.m_statusHistory.back() == vpObjectDetection::DETECTION_STATUS) {
            it->second.initTracking(m_prevI);
            it->second.track(I);
          } else {
            it->second.track(I);
          }
        }
        break;
#endif

        default:
          break;
        }
      }
    }
  }

  //Update the list of detected objects
  eraseLostObjects();
}

#endif
