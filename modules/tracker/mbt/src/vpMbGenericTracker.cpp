#include <iostream>

#include <visp3/mbt/vpMbGenericTracker.h>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/mbt/vpMbtXmlParser.h>
#include <visp3/core/vpTrackingException.h>

#include <visp3/mbt/vpMbtEdgeKltXmlParser.h>


vpMbGenericTracker::vpMbGenericTracker() :
  m_mapOfCameraTransformationMatrix(), m_mapOfTrackers(), m_referenceCameraName("Camera"),
  m_L(), m_error(), m_w(), m_weightedError()
{
  m_mapOfTrackers["Camera"] = new TrackerWrapper(EDGE_TRACKER);

  //Add default camera transformation matrix
  m_mapOfCameraTransformationMatrix["Camera"] = vpHomogeneousMatrix();
}

vpMbGenericTracker::vpMbGenericTracker(const unsigned int nbCameras, const int trackerType) :
  m_mapOfCameraTransformationMatrix(), m_mapOfTrackers(), m_referenceCameraName("Camera"),
  m_L(), m_error(), m_w(), m_weightedError()
{
  if (nbCameras == 0) {
    throw vpException(vpTrackingException::fatalError, "Cannot use no camera!");
  } else if(nbCameras == 1) {
    m_mapOfTrackers["Camera"] = new TrackerWrapper(trackerType);

    //Add default camera transformation matrix
    m_mapOfCameraTransformationMatrix["Camera"] = vpHomogeneousMatrix();
  } else {
    for(unsigned int i = 1; i <= nbCameras; i++) {
      std::stringstream ss;
      ss << "Camera" << i;
      m_mapOfTrackers[ss.str()] = new TrackerWrapper(trackerType);

      //Add default camera transformation matrix
      m_mapOfCameraTransformationMatrix[ss.str()] = vpHomogeneousMatrix();
    }

    //Set by default the reference camera to the first one
    m_referenceCameraName = m_mapOfTrackers.begin()->first;
  }
}

vpMbGenericTracker::vpMbGenericTracker(const std::vector<int> &trackerTypes) :
  m_mapOfCameraTransformationMatrix(), m_mapOfTrackers(), m_referenceCameraName("Camera"),
  m_L(), m_error(), m_w(), m_weightedError()
{
  if (trackerTypes.empty()) {
    throw vpException(vpException::badValue, "There is no camera!");
  }

  if (trackerTypes.size() == 1) {
    m_mapOfTrackers["Camera"] = new TrackerWrapper(trackerTypes[0]);

    //Add default camera transformation matrix
    m_mapOfCameraTransformationMatrix["Camera"] = vpHomogeneousMatrix();
  } else {
    for(size_t i = 1; i <= trackerTypes.size(); i++) {
      std::stringstream ss;
      ss << "Camera" << i;
      m_mapOfTrackers[ss.str()] = new TrackerWrapper(trackerTypes[i-1]);

      //Add default camera transformation matrix
      m_mapOfCameraTransformationMatrix[ss.str()] = vpHomogeneousMatrix();
    }

    //Set by default the reference camera to the first one
    m_referenceCameraName = m_mapOfTrackers.begin()->first;
  }
}

vpMbGenericTracker::~vpMbGenericTracker() {
  for (std::map<std::string, TrackerWrapper*>::iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    delete it->second;
    it->second = NULL;
  }
}

void vpMbGenericTracker::computeProjectionError() {
  if (computeProjError) {
    double rawTotalProjectionError = 0.0;
    unsigned int nbTotalFeaturesUsed = 0;

    TrackerWrapper *tracker;
    for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
      tracker = it->second;

      double curProjError = tracker->getProjectionError();
      unsigned int nbFeaturesUsed = tracker->nbFeaturesForProjErrorComputation;

      if (nbFeaturesUsed > 0) {
        nbTotalFeaturesUsed += nbFeaturesUsed;
        rawTotalProjectionError += ( vpMath::rad(curProjError)*nbFeaturesUsed );
      }
    }

    if (nbTotalFeaturesUsed > 0) {
      projectionError = vpMath::deg(rawTotalProjectionError / (double)nbTotalFeaturesUsed);
    } else {
      projectionError = 90.0;
    }
  }
}

void vpMbGenericTracker::computeVVS(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages) {
  computeVVSInit(mapOfImages);

  if (m_error.getRows() < 4) {
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "\n\t\t Error-> not enough data");
  }

  double normRes = 0;
  double normRes_1 = -1;
  unsigned int iter = 0;

  vpMatrix LTL;
  vpColVector LTR, v;
  vpColVector error_prev;

  double mu = m_initialMu;
  vpHomogeneousMatrix cMo_prev;

  bool isoJoIdentity_ = true;

  //Covariance
  vpColVector W_true(m_error.getRows());
  vpMatrix L_true, LVJ_true;

  //Create the map of VelocityTwistMatrices
  std::map<std::string, vpVelocityTwistMatrix> mapOfVelocityTwist;
  for(std::map<std::string, vpHomogeneousMatrix>::const_iterator it = m_mapOfCameraTransformationMatrix.begin(); it != m_mapOfCameraTransformationMatrix.end(); ++it) {
    vpVelocityTwistMatrix cVo;
    cVo.buildFrom(it->second);
    mapOfVelocityTwist[it->first] = cVo;
  }

  double factorEdge = 1.0;
  double factorKlt = 1.0;

  while( std::fabs(normRes_1 - normRes) > m_stopCriteriaEpsilon && (iter < m_maxIter) ) {
    computeVVSInteractionMatrixAndResidu(mapOfImages, mapOfVelocityTwist);

    bool reStartFromLastIncrement = false;
    computeVVSCheckLevenbergMarquardt(iter, m_error, error_prev, cMo_prev, mu, reStartFromLastIncrement);
    if (reStartFromLastIncrement) {
      TrackerWrapper *tracker;
      for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
        tracker = it->second;

        tracker->cMo = m_mapOfCameraTransformationMatrix[it->first] * cMo_prev;
        vpHomogeneousMatrix c_curr_tTc_curr0 = m_mapOfCameraTransformationMatrix[it->first] * cMo_prev * tracker->c0Mo.inverse();
        tracker->ctTc0 = c_curr_tTc_curr0;
      }
    }

    if (!reStartFromLastIncrement) {
      computeVVSWeights();

      if (computeCovariance) {
        L_true = m_L;
        if (!isoJoIdentity_) {
          vpVelocityTwistMatrix cVo;
          cVo.buildFrom(cMo);
          LVJ_true = (m_L*cVo*oJo);
        }
      }

      vpVelocityTwistMatrix cVo;
      if (iter == 0) {
        isoJoIdentity_ = true;
        oJo.eye();

        // If all the 6 dof should be estimated, we check if the interaction matrix is full rank.
        // If not we remove automatically the dof that cannot be estimated
        // This is particularly useful when consering circles (rank 5) and cylinders (rank 4)
        if (isoJoIdentity_) {
          cVo.buildFrom(cMo);

          vpMatrix K; // kernel
          unsigned int rank = (m_L*cVo).kernel(K);
          if (rank == 0) {
            throw vpException(vpException::fatalError, "Rank=0, cannot estimate the pose !");
          }

          if (rank != 6) {
            vpMatrix I; // Identity
            I.eye(6);
            oJo = I-K.AtA();

            isoJoIdentity_ = false;
          }
        }
      }

      //Weighting
      double wi;
      double num = 0;
      double den = 0;

      unsigned int start_index = 0;
      TrackerWrapper *tracker;
      for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
        tracker = it->second;

        if (tracker->m_trackerType & EDGE_TRACKER) {
          for (unsigned int i = 0; i < tracker->m_error_edge.getRows(); i++) {
            wi = tracker->m_w_edge[i] * tracker->m_factor[i] * factorEdge;
            W_true[start_index + i] = wi;
            m_weightedError[start_index + i] = wi * m_error[start_index + i];

            num += wi*vpMath::sqr(m_error[start_index + i]);
            den += wi;

            for (unsigned int j = 0; j < m_L.getCols(); j++) {
              m_L[start_index + i][j] *= wi;
            }
          }

          start_index += tracker->m_error_edge.getRows();
        }

        if (tracker->m_trackerType & KLT_TRACKER) {
          for (unsigned int i = 0; i < tracker->m_error_klt.getRows(); i++) {
            wi = tracker->m_w_klt[i] * factorKlt;
            W_true[start_index + i] = wi;
            m_weightedError[start_index + i] = wi * m_error[start_index + i];

            num += wi*vpMath::sqr(m_error[start_index + i]);
            den += wi;

            for (unsigned int j = 0; j < m_L.getCols(); j++) {
              m_L[start_index + i][j] *= wi;
            }
          }

          start_index += tracker->m_error_klt.getRows();
        }
      }

      normRes_1 = normRes;
      normRes = sqrt(num/den);

      computeVVSPoseEstimation(isoJoIdentity_, iter, m_L, LTL, m_weightedError, m_error, error_prev, LTR, mu, v);

      cMo_prev = cMo;

      cMo = vpExponentialMap::direct(v).inverse() * cMo;

      for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
        tracker = it->second;

        vpHomogeneousMatrix c_curr_tTc_curr0 = m_mapOfCameraTransformationMatrix[it->first] * cMo * tracker->c0Mo.inverse();
        tracker->ctTc0 = c_curr_tTc_curr0;
      }

      //Update cMo
      for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
        tracker = it->second;

        tracker->cMo = m_mapOfCameraTransformationMatrix[it->first] * cMo;
      }
    }

    iter++;
  }

  computeCovarianceMatrixVVS(isoJoIdentity_, W_true, cMo_prev, L_true, LVJ_true, m_error);

  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;

    if (tracker->m_trackerType & EDGE_TRACKER) {
      tracker->updateMovingEdgeWeights();
    }
  }
}

void vpMbGenericTracker::computeVVSInit() {
  throw vpException(vpException::fatalError, "vpMbGenericTracker::computeVVSInit() should not be called!");
}

void vpMbGenericTracker::computeVVSInit(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages) {
  unsigned int nbFeatures = 0;

  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->computeVVSInit(*mapOfImages[it->first]);

    nbFeatures += tracker->m_error.getRows();
  }

  m_L.resize(nbFeatures, 6, false);
  m_error.resize(nbFeatures, false);

  m_weightedError.resize(nbFeatures, false);
  m_w.resize(nbFeatures, false);
  m_w = 1;
}

void vpMbGenericTracker::computeVVSInteractionMatrixAndResidu() {
  throw vpException(vpException::fatalError, "vpMbGenericTracker::computeVVSInteractionMatrixAndResidu() should not be called");
}

void vpMbGenericTracker::computeVVSInteractionMatrixAndResidu(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                                             std::map<std::string, vpVelocityTwistMatrix> &mapOfVelocityTwist) {
  unsigned int start_index = 0;

  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;

    tracker->cMo = m_mapOfCameraTransformationMatrix[it->first]*cMo;
    vpHomogeneousMatrix c_curr_tTc_curr0 = m_mapOfCameraTransformationMatrix[it->first] * cMo * tracker->c0Mo.inverse();
    tracker->ctTc0 = c_curr_tTc_curr0;

    tracker->computeVVSInteractionMatrixAndResidu(*mapOfImages[it->first]);

    m_L.insert(tracker->m_L*mapOfVelocityTwist[it->first], start_index, 0);
    m_error.insert(start_index, tracker->m_error);

    start_index += tracker->m_error.getRows();
  }
}

void vpMbGenericTracker::computeVVSWeights() {
  unsigned int start_index = 0;

  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->computeVVSWeights();

    m_w.insert(start_index, tracker->m_w);
    start_index += tracker->m_w.getRows();
  }
}

void vpMbGenericTracker::display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                                const vpColor& col , const unsigned int thickness, const bool displayFullModel) {
  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->display(I, cMo, cam, col, thickness, displayFullModel);
  }
}

void vpMbGenericTracker::display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                                const vpColor& col , const unsigned int thickness, const bool displayFullModel) {
  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->display(I, cMo, cam, col, thickness, displayFullModel);
  }
}

void vpMbGenericTracker::display(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo,
                               const vpCameraParameters &cam1, const vpCameraParameters &cam2, const vpColor &color, const unsigned int thickness, const bool displayFullModel) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    it->second->display(I1, c1Mo, cam1, color, thickness, displayFullModel);
    ++it;

    it->second->display(I2, c2Mo, cam2, color, thickness, displayFullModel);
  } else {
    std::cerr << "The tracker is not set as a stereo configuration! There are "
              << m_mapOfTrackers.size() << " cameras!" << std::endl;
  }
}

void vpMbGenericTracker::display(const vpImage<vpRGBa> &I1, const vpImage<vpRGBa> &I2, const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo,
                               const vpCameraParameters &cam1, const vpCameraParameters &cam2, const vpColor &color, const unsigned int thickness, const bool displayFullModel) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    it->second->display(I1, c1Mo, cam1, color, thickness, displayFullModel);
    ++it;

    it->second->display(I2, c2Mo, cam2, color, thickness, displayFullModel);
  } else {
    std::cerr << "The tracker is not set as a stereo configuration! There are "
              << m_mapOfTrackers.size() << " cameras!" << std::endl;
  }
}

void vpMbGenericTracker::display(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                               const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
                               const std::map<std::string, vpCameraParameters> &mapOfCameraParameters,
                               const vpColor& col, const unsigned int thickness, const bool displayFullModel) {
  TrackerWrapper *tracker;

  //Display only for the given images
  for (std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img = mapOfImages.begin(); it_img != mapOfImages.end(); ++it_img) {
    std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(it_img->first);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(it_img->first);
    std::map<std::string, vpCameraParameters>::const_iterator it_cam = mapOfCameraParameters.find(it_img->first);

    if (it_tracker != m_mapOfTrackers.end() && it_camPose != mapOfCameraPoses.end() && it_cam != mapOfCameraParameters.end()) {
      tracker = it_tracker->second;
      tracker->display(*it_img->second, it_camPose->second, it_cam->second, col, thickness, displayFullModel);
    } else {
      std::cerr << "Missing elements! " << std::endl;
    }
  }
}

void vpMbGenericTracker::display(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfImages,
                               const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
                               const std::map<std::string, vpCameraParameters> &mapOfCameraParameters,
                               const vpColor& col, const unsigned int thickness, const bool displayFullModel) {
  TrackerWrapper *tracker;

  //Display only for the given images
  for (std::map<std::string, const vpImage<vpRGBa> *>::const_iterator it_img = mapOfImages.begin(); it_img != mapOfImages.end(); ++it_img) {
    std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(it_img->first);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(it_img->first);
    std::map<std::string, vpCameraParameters>::const_iterator it_cam = mapOfCameraParameters.find(it_img->first);

    if (it_tracker != m_mapOfTrackers.end() && it_camPose != mapOfCameraPoses.end() && it_cam != mapOfCameraParameters.end()) {
      tracker = it_tracker->second;
      tracker->display(*it_img->second, it_camPose->second, it_cam->second, col, thickness, displayFullModel);
    } else {
      std::cerr << "Missing elements! " << std::endl;
    }
  }
}

std::vector<std::string> vpMbGenericTracker::getCameraNames() const {
  std::vector<std::string> cameraNames;

  for (std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    cameraNames.push_back(it_tracker->first);
  }

  return cameraNames;
}

void vpMbGenericTracker::getCameraParameters(vpCameraParameters &cam1, vpCameraParameters &cam2) const {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    it->second->getCameraParameters(cam1);
    ++it;

    it->second->getCameraParameters(cam2);
  } else {
    std::cerr << "The tracker is not set as a stereo configuration! There are "
              << m_mapOfTrackers.size() << " cameras!" << std::endl;
  }
}

void vpMbGenericTracker::getCameraParameters(std::map<std::string, vpCameraParameters> &mapOfCameraParameters) const {
  //Clear the input map
  mapOfCameraParameters.clear();

  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    vpCameraParameters cam_;
    it->second->getCameraParameters(cam_);
    mapOfCameraParameters[it->first] = cam_;
  }
}

std::map<std::string, int> vpMbGenericTracker::getCameraTrackerTypes() const {
  std::map<std::string, int> trackingTypes;

  TrackerWrapper *traker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    traker = it_tracker->second;
    trackingTypes[it_tracker->first] = traker->getTrackerType();
  }

  return trackingTypes;
}

std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > vpMbGenericTracker::getPolygonFaces(const bool orderPolygons, const bool useVisibility, const bool clipPolygon) {
  std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > polygonFaces;

  TrackerWrapper *tracker;
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    tracker = it->second;
    polygonFaces = tracker->getPolygonFaces(orderPolygons, useVisibility, clipPolygon);
  } else {
    std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
  }

  return polygonFaces;
}

void vpMbGenericTracker::getPose(vpHomogeneousMatrix &c1Mo, vpHomogeneousMatrix &c2Mo) const {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    it->second->getPose(c1Mo);
    ++it;

    it->second->getPose(c2Mo);
  } else {
    std::cerr << "The tracker is not set as a stereo configuration! There are "
              << m_mapOfTrackers.size() << " cameras!" << std::endl;
  }
}

void vpMbGenericTracker::getPose(std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) const {
  //Clear the map
  mapOfCameraPoses.clear();

  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->getPose(mapOfCameraPoses[it->first]);
  }
}

void vpMbGenericTracker::init(const vpImage<unsigned char>& I) {
  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->cMo = m_mapOfCameraTransformationMatrix[it->first] * cMo;
    tracker->init(I);
  }
}

void vpMbGenericTracker::initCircle(const vpPoint& /*p1*/, const vpPoint &/*p2*/, const vpPoint &/*p3*/, const double /*radius*/, const int /*idFace*/, const std::string &/*name*/) {
  throw vpException(vpException::fatalError, "vpMbGenericTracker::initCircle() should not be called!");
}

#ifdef VISP_HAVE_MODULE_GUI
void vpMbGenericTracker::initClick(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                                 const std::string &initFile1, const std::string &initFile2, const bool displayHelp) {
  if (m_mapOfTrackers.size() == 2) {
    TrackerWrapper *tracker;
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    tracker = it->second;
    tracker->initClick(I1, initFile1, displayHelp);

    ++it;

    tracker = it->second;
    tracker->initClick(I2, initFile2, displayHelp);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      tracker = it->second;

      //Set the reference cMo
      tracker->getPose(cMo);
    }
  } else {
    std::stringstream ss;
    ss << "Cannot initClick()! Require two cameras but there are " << m_mapOfTrackers.size() << " cameras!";
    throw vpException(vpTrackingException::initializationError, ss.str());
  }
}

void vpMbGenericTracker::initClick(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                 const std::map<std::string, std::string> &mapOfInitFiles, const bool displayHelp) {
  std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char>* >::const_iterator it_img = mapOfImages.find(m_referenceCameraName);
  std::map<std::string, std::string>::const_iterator it_initFile = mapOfInitFiles.find(m_referenceCameraName);

  TrackerWrapper *tracker;
  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfImages.end() && it_initFile != mapOfInitFiles.end()) {
    tracker = it_tracker->second;
    tracker->initClick(*it_img->second, it_initFile->second, displayHelp);
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot initClick for the reference camera!");
  }

  //Vector of missing initFile for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  //Set pose for the specified cameras
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    if (it_tracker->first != m_referenceCameraName) {
      it_img = mapOfImages.find(it_tracker->first);
      it_initFile = mapOfInitFiles.find(it_tracker->first);

      if (it_img != mapOfImages.end() && it_initFile != mapOfInitFiles.end()) {
        //InitClick for the current camera
        tracker = it_tracker->second;
        tracker->initClick(*it_img->second, it_initFile->second, displayHelp);
      } else {
        vectorOfMissingCameraPoses.push_back(it_tracker->first);
      }
    }
  }

  //Set pose for cameras that do not have an initFile
  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameraPoses.begin(); it != vectorOfMissingCameraPoses.end(); ++it) {
    it_img = mapOfImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->cMo = cCurrentMo;
      m_mapOfTrackers[*it]->init(*it_img->second);
    } else {
      std::stringstream ss;
      ss << "Missing image or missing camera transformation matrix! Cannot set the pose for camera: " << (*it) << "!";
      throw vpException(vpTrackingException::initializationError, ss.str().c_str());
    }
  }
}
#endif

void vpMbGenericTracker::initCylinder(const vpPoint& /*p1*/, const vpPoint &/*p2*/, const double /*radius*/, const int /*idFace*/, const std::string &/*name*/) {
  throw vpException(vpException::fatalError, "vpMbGenericTracker::initCylinder() should not be called!");
}

void vpMbGenericTracker::initFaceFromCorners(vpMbtPolygon &/*polygon*/) {
  throw vpException(vpException::fatalError, "vpMbGenericTracker::initFaceFromCorners() should not be called!");
}

void vpMbGenericTracker::initFaceFromLines(vpMbtPolygon &/*polygon*/) {
  throw vpException(vpException::fatalError, "vpMbGenericTracker::initFaceFromLines() should not be called!");
}

void vpMbGenericTracker::initFromPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, const std::string &initFile1, const std::string &initFile2) {
  if (m_mapOfTrackers.size() == 2) {
    TrackerWrapper *tracker;
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    tracker = it->second;
    tracker->initFromPose(I1, initFile1);

    ++it;

    tracker = it->second;
    tracker->initFromPose(I2, initFile2);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      tracker = it->second;

      //Set the reference cMo
      tracker->getPose(cMo);

      //Set the reference camera parameters
      tracker->getCameraParameters(cam);
    }
  } else {
    std::stringstream ss;
    ss << "Cannot initFromPose()! Require two cameras but there are " << m_mapOfTrackers.size() << " cameras!";
    throw vpException(vpTrackingException::initializationError, ss.str());
  }
}

void vpMbGenericTracker::initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages, const std::map<std::string, std::string> &mapOfInitPoses) {
  //Set the reference cMo
  std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char>* >::const_iterator it_img = mapOfImages.find(m_referenceCameraName);
  std::map<std::string, std::string>::const_iterator it_initPose = mapOfInitPoses.find(m_referenceCameraName);

  TrackerWrapper *tracker;
  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfImages.end() && it_initPose != mapOfInitPoses.end()) {
    tracker = it_tracker->second;
    tracker->initFromPose(*it_img->second, it_initPose->second);
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot initFromPose() for the reference camera!");
  }

  //Vector of missing pose matrices for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  //Set pose for the specified cameras
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    it_img = mapOfImages.find(it_tracker->first);
    it_initPose = mapOfInitPoses.find(it_tracker->first);

    if (it_img != mapOfImages.end() && it_initPose != mapOfInitPoses.end()) {
      //Set pose
      it_tracker->second->initFromPose(*it_img->second, it_initPose->second);
    } else {
      vectorOfMissingCameraPoses.push_back(it_tracker->first);
    }
  }

  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameraPoses.begin(); it != vectorOfMissingCameraPoses.end(); ++it) {
    it_img = mapOfImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->initFromPose(*it_img->second, cCurrentMo);
    } else {
      std::stringstream ss;
      ss << "Missing image or missing camera transformation matrix! Cannot init the pose for camera: " << (*it) << "!";
      throw vpException(vpTrackingException::initializationError, ss.str().c_str());
    }
  }
}

void vpMbGenericTracker::initFromPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    it->second->initFromPose(I1, c1Mo);

    ++it;

    it->second->initFromPose(I2, c2Mo);

    this->cMo = c1Mo;
  } else {
    std::stringstream ss;
    ss << "This method requires 2 cameras but there are " << m_mapOfTrackers.size() << " cameras!";
    throw vpException(vpTrackingException::initializationError, ss.str());
  }
}

void vpMbGenericTracker::initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages, const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) {
  //Set the reference cMo
  std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char>* >::const_iterator it_img = mapOfImages.find(m_referenceCameraName);
  std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(m_referenceCameraName);

  TrackerWrapper *tracker;
  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
    tracker = it_tracker->second;
    tracker->initFromPose(*it_img->second, it_camPose->second);
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot set pose for the reference camera!");
  }

  //Vector of missing pose matrices for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  //Set pose for the specified cameras
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    it_img = mapOfImages.find(it_tracker->first);
    it_camPose = mapOfCameraPoses.find(it_tracker->first);

    if (it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
      //Set pose
      it_tracker->second->initFromPose(*it_img->second, it_camPose->second);
    } else {
      vectorOfMissingCameraPoses.push_back(it_tracker->first);
    }
  }

  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameraPoses.begin(); it != vectorOfMissingCameraPoses.end(); ++it) {
    it_img = mapOfImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->initFromPose(*it_img->second, cCurrentMo);
    } else {
      std::stringstream ss;
      ss << "Missing image or missing camera transformation matrix! Cannot set the pose for camera: " << (*it) << "!";
      throw vpException(vpTrackingException::initializationError, ss.str().c_str());
    }
  }
}

void vpMbGenericTracker::loadConfigFile(const std::string& configFile) {
  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->loadConfigFile(configFile);
  }

  if (m_mapOfTrackers.find(m_referenceCameraName) == m_mapOfTrackers.end()) {
    throw vpException(vpException::fatalError, std::string("Reference camera: " + m_referenceCameraName + " does not exist!"));
  }

  m_mapOfTrackers[m_referenceCameraName]->getCameraParameters(this->cam);
  this->angleAppears = m_mapOfTrackers[m_referenceCameraName]->getAngleAppear();
  this->angleDisappears = m_mapOfTrackers[m_referenceCameraName]->getAngleDisappear();
  this->clippingFlag = m_mapOfTrackers[m_referenceCameraName]->getClipping();
}

void vpMbGenericTracker::loadConfigFile(const std::string& configFile1, const std::string& configFile2) {
  if (m_mapOfTrackers.size() != 2) {
    throw vpException(vpException::fatalError, "The tracker is not set in a stereo configuration!");
  }

  std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.begin();
  TrackerWrapper *tracker = it_tracker->second;
  tracker->loadConfigFile(configFile1);

  ++it_tracker;
  tracker = it_tracker->second;
  tracker->loadConfigFile(configFile2);

  if (m_mapOfTrackers.find(m_referenceCameraName) == m_mapOfTrackers.end()) {
    throw vpException(vpException::fatalError, std::string("Reference camera: " + m_referenceCameraName + " does not exist!"));
  }

  m_mapOfTrackers[m_referenceCameraName]->getCameraParameters(this->cam);
  this->angleAppears = m_mapOfTrackers[m_referenceCameraName]->getAngleAppear();
  this->angleDisappears = m_mapOfTrackers[m_referenceCameraName]->getAngleDisappear();
  this->clippingFlag = m_mapOfTrackers[m_referenceCameraName]->getClipping();
}

void vpMbGenericTracker::loadConfigFile(const std::map<std::string, std::string> &mapOfConfigFiles) {
  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    tracker = it_tracker->second;

    std::map<std::string, std::string>::const_iterator it_config = mapOfConfigFiles.find(it_tracker->first);
    if (it_config != mapOfConfigFiles.end()) {
      tracker->loadConfigFile(it_config->second);
    } else {
      std::stringstream ss;
      ss << "Missing configuration file for camera: " << it_tracker->first << "!";
      throw vpException(vpTrackingException::initializationError, ss.str().c_str());
    }
  }

  //Set the reference camera parameters
  std::map<std::string, TrackerWrapper*>::iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    tracker = it->second;
    tracker->getCameraParameters(cam);

    //Set clipping
    this->clippingFlag = tracker->getClipping();
    this->angleAppears = tracker->getAngleAppear();
    this->angleDisappears = tracker->getAngleDisappear();
  } else {
    std::stringstream ss;
    ss << "The reference camera: " << m_referenceCameraName << " does not exist!";
    throw vpException(vpTrackingException::initializationError, ss.str().c_str());
  }
}

void vpMbGenericTracker::loadModel(const std::string &modelFile, const bool verbose) {
  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->loadModel(modelFile, verbose);
  }
}

void vpMbGenericTracker::loadModel(const std::string &modelFile1, const std::string &modelFile2, const bool verbose) {
  if (m_mapOfTrackers.size() != 2) {
    throw vpException(vpException::fatalError, "The tracker is not set in a stereo configuration!");
  }

  std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.begin();
  TrackerWrapper *tracker = it_tracker->second;
  tracker->loadModel(modelFile1, verbose);

  ++it_tracker;
  tracker = it_tracker->second;
  tracker->loadModel(modelFile2, verbose);
}

void vpMbGenericTracker::loadModel(const std::map<std::string, std::string> &mapOfModelFiles, const bool verbose) {
  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    std::map<std::string, std::string>::const_iterator it_model = mapOfModelFiles.find(it_tracker->first);

    if (it_model != mapOfModelFiles.end()) {
      tracker = it_tracker->second;
      tracker->loadModel(it_model->second, verbose);
    } else {
      throw vpException(vpTrackingException::initializationError, std::string("Cannot load model for camera: " + it_tracker->first));
    }
  }
}

void vpMbGenericTracker::preTracking(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages) {
  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;

    tracker->preTracking(*mapOfImages[it->first]);
  }
}

void vpMbGenericTracker::reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name, const vpHomogeneousMatrix &cMo_, const bool verbose) {
  if (m_mapOfTrackers.size() != 1) {
    std::stringstream ss;
    ss << "This method requires exactly one camera, there are " << m_mapOfTrackers.size() << " cameras!";
    throw vpException(vpTrackingException::fatalError, ss.str().c_str());
  }

  TrackerWrapper *tracker;
  std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  if (it_tracker != m_mapOfTrackers.end()) {
    tracker = it_tracker->second;
    tracker->reInitModel(I, cad_name, cMo_, verbose);

    //Set reference pose
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::fatalError, "Cannot reInitModel() the reference camera!");
  }

  modelInitialised = true;
}

void vpMbGenericTracker::reInitModel(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, const std::string &cad_name,
                                   const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo, const bool verbose) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.begin();

    it_tracker->second->reInitModel(I1, cad_name, c1Mo, verbose);

    ++it_tracker;

    it_tracker->second->reInitModel(I2, cad_name, c2Mo, verbose);

    it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
    if (it_tracker != m_mapOfTrackers.end()) {
      //Set reference pose
      it_tracker->second->getPose(cMo);
    }
  } else {
    throw vpException(vpTrackingException::fatalError, "This method requires exactly two cameras!");
  }
}

void vpMbGenericTracker::reInitModel(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages, const std::string &cad_name,
                                   const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses, const bool verbose) {
  std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img = mapOfImages.find(m_referenceCameraName);
  std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(m_referenceCameraName);

  TrackerWrapper *tracker;
  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
    tracker = it_tracker->second;
    tracker->reInitModel(*it_img->second, cad_name, it_camPose->second, verbose);
    modelInitialised = true;

    //Set reference pose
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::fatalError, "Cannot reInitModel() for reference camera!");
  }

  std::vector<std::string> vectorOfMissingCameras;
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    if (it_tracker->first != m_referenceCameraName) {
      it_img = mapOfImages.find(it_tracker->first);
      it_camPose = mapOfCameraPoses.find(it_tracker->first);

      if (it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
        tracker = it_tracker->second;
        tracker->reInitModel(*it_img->second, cad_name, it_camPose->second, verbose);
      } else {
        vectorOfMissingCameras.push_back(it_tracker->first);
      }
    }
  }

  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameras.begin(); it != vectorOfMissingCameras.end(); ++it) {
    it_img = mapOfImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->reInitModel(*it_img->second, cad_name, cCurrentMo, verbose);
    }
  }
}

void vpMbGenericTracker::resetTracker() {
  cMo.eye();

#ifdef VISP_HAVE_OGRE
  useOgre = false;
#endif

  m_computeInteraction = true;
  m_lambda = 1.0;

  angleAppears = vpMath::rad(89);
  angleDisappears = vpMath::rad(89);
  clippingFlag = vpPolygon3D::NO_CLIPPING;

  m_optimizationMethod = vpMbTracker::GAUSS_NEWTON_OPT;

  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->resetTracker();
  }
}

void vpMbGenericTracker::setAngleAppear(const double &a) {
  vpMbTracker::setAngleAppear(a);

  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->setAngleAppear(a);
  }
}

void vpMbGenericTracker::setAngleDisappear(const double &a) {
  vpMbTracker::setAngleDisappear(a);

  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->setAngleDisappear(a);
  }
}

void vpMbGenericTracker::setCameraParameters(const vpCameraParameters &camera) {
  vpMbTracker::setCameraParameters(camera);

  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->setCameraParameters(camera);
  }
}

void vpMbGenericTracker::setCameraParameters(const vpCameraParameters &camera1, const vpCameraParameters &camera2) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    it->second->setCameraParameters(camera1);

    ++it;
    it->second->setCameraParameters(camera2);

    this->cam = camera1;
  } else {
    std::stringstream ss;
    ss << "Require two cameras! There are " << m_mapOfTrackers.size() << " cameras!";
    throw vpException(vpTrackingException::fatalError, ss.str());
  }
}

void vpMbGenericTracker::setCameraParameters(const std::map<std::string, vpCameraParameters> &mapOfCameraParameters) {
  TrackerWrapper *tracker;
  //Set parameters only for the given cameras
  for (std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    std::map<std::string, vpCameraParameters>::const_iterator it_cam = mapOfCameraParameters.find(it_tracker->first);

    if (it_cam != mapOfCameraParameters.end()) {
      tracker = it_tracker->second;
      tracker->setCameraParameters(it_cam->second);

      if (it_tracker->first == m_referenceCameraName) {
        this->cam = it_cam->second;
      }
    }
  }
}

void vpMbGenericTracker::setCameraTransformationMatrix(const std::string &cameraName, const vpHomogeneousMatrix &cameraTransformationMatrix) {
  std::map<std::string, vpHomogeneousMatrix>::iterator it = m_mapOfCameraTransformationMatrix.find(cameraName);

  if (it != m_mapOfCameraTransformationMatrix.end()) {
    it->second = cameraTransformationMatrix;
  } else {
    throw vpException(vpTrackingException::fatalError, std::string("Cannot find camera: " + cameraName + "!"));
  }
}

void vpMbGenericTracker::setCameraTransformationMatrix(const std::map<std::string, vpHomogeneousMatrix> &mapOfTransformationMatrix) {
  //Check if all the cameras have a transformation matrix
  for (std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = mapOfTransformationMatrix.find(it_tracker->first);

    if (it_camTrans == mapOfTransformationMatrix.end()) {
      throw vpException(vpTrackingException::initializationError, std::string("Missing transformation matrix for camera: " + it_tracker->first));
    }
  }

  m_mapOfCameraTransformationMatrix = mapOfTransformationMatrix;
}

void vpMbGenericTracker::setClipping(const unsigned int &flags) {
  //Set clipping for vpMbEdgeMultiTracker class
  vpMbTracker::setClipping(flags);

  TrackerWrapper *tracker;
  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->setClipping(flags);
  }
}

void vpMbGenericTracker::setDisplayFeatures(const bool displayF) {
  vpMbTracker::setDisplayFeatures(displayF);

  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->setDisplayFeatures(displayF);
  }
}

void vpMbGenericTracker::setFarClippingDistance(const double &dist) {
  vpMbTracker::setFarClippingDistance(dist);

  TrackerWrapper *tracker;
  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->setFarClippingDistance(dist);
  }
}

void vpMbGenericTracker::setKltOpencv(const vpKltOpencv &t) {
  TrackerWrapper *tracker;
  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->setKltOpencv(t);
  }
}

void vpMbGenericTracker::setMaskBorder(const unsigned int &e) {
  TrackerWrapper *tracker;
  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->setMaskBorder(e);
  }
}

void vpMbGenericTracker::setMovingEdge(const vpMe &me) {
  TrackerWrapper *tracker;
  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->setMovingEdge(me);
  }
}

void vpMbGenericTracker::setNearClippingDistance(const double &dist) {
  vpMbTracker::setNearClippingDistance(dist);

  TrackerWrapper *tracker;
  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->setNearClippingDistance(dist);
  }
}

void vpMbGenericTracker::setOgreShowConfigDialog(const bool showConfigDialog) {
  vpMbTracker::setOgreShowConfigDialog(showConfigDialog);

  TrackerWrapper *tracker;
  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->setOgreShowConfigDialog(showConfigDialog);
  }
}

void vpMbGenericTracker::setOgreVisibilityTest(const bool &v) {
  vpMbTracker::setOgreVisibilityTest(v);

  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->setOgreVisibilityTest(v);
  }

#ifdef VISP_HAVE_OGRE
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->faces.getOgreContext()->setWindowName("Multi Generic MBT (" + it->first + ")");
  }
#endif
}

void vpMbGenericTracker::setOptimizationMethod(const vpMbtOptimizationMethod &opt) {
  vpMbTracker::setOptimizationMethod(opt);

  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->setOptimizationMethod(opt);
  }
}

void vpMbGenericTracker::setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix& cdMo) {
  cMo = cdMo;

  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->setPose(I, cdMo);
  }
}

void vpMbGenericTracker::setPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix c2Mo) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    it->second->setPose(I1, c1Mo);

    ++it;

    it->second->setPose(I2, c2Mo);

    this->cMo = c1Mo;
  } else {
    std::stringstream ss;
    ss << "This method requires 2 cameras but there are " << m_mapOfTrackers.size() << " cameras!";
    throw vpException(vpTrackingException::fatalError, ss.str());
  }
}

void vpMbGenericTracker::setPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages, const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) {
  //Set the reference cMo
  std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char>* >::const_iterator it_img = mapOfImages.find(m_referenceCameraName);
  std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(m_referenceCameraName);

  TrackerWrapper *tracker;
  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
    tracker = it_tracker->second;
    tracker->setPose(*it_img->second, it_camPose->second);
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::fatalError, "Cannot set pose for the reference camera!");
  }

  //Vector of missing pose matrices for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  //Set pose for the specified cameras
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    if (it_tracker->first != m_referenceCameraName) {
      it_img = mapOfImages.find(it_tracker->first);
      it_camPose = mapOfCameraPoses.find(it_tracker->first);

      if (it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
        //Set pose
        tracker = it_tracker->second;
        tracker->setPose(*it_img->second, it_camPose->second);
      } else {
        vectorOfMissingCameraPoses.push_back(it_tracker->first);
      }
    }
  }

  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameraPoses.begin(); it != vectorOfMissingCameraPoses.end(); ++it) {
    it_img = mapOfImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->setPose(*it_img->second, cCurrentMo);
    } else {
      std::stringstream ss;
      ss << "Missing image or missing camera transformation matrix! Cannot set pose for camera: " << (*it) << "!";
      throw vpException(vpTrackingException::fatalError, ss.str());
    }
  }
}

void vpMbGenericTracker::setProjectionErrorComputation(const bool &flag) {
  vpMbTracker::setProjectionErrorComputation(flag);

  TrackerWrapper *tracker;
  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->setProjectionErrorComputation(flag);
  }
}

void vpMbGenericTracker::setReferenceCameraName(const std::string &referenceCameraName) {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    m_referenceCameraName = referenceCameraName;
  } else {
    std::cerr << "The reference camera: " << referenceCameraName << " does not exist!";
  }
}

void vpMbGenericTracker::setScanLineVisibilityTest(const bool &v) {
  vpMbTracker::setScanLineVisibilityTest(v);

  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->setScanLineVisibilityTest(v);
  }
}

void vpMbGenericTracker::setTrackerType(const int type) {
  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;
    tracker->setTrackerType(type);
  }
}

void vpMbGenericTracker::testTracking() {

}

void vpMbGenericTracker::track(const vpImage<unsigned char> &I) {
  std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
  mapOfImages[m_referenceCameraName] = &I;

  track(mapOfImages);
}

void vpMbGenericTracker::track(const vpImage<unsigned char>& I1, const vpImage<unsigned char>& I2) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
    mapOfImages[it->first] = &I1;
    ++it;

    mapOfImages[it->first] = &I2;
    track(mapOfImages);
  } else {
    std::stringstream ss;
    ss << "Require two cameras! There are " << m_mapOfTrackers.size() << " cameras!";
    throw vpException(vpTrackingException::fatalError, ss.str().c_str());
  }
}

void vpMbGenericTracker::track(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages) {
  TrackerWrapper *tracker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;

    if ( (tracker->m_trackerType & (EDGE_TRACKER | KLT_TRACKER)) == 0 ) {
      std::cerr << "Bad tracker type: " << tracker->m_trackerType << std::endl;
      return;
    }

    if (tracker->m_trackerType & (EDGE_TRACKER | KLT_TRACKER) && mapOfImages[it->first] == NULL) {
      throw vpException(vpException::fatalError, "Image pointer is NULL!");
    }
  }

  preTracking(mapOfImages);

  try {
    computeVVS(mapOfImages);
  } catch (...) {
    covarianceMatrix = -1;
    throw; // throw the original exception
  }

  //TODO: testTracking somewhere/needed?

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    tracker = it->second;

    tracker->postTracking(*mapOfImages[it->first]);
  }

  computeProjectionError();
}


/** TrackerWrapper **/
vpMbGenericTracker::TrackerWrapper::TrackerWrapper() : m_trackerType(EDGE_TRACKER),
  m_L(), m_error(), m_w(), m_weightedError()
{
  m_lambda = 1.0;
  m_maxIter = 30;

#ifdef VISP_HAVE_OGRE
  faces.getOgreContext()->setWindowName("MBT TrackerWrapper");
#endif
}

vpMbGenericTracker::TrackerWrapper::TrackerWrapper(const int trackerType) : m_trackerType(trackerType),
  m_L(), m_error(), m_w(), m_weightedError()
{
  m_lambda = 1.0;
  m_maxIter = 30;

#ifdef VISP_HAVE_OGRE
  faces.getOgreContext()->setWindowName("MBT TrackerWrapper");
#endif
}

vpMbGenericTracker::TrackerWrapper::~TrackerWrapper() { }

void vpMbGenericTracker::TrackerWrapper::computeVVS(const vpImage<unsigned char> &I) {
  computeVVSInit(I);

  if (m_error.getRows() < 4) {
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "\n\t\t Error-> not enough data");
  }

  double normRes = 0;
  double normRes_1 = -1;
  unsigned int iter = 0;

  double factorEdge = 1.0;
  double factorKlt = 1.0;

  vpMatrix LTL;
  vpColVector LTR, v;
  vpColVector error_prev;

  double mu = m_initialMu;
  vpHomogeneousMatrix cMo_prev;
  vpHomogeneousMatrix ctTc0_Prev; //Only for KLT

  bool isoJoIdentity_ = true;

  //Covariance
  vpColVector W_true(m_error.getRows());
  vpMatrix L_true, LVJ_true;

  unsigned int nb_edge_features = m_error_edge.getRows();
  unsigned int nb_klt_features = m_error_klt.getRows();

  if (nb_edge_features < 4 && nb_klt_features < 4) {
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "\n\t\t Error-> not enough data");
  } else if(nb_edge_features < 4) {
    nb_edge_features = 0;
  }

  while( std::fabs(normRes_1 - normRes) > m_stopCriteriaEpsilon && (iter < m_maxIter) ) {
    computeVVSInteractionMatrixAndResidu(I);

    bool reStartFromLastIncrement = false;
    computeVVSCheckLevenbergMarquardt(iter, m_error, error_prev, cMo_prev, mu, reStartFromLastIncrement);
    if (reStartFromLastIncrement) {
      if (m_trackerType & KLT_TRACKER) {
        ctTc0 = ctTc0_Prev;
      }
    }

    if (!reStartFromLastIncrement) {
      computeVVSWeights();

      if (computeCovariance) {
        L_true = m_L;
        if (!isoJoIdentity_) {
          vpVelocityTwistMatrix cVo;
          cVo.buildFrom(cMo);
          LVJ_true = (m_L*cVo*oJo);
        }
      }

      vpVelocityTwistMatrix cVo;
      if (iter == 0) {
        isoJoIdentity_ = true;
        oJo.eye();

        // If all the 6 dof should be estimated, we check if the interaction matrix is full rank.
        // If not we remove automatically the dof that cannot be estimated
        // This is particularly useful when consering circles (rank 5) and cylinders (rank 4)
        if (isoJoIdentity_) {
          cVo.buildFrom(cMo);

          vpMatrix K; // kernel
          unsigned int rank = (m_L*cVo).kernel(K);
          if (rank == 0) {
            throw vpException(vpException::fatalError, "Rank=0, cannot estimate the pose !");
          }

          if (rank != 6) {
            vpMatrix I; // Identity
            I.eye(6);
            oJo = I-K.AtA();

            isoJoIdentity_ = false;
          }
        }
      }

      //Weighting
      double wi;
      double num = 0;
      double den = 0;

      unsigned int start_index = 0;
      if (m_trackerType & EDGE_TRACKER) {
        for (unsigned int i = 0; i < nb_edge_features; i++) {
          wi = m_w_edge[i] * m_factor[i] * factorEdge;
          W_true[i] = wi;
          m_weightedError[i] = wi*m_error[i];

          num += wi*vpMath::sqr(m_error[i]);
          den += wi;

          for (unsigned int j = 0; j < m_L.getCols(); j++) {
            m_L[i][j] *= wi;
          }
        }

        start_index += nb_edge_features;
      }

      if (m_trackerType & KLT_TRACKER) {
        for (unsigned int i = 0; i < nb_klt_features; i++) {
          wi = m_w_klt[i] * factorKlt;
          W_true[start_index + i] = wi;
          m_weightedError[start_index + i] = wi * m_error_klt[i];

          num += wi*vpMath::sqr(m_error[start_index + i]);
          den += wi;

          for (unsigned int j = 0; j < m_L.getCols(); j++) {
            m_L[start_index + i][j] *= wi;
          }
        }

        start_index += nb_klt_features;
      }

      computeVVSPoseEstimation(isoJoIdentity_, iter, m_L, LTL, m_weightedError, m_error, error_prev, LTR, mu, v);

      cMo_prev = cMo;
      if (m_trackerType & KLT_TRACKER) {
        ctTc0_Prev = ctTc0;
      }

      cMo = vpExponentialMap::direct(v).inverse() * cMo;

      if (m_trackerType & KLT_TRACKER) {
        ctTc0 = vpExponentialMap::direct(v).inverse() * ctTc0;
      }

      normRes_1 = normRes;

      normRes = sqrt(num/den);
    }

    iter++;
  }

  computeCovarianceMatrixVVS(isoJoIdentity_, W_true, cMo_prev, L_true, LVJ_true, m_error);

  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::updateMovingEdgeWeights();
  }
}

void vpMbGenericTracker::TrackerWrapper::computeVVSInit() {
  throw vpException(vpException::fatalError, "vpMbGenericTracker::TrackerWrapper::computeVVSInit() should not be called!");
}

void vpMbGenericTracker::TrackerWrapper::computeVVSInit(const vpImage<unsigned char> &I) {
  initMbtTracking(I);

  unsigned int nbFeatures = 0;

  if (m_trackerType & EDGE_TRACKER) {
    nbFeatures += m_error_edge.getRows();
  } else {
    m_error_edge.clear();
    m_weightedError_edge.clear();
    m_L_edge.clear();
    m_w_edge.clear();
  }

  if (m_trackerType & KLT_TRACKER) {
    vpMbKltTracker::computeVVSInit();
    nbFeatures += m_error_klt.getRows();
  } else {
    m_error_klt.clear();
    m_weightedError_klt.clear();
    m_L_klt.clear();
    m_w_klt.clear();
  }

  m_L.resize(nbFeatures, 6, false);
  m_error.resize(nbFeatures, false);

  m_weightedError.resize(nbFeatures, false);
  m_w.resize(nbFeatures, false);
  m_w = 1;
}

void vpMbGenericTracker::TrackerWrapper::computeVVSInteractionMatrixAndResidu() {
  throw vpException(vpException::fatalError, "vpMbGenericTracker::TrackerWrapper::computeVVSInteractionMatrixAndResidu() should not be called!");
}

void vpMbGenericTracker::TrackerWrapper::computeVVSInteractionMatrixAndResidu(const vpImage<unsigned char> &I) {
  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::computeVVSInteractionMatrixAndResidu(I);
  }

  if (m_trackerType & KLT_TRACKER) {
    vpMbKltTracker::computeVVSInteractionMatrixAndResidu();
  }

  unsigned int start_index = 0;
  if (m_trackerType & EDGE_TRACKER) {
    m_L.insert(m_L_edge, start_index, 0);
    m_error.insert(start_index, m_error_edge);

    start_index += m_error_edge.getRows();
  }

  if (m_trackerType & KLT_TRACKER) {
    m_L.insert(m_L_klt, start_index, 0);
    m_error.insert(start_index, m_error_klt);

    start_index += m_error_klt.getRows();
  }
}

void vpMbGenericTracker::TrackerWrapper::computeVVSWeights() {
  unsigned int start_index = 0;

  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::computeVVSWeights();
    m_w.insert(start_index, m_w_edge);

    start_index += m_w_edge.getRows();
  }

  if (m_trackerType & KLT_TRACKER) {
    vpMbTracker::computeVVSWeights(m_robust_klt, m_error_klt, m_w_klt);
    m_w.insert(start_index, m_w_klt);

    start_index += m_w_klt.getRows();
  }
}

void vpMbGenericTracker::TrackerWrapper::display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo_, const vpCameraParameters &camera,
                             const vpColor& col , const unsigned int thickness, const bool displayFullModel) {
  if ( m_trackerType == EDGE_TRACKER ) {
    vpMbEdgeTracker::display(I, cMo_, camera, col, thickness, displayFullModel);
  } else if ( m_trackerType == KLT_TRACKER) {
    vpMbKltTracker::display(I, cMo_, camera, col, thickness, displayFullModel);
  } else {
    if (m_trackerType & EDGE_TRACKER) {
      for (unsigned int i = 0; i < scales.size(); i += 1){
        if(scales[i]){
          for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
            (*it)->display(I,cMo_, camera, col, thickness, displayFullModel);
          }

          for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
            (*it)->display(I, cMo_, camera, col, thickness, displayFullModel);
          }

          for(std::list<vpMbtDistanceCircle*>::const_iterator it=circles[scaleLevel].begin(); it!=circles[scaleLevel].end(); ++it){
            (*it)->display(I, cMo_, camera, col, thickness, displayFullModel);
          }

          break; //displaying model on one scale only
        }
      }
    }

    if (m_trackerType & KLT_TRACKER) {
      vpMbtDistanceKltPoints *kltpoly;
      for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
        kltpoly = *it;
        if(displayFeatures && kltpoly->hasEnoughPoints() && kltpoly->isTracked() && kltpoly->polygon->isVisible()) {
            kltpoly->displayPrimitive(I);
        }
      }

      vpMbtDistanceKltCylinder *kltPolyCylinder;
      for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it=kltCylinders.begin(); it!=kltCylinders.end(); ++it){
        kltPolyCylinder = *it;
        if(displayFeatures && kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints())
          kltPolyCylinder->displayPrimitive(I);
      }
    }

  #ifdef VISP_HAVE_OGRE
    if(useOgre)
      faces.displayOgre(cMo_);
  #endif
  }
}

void vpMbGenericTracker::TrackerWrapper::display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo_, const vpCameraParameters &camera,
                             const vpColor& col , const unsigned int thickness, const bool displayFullModel) {
  if ( m_trackerType == EDGE_TRACKER ) {
    vpMbEdgeTracker::display(I, cMo_, camera, col, thickness, displayFullModel);
  } else if ( m_trackerType == KLT_TRACKER ) {
    vpMbKltTracker::display(I, cMo_, camera, col, thickness, displayFullModel);
  } else {
    if (m_trackerType & EDGE_TRACKER) {
      for (unsigned int i = 0; i < scales.size(); i += 1){
        if(scales[i]){
          for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
            (*it)->display(I,cMo_, camera, col, thickness, displayFullModel);
          }

          for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
            (*it)->display(I, cMo_, camera, col, thickness, displayFullModel);
          }

          for(std::list<vpMbtDistanceCircle*>::const_iterator it=circles[scaleLevel].begin(); it!=circles[scaleLevel].end(); ++it){
            (*it)->display(I, cMo_, camera, col, thickness, displayFullModel);
          }

          break; //displaying model on one scale only
        }
      }
    }

    if (m_trackerType & KLT_TRACKER) {
      vpMbtDistanceKltPoints *kltpoly;
      for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
        kltpoly = *it;
        if(displayFeatures && kltpoly->hasEnoughPoints() && kltpoly->isTracked() && kltpoly->polygon->isVisible()) {
            kltpoly->displayPrimitive(I);
        }
      }

      vpMbtDistanceKltCylinder *kltPolyCylinder;
      for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it=kltCylinders.begin(); it!=kltCylinders.end(); ++it){
        kltPolyCylinder = *it;
        if(displayFeatures && kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints())
          kltPolyCylinder->displayPrimitive(I);
      }
    }

  #ifdef VISP_HAVE_OGRE
    if(useOgre)
      faces.displayOgre(cMo_);
  #endif
  }
}

void vpMbGenericTracker::TrackerWrapper::init(const vpImage<unsigned char>& I) {
  if(!modelInitialised){
    throw vpException(vpException::fatalError, "model not initialized");
  }

  if (clippingFlag > 2)
    cam.computeFov(I.getWidth(), I.getHeight());

  bool reInitialisation = false;
  if (!useOgre) {
    faces.setVisible(I, cam, cMo, angleAppears, angleDisappears, reInitialisation);
  } else {
#ifdef VISP_HAVE_OGRE
    if(!faces.isOgreInitialised()){
      faces.setBackgroundSizeOgre(I.getHeight(), I.getWidth());

      faces.setOgreShowConfigDialog(ogreShowConfigDialog);
      faces.initOgre(cam);
      // Turn off Ogre config dialog display for the next call to this function
      // since settings are saved in the ogre.cfg file and used during the next
      // call
      ogreShowConfigDialog = false;
    }

    faces.setVisibleOgre(I, cam, cMo, angleAppears, angleDisappears, reInitialisation);
#else
    faces.setVisible(I, cam, cMo, angleAppears, angleDisappears, reInitialisation);
#endif
  }

  if (useScanLine) {
    if (clippingFlag <= 2)
      cam.computeFov(I.getWidth(), I.getHeight());

    faces.computeClippedPolygons(cMo, cam);
    faces.computeScanLineRender(cam, I.getWidth(), I.getHeight());
  }


  if (m_trackerType & KLT_TRACKER)
    vpMbKltTracker::reinit(I);

  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::resetMovingEdge();

    bool a = false;
    vpMbEdgeTracker::visibleFace(I, cMo, a);

    initMovingEdge(I, cMo);
  }
}

void vpMbGenericTracker::TrackerWrapper::initCircle(const vpPoint& p1, const vpPoint &p2, const vpPoint &p3, const double radius,
                                const int idFace, const std::string &name) {
  if (m_trackerType & EDGE_TRACKER)
    vpMbEdgeTracker::initCircle(p1, p2, p3, radius, idFace, name);
}

void vpMbGenericTracker::TrackerWrapper::initCylinder(const vpPoint& p1, const vpPoint &p2, const double radius, const int idFace,
                                  const std::string &name) {
  if (m_trackerType & EDGE_TRACKER)
    vpMbEdgeTracker::initCylinder(p1, p2, radius, idFace, name);

  if (m_trackerType & KLT_TRACKER)
    vpMbKltTracker::initCylinder(p1, p2, radius, idFace, name);
}

void vpMbGenericTracker::TrackerWrapper::initFaceFromCorners(vpMbtPolygon &polygon) {
  if (m_trackerType & EDGE_TRACKER)
    vpMbEdgeTracker::initFaceFromCorners(polygon);

  if (m_trackerType & KLT_TRACKER)
    vpMbKltTracker::initFaceFromCorners(polygon);
}

void vpMbGenericTracker::TrackerWrapper::initFaceFromLines(vpMbtPolygon &polygon) {
  if (m_trackerType & EDGE_TRACKER)
    vpMbEdgeTracker::initFaceFromLines(polygon);

  if (m_trackerType & KLT_TRACKER)
    vpMbKltTracker::initFaceFromLines(polygon);
}

void vpMbGenericTracker::TrackerWrapper::initMbtTracking(const vpImage<unsigned char> &I) {
  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::computeVVSInit();
    vpMbEdgeTracker::computeVVSFirstPhaseFactor(I, 0);
  }
}

void vpMbGenericTracker::TrackerWrapper::loadConfigFile(const std::string& configFile) {
#ifdef VISP_HAVE_XML2
  vpMbtEdgeKltXmlParser xmlp;

  xmlp.setCameraParameters(cam);
  xmlp.setAngleAppear(vpMath::deg(angleAppears));
  xmlp.setAngleDisappear(vpMath::deg(angleDisappears));

  xmlp.setMovingEdge(me);

  xmlp.setMaxFeatures(10000);
  xmlp.setWindowSize(5);
  xmlp.setQuality(0.01);
  xmlp.setMinDistance(5);
  xmlp.setHarrisParam(0.01);
  xmlp.setBlockSize(3);
  xmlp.setPyramidLevels(3);
  xmlp.setMaskBorder(maskBorder);

  try{
    std::cout << " *********** Parsing XML for Mb Edge Tracker ************ " << std::endl;
    xmlp.parse(configFile.c_str());
  }
  catch(...){
    vpERROR_TRACE("Can't open XML file \"%s\"\n ", configFile);
    throw vpException(vpException::ioError, "problem to parse configuration file.");
  }

  vpCameraParameters camera;
  xmlp.getCameraParameters(camera);
  setCameraParameters(camera);

  angleAppears = vpMath::rad(xmlp.getAngleAppear());
  angleDisappears = vpMath::rad(xmlp.getAngleDisappear());

  if(xmlp.hasNearClippingDistance())
    setNearClippingDistance(xmlp.getNearClippingDistance());

  if(xmlp.hasFarClippingDistance())
    setFarClippingDistance(xmlp.getFarClippingDistance());

  if(xmlp.getFovClipping()){
    setClipping(vpMbEdgeTracker::clippingFlag | vpPolygon3D::FOV_CLIPPING);
  }

  useLodGeneral = xmlp.getLodState();
  minLineLengthThresholdGeneral = xmlp.getMinLineLengthThreshold();
  minPolygonAreaThresholdGeneral = xmlp.getMinPolygonAreaThreshold();

  applyLodSettingInConfig = false;
  if(this->getNbPolygon() > 0) {
    applyLodSettingInConfig = true;
    setLod(useLodGeneral);
    setMinLineLengthThresh(minLineLengthThresholdGeneral);
    setMinPolygonAreaThresh(minPolygonAreaThresholdGeneral);
  }

  vpMe meParser;
  xmlp.getMe(meParser);
  vpMbEdgeTracker::setMovingEdge(meParser);

  tracker.setMaxFeatures((int)xmlp.getMaxFeatures());
  tracker.setWindowSize((int)xmlp.getWindowSize());
  tracker.setQuality(xmlp.getQuality());
  tracker.setMinDistance(xmlp.getMinDistance());
  tracker.setHarrisFreeParameter(xmlp.getHarrisParam());
  tracker.setBlockSize((int)xmlp.getBlockSize());
  tracker.setPyramidLevels((int)xmlp.getPyramidLevels());
  maskBorder = xmlp.getMaskBorder();

  //if(useScanLine)
  faces.getMbScanLineRenderer().setMaskBorder(maskBorder);

#else
  vpTRACE("You need the libXML2 to read the config file %s", configFile);
#endif
}

void vpMbGenericTracker::TrackerWrapper::preTracking(const vpImage<unsigned char> &I) {
  if (m_trackerType & EDGE_TRACKER) {
    try {
      vpMbEdgeTracker::trackMovingEdge(I);
    } catch (...) {
      std::cerr << "Error in moving edge tracking" << std::endl;
      throw;
    }
  }

  if (m_trackerType & KLT_TRACKER) {
    try {
      vpMbKltTracker::preTracking(I);
    } catch (...) {
      std::cerr << "Error in KLT tracking" << std::endl;
      throw;
    }
  }
}

void vpMbGenericTracker::TrackerWrapper::postTracking(const vpImage<unsigned char> &I) {
  if (displayFeatures) {
    if (m_trackerType & EDGE_TRACKER) {
      vpMbEdgeTracker::displayFeaturesOnImage(I, 0);
    }
  }

  //KLT
  if (m_trackerType & KLT_TRACKER) {
    if (vpMbKltTracker::postTracking(I, m_w_klt)) {
      vpMbKltTracker::reinit(I);
    }
  }

  // Looking for new visible face
  if (m_trackerType & EDGE_TRACKER) {
    bool newvisibleface = false ;
    vpMbEdgeTracker::visibleFace(I, cMo, newvisibleface);

    if (useScanLine) {
      faces.computeClippedPolygons(cMo, cam);
      faces.computeScanLineRender(cam, I.getWidth(), I.getHeight());
    }
  }

  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::updateMovingEdge(I);

    vpMbEdgeTracker::initMovingEdge(I, cMo);
    // Reinit the moving edge for the lines which need it.
    vpMbEdgeTracker::reinitMovingEdge(I, cMo);

    if (computeProjError) {
      vpMbEdgeTracker::computeProjectionError(I);
    }
  }
}

void vpMbGenericTracker::TrackerWrapper::reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name, const vpHomogeneousMatrix &cMo_, const bool verbose) {
  cMo.eye();


  //Edge
  vpMbtDistanceLine *l;
  vpMbtDistanceCylinder *cy;
  vpMbtDistanceCircle *ci;

  for (unsigned int i = 0; i < scales.size(); i++) {
    if (scales[i]) {
      for (std::list<vpMbtDistanceLine*>::const_iterator it = lines[i].begin(); it != lines[i].end(); ++it) {
        l = *it;
        if (l != NULL) delete l;
        l = NULL;
      }

      for (std::list<vpMbtDistanceCylinder*>::const_iterator it = cylinders[i].begin(); it != cylinders[i].end(); ++it) {
        cy = *it;
        if (cy != NULL) delete cy;
        cy = NULL;
      }

      for (std::list<vpMbtDistanceCircle*>::const_iterator it = circles[i].begin(); it != circles[i].end(); ++it) {
        ci = *it;
        if (ci != NULL) delete ci;
        ci = NULL;
      }

      lines[i].clear();
      cylinders[i].clear();
      circles[i].clear();
    }
  }

  nline = 0;
  ncylinder = 0;
  ncircle = 0;
  nbvisiblepolygone = 0;


  //KLT
  // delete the Klt Polygon features
  vpMbtDistanceKltPoints *kltpoly;
  for (std::list<vpMbtDistanceKltPoints*>::const_iterator it = kltPolygons.begin(); it != kltPolygons.end(); ++it) {
    kltpoly = *it;
    if (kltpoly != NULL) {
      delete kltpoly;
    }
    kltpoly = NULL;
  }
  kltPolygons.clear();

  vpMbtDistanceKltCylinder *kltPolyCylinder;
  for (std::list<vpMbtDistanceKltCylinder*>::const_iterator it = kltCylinders.begin(); it != kltCylinders.end(); ++it) {
    kltPolyCylinder = *it;
    if (kltPolyCylinder!=NULL) {
      delete kltPolyCylinder;
    }
    kltPolyCylinder = NULL;
  }
  kltCylinders.clear();

  // delete the structures used to display circles
  for (std::list<vpMbtDistanceCircle*>::const_iterator it = circles_disp.begin(); it != circles_disp.end(); ++it) {
    ci = *it;
    if (ci!=NULL) {
      delete ci;
    }
    ci = NULL;
  }


  faces.reset();

  loadModel(cad_name, verbose);
  initFromPose(I, cMo_);
}

void vpMbGenericTracker::TrackerWrapper::resetTracker() {
  vpMbEdgeTracker::resetTracker();
  vpMbKltTracker::resetTracker();
}

void vpMbGenericTracker::TrackerWrapper::setCameraParameters(const vpCameraParameters &camera) {
  this->cam = camera;

  vpMbEdgeTracker::setCameraParameters(cam);
  vpMbKltTracker::setCameraParameters(cam);
}

void vpMbGenericTracker::TrackerWrapper::setOgreVisibilityTest(const bool &v) {
  vpMbTracker::setOgreVisibilityTest(v);
#ifdef VISP_HAVE_OGRE
  faces.getOgreContext()->setWindowName("TrackerWrapper");
#endif
}

void vpMbGenericTracker::TrackerWrapper::setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix& cdMo) {
  vpMbKltTracker::setPose(I, cdMo);

  resetMovingEdge();

  if (useScanLine) {
    cam.computeFov(I.getWidth(), I.getHeight());
    faces.computeClippedPolygons(cMo,cam);
    faces.computeScanLineRender(cam, I.getWidth(), I.getHeight());
  }

  initPyramid(I, Ipyramid);

  unsigned int i = (unsigned int) scales.size();
  do {
    i--;
    if(scales[i]){
      downScale(i);
      initMovingEdge(*Ipyramid[i], cMo);
      upScale(i);
    }
  } while(i != 0);

  cleanPyramid(Ipyramid);
}

void vpMbGenericTracker::TrackerWrapper::setProjectionErrorComputation(const bool &flag) {
  vpMbEdgeTracker::setProjectionErrorComputation(flag);
}

void vpMbGenericTracker::TrackerWrapper::setScanLineVisibilityTest(const bool &v) {
  vpMbEdgeTracker::setScanLineVisibilityTest(v);
  vpMbKltTracker::setScanLineVisibilityTest(v);
}

void vpMbGenericTracker::TrackerWrapper::setTrackerType(const int type) {
  m_trackerType = type;
}

void vpMbGenericTracker::TrackerWrapper::testTracking() {
  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::testTracking();
  }
}

void vpMbGenericTracker::TrackerWrapper::track(const vpImage<unsigned char> &I) {
  if ( (m_trackerType & (EDGE_TRACKER | KLT_TRACKER)) == 0 ) {
    std::cerr << "Bad tracker type: " << m_trackerType << std::endl;
    return;
  }

  //Back-up cMo in case of exception
  vpHomogeneousMatrix cMo_1 = cMo;
  try {
    preTracking(I);

    try {
      computeVVS(I);
    } catch (...) {
      covarianceMatrix = -1;
      throw; // throw the original exception
    }

    if (m_trackerType == EDGE_TRACKER)
      testTracking();

    postTracking(I);

  } catch (vpException &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    cMo = cMo_1;
    throw e;
  }
}
