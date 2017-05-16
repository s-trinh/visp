#ifndef __vpMbGenericTracker_h_
#define __vpMbGenericTracker_h_

#include <iostream>

#include <visp3/mbt/vpMbEdgeTracker.h>
#include <visp3/mbt/vpMbKltTracker.h>


class VISP_EXPORT vpMbGenericTracker : public vpMbTracker {
public:
  enum TRACKER_TYPE {
    EDGE_TRACKER  = 1 << 0,
    KLT_TRACKER   = 1 << 1
  };

  vpMbGenericTracker();
  vpMbGenericTracker(const unsigned int nbCameras, const int trackerType=EDGE_TRACKER);
  vpMbGenericTracker(const std::vector<int> &trackerTypes);

  virtual ~vpMbGenericTracker();

  virtual void display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor& col, const unsigned int thickness=1, const bool displayFullModel = false);
  virtual void display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor& col, const unsigned int thickness=1, const bool displayFullModel = false);

  virtual void display(const vpImage<unsigned char>& I1, const vpImage<unsigned char>& I2, const vpHomogeneousMatrix &c1Mo,
                       const vpHomogeneousMatrix &c2Mo, const vpCameraParameters &cam1, const vpCameraParameters &cam2,
                       const vpColor& color, const unsigned int thickness=1, const bool displayFullModel = false);
  virtual void display(const vpImage<vpRGBa>& I1, const vpImage<vpRGBa>& I2, const vpHomogeneousMatrix &c1Mo,
                       const vpHomogeneousMatrix &c2Mo, const vpCameraParameters &cam1, const vpCameraParameters &cam2,
                       const vpColor& color, const unsigned int thickness=1, const bool displayFullModel = false);

  virtual void display(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                       const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
                       const std::map<std::string, vpCameraParameters> &mapOfCameraParameters,
                       const vpColor& col, const unsigned int thickness=1, const bool displayFullModel=false);
  virtual void display(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfImages,
                       const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
                       const std::map<std::string, vpCameraParameters> &mapOfCameraParameters,
                       const vpColor& col, const unsigned int thickness=1, const bool displayFullModel=false);

  virtual std::vector<std::string> getCameraNames() const;

  using vpMbTracker::getCameraParameters;
  virtual void getCameraParameters(vpCameraParameters &cam1, vpCameraParameters &cam2) const;
  virtual void getCameraParameters(std::map<std::string, vpCameraParameters> &mapOfCameraParameters) const;

  virtual std::map<std::string, int> getCameraTrackerTypes() const;

  virtual inline vpColVector getError() const {
    return m_error;
  }

  virtual std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > getPolygonFaces(const bool orderPolygons=true,
                                                                                                 const bool useVisibility=true,
                                                                                                 const bool clipPolygon=false);

  using vpMbTracker::getPose;
  virtual void getPose(vpHomogeneousMatrix &c1Mo, vpHomogeneousMatrix &c2Mo) const;
  virtual void getPose(std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) const;

  virtual inline vpColVector getRobustWeights() const {
    return m_w;
  }

  virtual void init(const vpImage<unsigned char>& I);

#ifdef VISP_HAVE_MODULE_GUI
  using vpMbTracker::initClick;
  virtual void initClick(const vpImage<unsigned char>& I1, const vpImage<unsigned char> &I2,
                         const std::string& initFile1, const std::string& initFile2,
                         const bool displayHelp=false);
  virtual void initClick(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                         const std::map<std::string, std::string> &mapOfInitFiles, const bool displayHelp=false);
#endif

  virtual void initFromPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                            const std::string &initFile1, const std::string &initFile2);
  virtual void initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                            const std::map<std::string, std::string> &mapOfInitPoses);

  virtual void initFromPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                            const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo);
  virtual void initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                            const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses);

  virtual void loadConfigFile(const std::string& configFile);
  virtual void loadConfigFile(const std::string& configFile1, const std::string& configFile2);
  virtual void loadConfigFile(const std::map<std::string, std::string> &mapOfConfigFiles);

  virtual void loadModel(const std::string &modelFile, const bool verbose=false);
  virtual void loadModel(const std::string &modelFile1, const std::string &modelFile2, const bool verbose=false);
  virtual void loadModel(const std::map<std::string, std::string> &mapOfModelFiles, const bool verbose=false);

  virtual void reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name, const vpHomogeneousMatrix &cMo_, const bool verbose=false);
  virtual void reInitModel(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, const std::string &cad_name,
                           const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo, const bool verbose=false);
  virtual void reInitModel(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                           const std::string &cad_name, const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
                           const bool verbose=false);

  virtual void resetTracker();

  virtual void setAngleAppear(const double &a);
  virtual void setAngleDisappear(const double &a);

  virtual void setCameraParameters(const vpCameraParameters& camera);
  virtual void setCameraParameters(const vpCameraParameters& camera1, const vpCameraParameters& camera2);
  virtual void setCameraParameters(const std::map<std::string, vpCameraParameters> &mapOfCameraParameters);

  virtual void setCameraTransformationMatrix(const std::string &cameraName, const vpHomogeneousMatrix &cameraTransformationMatrix);
  virtual void setCameraTransformationMatrix(const std::map<std::string, vpHomogeneousMatrix> &mapOfTransformationMatrix);

  virtual void setClipping(const unsigned int &flags);

  virtual void setDisplayFeatures(const bool displayF);

  virtual void setFarClippingDistance(const double &dist);

  virtual void setKltOpencv(const vpKltOpencv &t);

  virtual void setMaskBorder(const unsigned int &e);

  virtual void setMovingEdge(const vpMe &me);

  virtual void setNearClippingDistance(const double &dist);

  virtual void setOgreShowConfigDialog(const bool showConfigDialog);
  virtual void setOgreVisibilityTest(const bool &v);

  virtual void setOptimizationMethod(const vpMbtOptimizationMethod &opt);

  virtual void setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix& cdMo);
  virtual void setPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix c2Mo);
  virtual void setPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages, const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses);

  virtual void setProjectionErrorComputation(const bool &flag);

  virtual void setReferenceCameraName(const std::string &referenceCameraName);

  virtual void setScanLineVisibilityTest(const bool &v);

  virtual void setTrackerType(const int type);

  virtual void testTracking();

  virtual void track(const vpImage<unsigned char> &I);
  virtual void track(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2);
  virtual void track(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages);


protected:
  virtual void computeProjectionError();

  virtual void computeVVS(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages);

  virtual void computeVVSInit();
  virtual void computeVVSInit(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages);
  virtual void computeVVSInteractionMatrixAndResidu();
  virtual void computeVVSInteractionMatrixAndResidu(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                                    std::map<std::string, vpVelocityTwistMatrix> &mapOfVelocityTwist);
  virtual void computeVVSWeights();

  virtual void initCircle(const vpPoint& p1, const vpPoint &p2, const vpPoint &p3, const double radius,
                          const int idFace=0, const std::string &name="");

  virtual void initCylinder(const vpPoint& p1, const vpPoint &p2, const double radius, const int idFace=0,
                            const std::string &name="");

  virtual void initFaceFromCorners(vpMbtPolygon &polygon);

  virtual void initFaceFromLines(vpMbtPolygon &polygon);

  virtual void preTracking(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages);


private:
  class TrackerWrapper : public vpMbEdgeTracker, public vpMbKltTracker {
    friend class vpMbGenericTracker;

  public:
    //! Type of the tracker (a combination of the above)
    int m_trackerType;
    //! Interaction matrix
    vpMatrix m_L;
    //! (s - s*)
    vpColVector m_error;
    //! Robust weights
    vpColVector m_w;
    //! Weighted error
    vpColVector m_weightedError;


    TrackerWrapper();
    TrackerWrapper(const int trackerType);

    virtual ~TrackerWrapper();

    virtual inline vpColVector getError() const {
      return m_error;
    }

    virtual inline vpColVector getRobustWeights() const {
      return m_w;
    }

    virtual inline int getTrackerType() const {
      return m_trackerType;
    }

    virtual void display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                         const vpColor& col , const unsigned int thickness=1, const bool displayFullModel = false);
    virtual void display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                         const vpColor& col , const unsigned int thickness=1, const bool displayFullModel = false);

    virtual void init(const vpImage<unsigned char>& I);

    virtual void loadConfigFile(const std::string& configFile);

    virtual void reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name, const vpHomogeneousMatrix &cMo_, const bool verbose=false);

    virtual void resetTracker();

    virtual void setCameraParameters(const vpCameraParameters& camera);

    virtual void setOgreVisibilityTest(const bool &v);

    virtual void setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix& cdMo);

    virtual void setProjectionErrorComputation(const bool &flag);

    virtual void setScanLineVisibilityTest(const bool &v);

    virtual void setTrackerType(const int type);

    virtual void testTracking();

    virtual void track(const vpImage<unsigned char>& I);


  protected:
    virtual void computeVVS(const vpImage<unsigned char> &I);
    virtual void computeVVSInit();
    virtual void computeVVSInit(const vpImage<unsigned char> &I);
    virtual void computeVVSInteractionMatrixAndResidu();
    virtual void computeVVSInteractionMatrixAndResidu(const vpImage<unsigned char> &I);
    virtual void computeVVSWeights();

    virtual void initCircle(const vpPoint& p1, const vpPoint &p2, const vpPoint &p3, const double radius,
        const int idFace=0, const std::string &name="");

    virtual void initCylinder(const vpPoint& p1, const vpPoint &p2, const double radius, const int idFace=0,
        const std::string &name="");

    virtual void initFaceFromCorners(vpMbtPolygon &polygon);
    virtual void initFaceFromLines(vpMbtPolygon &polygon);

    virtual void initMbtTracking(const vpImage<unsigned char> &I);

    virtual void preTracking(const vpImage<unsigned char> &I);
    virtual void postTracking(const vpImage<unsigned char> &I);
  };


protected:
  enum FeatureType {
    LINE, CYLINDER, CIRCLE
  };

  //! Map of camera transformation matrix between the current camera frame to the reference camera frame (cCurrent_M_cRef)
  std::map<std::string, vpHomogeneousMatrix> m_mapOfCameraTransformationMatrix;

  //! Map of Model-based trackers, key is the name of the camera, value is the tracker
  std::map<std::string, TrackerWrapper*> m_mapOfTrackers;

  //! Name of the reference camera
  std::string m_referenceCameraName;

  //! Interaction matrix
  vpMatrix m_L;
  //! (s - s*)
  vpColVector m_error;
  //! Robust weights
  vpColVector m_w;
  //! Weighted error
  vpColVector m_weightedError;
};
#endif
