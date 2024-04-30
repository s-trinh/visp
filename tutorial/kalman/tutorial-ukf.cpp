/****************************************************************************
 *
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
*****************************************************************************/

/** \example tutorial-ukf.cpp
 * Tutorial on how to use the Unscented Kalman Filter (UKF) on a complex non-linear use-case.
 * The system is a plate, whose coordinate frame origin is the point O, on which are sticked four markers.
 * The plate revolves around a fixed point W whose coordinate frame is the world frame.
 * The scene is observed by a pinhole camera whose coordinate frame has the origin C.
 *
 * The state vector of the UKF is:
 *  \f{eqnarray*}{
        \textbf{x}[0] &=& {}^WX_x \\
        \textbf{x}[1] &=& {}^WX_y \\
        \textbf{x}[2] &=& {}^WX_z \\
        \textbf{x}[3] &=& \omega \Delta t
   \f}

   The measurement \f$ \textbf{z} \f$ corresponds to the coordinates in pixels of the different markers.
   Be \f$ u_i \f$ and \f$ v_i \f$ the horizontal and vertical pixel coordinates of the \f$ i^{th} \f$ marker.
   The measurement vector can be written as:
   \f{eqnarray*}{
        \textbf{z}[2i] &=& u_i \\
        \textbf{z}[2i+1] &=& v_i
   \f}

 * Some noise is added to the measurement vector to simulate measurements which are
 * not perfect.
*/

// UKF includes
#include <visp3/core/vpUKSigmaDrawerMerwe.h>
#include <visp3/core/vpUnscentedKalman.h>

// ViSP includes
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpGaussRand.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>
#ifdef VISP_HAVE_DISPLAY
#include <visp3/gui/vpPlot.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#endif
#include <visp3/vision/vpPose.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
/**
 * \brief Process function that makes evolve the state model {\f$ {}^WX_x \f$, \f$ {}^WX_y \f$, \f$ {}^WX_z \f$, \f$ C = \omega \Delta t \f$}
 * over time.
 *
 * \param[in] x The state vector
 * \return vpColVector The state vector at the next iteration.
 */
vpColVector fx(const vpColVector &x, const double & /*dt*/)
{
  vpColVector x_kPlus1(4);
  x_kPlus1[0] = x[0] * std::cos(x[3]) - x[2] * std::sin(x[3]); // wX
  x_kPlus1[1] = x[1]; // wY
  x_kPlus1[2] = x[0] * std::sin(x[3]) + x[2] * std::cos(x[3]); // wZ
  x_kPlus1[3] = x[3]; // omega * dt
  return x_kPlus1;
}

/**
 * \brief Class that simulates the moving plate.
 */
class vpPlateSimulator
{
public:
  /**
   * \brief Construct a new vpPlateSimulator object.
   *
   * \param[in] R The radius of the revolution around the world frame origin.
   * \param[in] w The pulsation of the motion.
   * \param[in] phi The phase of the motion.
   * \param[in] wY The y-coordinate of the object in the world frame.
   */
  vpPlateSimulator(const double &R, const double &w, const double &phi, const double &wY)
    : m_R(R)
    , m_w(w)
    , m_phi(phi)
    , m_wY(wY)
  { }

  /**
   * \brief Move the object to its new position, expressed in the world frame.
   *
   * \param[in] t The current time.
   * \return vpColVector The new position of the object in the world frame, expressed as homogeneous coordinates.
   */
  vpColVector move(const double &t)
  {
    vpColVector wX(4, 1.);
    wX[0] = m_R * std::cos(m_w * t + m_phi);
    wX[1] = m_wY;
    wX[2] = m_R * std::sin(m_w * t + m_phi);
    return wX;
  }

private:
  double m_R; // Radius of the revolution around the world frame origin.
  double m_w; // Pulsation of the motion.
  double m_phi; // Phase of the motion.
  const double m_wY; // The y-coordinate of the object in the world frame.
};

/**
 * \brief Class that permits to convert the 3D position of the plate into measurements.
 */
class vpMarkersMeasurements
{
public:
  /**
   * \brief Construct a new vpMarkersMeasurements object.
   *
   * \param[in] cam The camera parameters.
   * \param[in] cMw The pose of the world frame with regard to the camera frame.
   * \param[in] wRo The rotation matrix expressing the rotation between the world frame and object frame.
   * \param[in] markers The position of the markers in the object frame.
   * \param[in] noise_stdev The standard deviation for the noise generator
   * \param[in] seed The seed for the noise generator
   */
  vpMarkersMeasurements(const vpCameraParameters &cam, const vpHomogeneousMatrix &cMw, const vpRotationMatrix &wRo,
                        const std::vector<vpColVector> &markers, const double &noise_stdev, const long &seed)
    : m_cam(cam)
    , m_cMw(cMw)
    , m_wRo(wRo)
    , m_markers(markers)
    , m_rng(noise_stdev, 0., seed)
  { }

  /**
   * \brief Convert the prior of the UKF into the measurement space.
   *
   * \param[in] x The prior.
   * \return vpColVector The prior expressed in the measurement space.
   */
  vpColVector state_to_measurement(const vpColVector &x)
  {
    unsigned int nbMarkers = m_markers.size();
    vpColVector meas(2*nbMarkers);
    vpHomogeneousMatrix wMo;
    vpTranslationVector wTo(x[0], x[1], x[2]);
    wMo.buildFrom(wTo, m_wRo);
    for (unsigned int i = 0; i < nbMarkers; ++i) {
      vpColVector cX = m_cMw * wMo * m_markers[i];
      double u = 0., v = 0.;
      vpMeterPixelConversion::convertPoint(m_cam, cX[0] / cX[2], cX[1] / cX[2], u, v);
      meas[2*i] = u;
      meas[2*i + 1] = v;
    }
    return meas;
  }

  /**
   * \brief Perfect measurement of the projection of the markers in the image when the plate
   * is located at \b wX.
   *
   * \param[in] wX The actual position of the robot (wX[0]: x, wX[1]: y, wX[2] = z.
   * \return vpColVector [2*i] u_i [2*i + 1] v_i where i is the index of the marker.
   */
  vpColVector measureGT(const vpColVector &wX)
  {
    unsigned int nbMarkers = m_markers.size();
    vpColVector meas(2*nbMarkers);
    vpHomogeneousMatrix wMo;
    vpTranslationVector wTo(wX[0], wX[1], wX[2]);
    wMo.buildFrom(wTo, m_wRo);
    for (unsigned int i = 0; i < nbMarkers; ++i) {
      vpColVector cX = m_cMw * wMo * m_markers[i];
      double u = 0., v = 0.;
      vpMeterPixelConversion::convertPoint(m_cam, cX[0] / cX[2], cX[1] / cX[2], u, v);
      meas[2*i] = u;
      meas[2*i + 1] = v;
    }
    return meas;
  }

  /**
   * \brief Noisy measurement of the projection of the markers in the image when the plate
   * is located at \b wX.
   *
   * \param[in] wX The actual position of the robot (wX[0]: x, wX[1]: y, wX[2] = z.
   * \return vpColVector [2*i] u_i [2*i + 1] v_i where i is the index of the marker.
   */
  vpColVector measureWithNoise(const vpColVector &wX)
  {
    vpColVector measurementsGT = measureGT(wX);
    vpColVector measurementsNoisy = measurementsGT;
    unsigned int sizeMeasurement = measurementsGT.size();
    for (unsigned int i = 0; i < sizeMeasurement; ++i) {
      measurementsNoisy[i] += m_rng();
    }
    return measurementsNoisy;
  }

private:
  vpCameraParameters m_cam; // The camera parameters
  vpHomogeneousMatrix m_cMw; // The pose of the world frame with regard to the camera frame.
  vpRotationMatrix m_wRo; // The rotation matrix that expresses the rotation between the world frame and object frame.
  std::vector<vpColVector> m_markers; // The position of the markers in the object frame.
  vpGaussRand m_rng; // Noise simulator for the measurements
};

/**
 * \brief Compute the pose from the 3D coordinates of the markers and their coordinates in pixels
 * in the image.
 *
 * \param[in] point The 3D coordinates of the markers in the object frame.
 * \param[in] ip The pixel coordinates of the markers in the image.
 * \param[in] cam The camera parameters used to acquire the image.
 * \return vpHomogeneousMatrix The pose of the object in the camera frame.
 */
vpHomogeneousMatrix computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &ip, const vpCameraParameters &cam)
{
  vpPose pose;
  double x = 0, y = 0;
  for (unsigned int i = 0; i < point.size(); i++) {
    vpPixelMeterConversion::convertPoint(cam, ip[i], x, y);
    point[i].set_x(x);
    point[i].set_y(y);
    pose.addPoint(point[i]);
  }

  vpHomogeneousMatrix cMo;
  pose.computePose(vpPose::DEMENTHON_LAGRANGE_VIRTUAL_VS, cMo);
  return cMo;
}

int main(/*const int argc, const char *argv[]*/)
{
  const double dt = 0.001; // Period of 0.1s
  const double sigmaMeasurements = 2.; // Standard deviation of the measurements: 2 pixels
  const double radius = 0.5; // Radius of revolution of 0.5m
  const double w = 2 * M_PI * 10; // Pulsation of the motion of revolution
  const double phi = 2; // Phase of the motion of revolution
  const std::vector<vpColVector> markers = { vpColVector({-0.05, 0., -0.05, 1.})
                                           , vpColVector({0.05, 0., -0.05, 1.})
                                           , vpColVector({0.05, 0., 0.05, 1.})
                                           , vpColVector({-0.05, 0., 0.05, 1.}) }; // Vector of the markers sticked on the object
  const unsigned int nbMarkers = markers.size();
  std::vector<vpPoint> markersAsVpPoint;
  for (unsigned int i = 0; i < nbMarkers; ++i) {
    vpColVector marker = markers[i];
    markersAsVpPoint.push_back(vpPoint(marker[0], marker[1], marker[2]));
  }

  const long seed = 42; // Seed for the random generator
  vpHomogeneousMatrix cMw; // Pose of the world frame with regard to the camera frame
  cMw[0][0] = 1.; cMw[0][1] = 0.; cMw[0][2] = 0.; cMw[0][3] = 0.2;
  cMw[1][0] = 0.; cMw[1][1] = 0.; cMw[1][2] = -1.; cMw[1][3] = 0.3;
  cMw[2][0] = 0.; cMw[2][1] = 1.; cMw[2][2] = 0.; cMw[2][3] = 1.;

  vpHomogeneousMatrix wMo; // Pose of the object frame with regard to the world frame
  wMo[0][0] = 1.; wMo[0][1] = 0.; wMo[0][2] = 0.; wMo[0][3] = radius;
  wMo[1][0] = 0.; wMo[1][1] = 1.; wMo[1][2] = 0.; wMo[1][3] = 0;
  wMo[2][0] = 0.; wMo[2][1] = 0.; wMo[2][2] = 1.; wMo[2][3] = 0.2;
  vpRotationMatrix wRo; // Rotation between the object frame and world frame
  wMo.extract(wRo);
  const double wY = wMo[1][3];

  // Create a camera parameter container
  // Camera initialization with a perspective projection without distortion model
  double px = 600; double py = 600; double u0 = 320; double v0 = 240;
  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(px, py, u0, v0);

  // Initialize the attributes of the UKF
  std::shared_ptr<vpUKSigmaDrawerAbstract> drawer = std::make_shared<vpUKSigmaDrawerMerwe>(4, 0.001, 2., -1);

  vpMatrix R1landmark(2, 2, 0.); // The covariance of the noise introduced by the measurement with 1 landmark
  R1landmark[0][0] = sigmaMeasurements*sigmaMeasurements;
  R1landmark[1][1] = sigmaMeasurements*sigmaMeasurements;
  vpMatrix R(2*nbMarkers, 2 * nbMarkers);
  for (unsigned int i = 0; i < nbMarkers; ++i) {
    R.insert(R1landmark, 2*i, 2*i);
  }

  const double processVariance = 0.000025; // Variance of the process of (0.005cm)^2
  vpMatrix Q; // The covariance of the process
  Q.eye(4);
  Q = Q * processVariance;

  vpMatrix P0(4, 4); //  The initial guess of the process covariance
  P0.eye(4);
  P0[0][0] = 1.;
  P0[1][1] = 1.;
  P0[2][2] = 1.;
  P0[2][2] = 5.;

  vpColVector X0(4); // The initial guess for the state
  X0[0] = radius; // wX = radius m
  X0[1] = 0.95 * wY; // Wrong estimation of the position along the y-axis: error of 5%
  X0[2] = 0; // wZ = 0m
  X0[3] = 0.75 * w * dt; // Wrong estimation of the pulsation: error of 25%

  vpUnscentedKalman::vpProcessFunction f = fx;
  vpMarkersMeasurements markerMeas(cam, cMw, wRo, markers, sigmaMeasurements, seed);
  vpPlateSimulator plate(radius, w, phi, wY);
  using std::placeholders::_1;
  vpUnscentedKalman::vpMeasurementFunction h = std::bind(&vpMarkersMeasurements::state_to_measurement, &markerMeas, _1);

  // Initialize the UKF
  vpUnscentedKalman ukf(Q, R, drawer, f, h);
  ukf.init(X0, P0);

#ifdef VISP_HAVE_DISPLAY
  // Initialize the plot
  vpPlot plot(1);
  plot.initGraph(0, 3);
  plot.setTitle(0, "Position of the robot wX");
  plot.setUnitX(0, "Position along x(m)");
  plot.setUnitY(0, "Position along z (m)");
  plot.setLegend(0, 0, "GT");
  plot.setLegend(0, 1, "Filtered");
  plot.setLegend(0, 2, "Measure");
  plot.initRange(0, -1.25 * radius, 1.25 * radius, -1.25 * radius, 1.25 * radius);
  plot.setColor(0, 0, vpColor::red);
  plot.setColor(0, 1, vpColor::blue);
  plot.setColor(0, 2, vpColor::black);
#endif

  // Initialize the display
  // Depending on the detected third party libraries, we instantiate here the
  // first video device which is available
  vpDisplay *d = nullptr;
#if defined(VISP_HAVE_X11)
  d = new vpDisplayX;
#elif defined(VISP_HAVE_GTK)
  d = new vpDisplayGTK;
#elif defined(VISP_HAVE_GDI)
  d = new vpDisplayGDI;
#elif defined(VISP_HAVE_D3D9)
  d = new vpDisplayD3D;
#elif defined(HAVE_OPENCV_HIGHGUI)
  d = new vpDisplayOpenCV;
#endif
  vpImage<vpRGBa> Idisp(800, 800, vpRGBa(255));
  if (d != nullptr) {
    d->init(Idisp, 800, 50, "Projection of the markers");
  }

  // Initialize the simulation
  vpColVector plate_pos = X0;

  for (unsigned int i = 0; i < 200; ++i) {
    plate_pos = plate.move(dt * static_cast<double>(i));

    // Perform the measurement
    vpColVector z = markerMeas.measureWithNoise(plate_pos);

    // Use the UKF to filter the measurement
    ukf.filter(z, dt);

#ifdef VISP_HAVE_DISPLAY
    // Plot the filtered state
    vpColVector Xest = ukf.getXest();
    plot.plot(0, 1, Xest[0], Xest[2]);

    // Plot the ground truth
    plot.plot(0, 0, plate_pos[0], plate_pos[2]);

    // Display the projection of the markers
    vpDisplay::display(Idisp);
    vpColVector zGT = markerMeas.measureGT(plate_pos);
    vpColVector zFilt = markerMeas.state_to_measurement(Xest);
    std::vector<vpImagePoint> ip;
    for (unsigned int id = 0; id < nbMarkers; ++id) {
      vpImagePoint markerProjGT(zGT[2*id + 1], zGT[2*id]);
      vpDisplay::displayCross(Idisp, markerProjGT, 5, vpColor::red);

      vpImagePoint markerProjFilt(zFilt[2*id + 1], zFilt[2*id]);
      vpDisplay::displayCross(Idisp, markerProjFilt, 5, vpColor::blue);

      vpImagePoint markerProjNoisy(z[2*id + 1], z[2*id]);
      vpDisplay::displayCross(Idisp, markerProjNoisy, 5, vpColor::black);
      ip.push_back(markerProjNoisy);
    }

    // Compute the pose using the noisy markers
    vpHomogeneousMatrix cMo_noisy = computePose(markersAsVpPoint, ip, cam);
    vpHomogeneousMatrix wMo_noisy = cMw.inverse() * cMo_noisy;
    double wXnoisy = wMo_noisy[0][3];
    double wZnoisy = wMo_noisy[2][3];
    plot.plot(0, 2, wXnoisy, wZnoisy);

    vpImagePoint ipText(20, 20);
    vpDisplay::displayText(Idisp, ipText, std::string("GT"), vpColor::red);
    ipText.set_i(ipText.get_i() + 20);
    vpDisplay::displayText(Idisp, ipText, std::string("Filtered"), vpColor::blue);
    ipText.set_i(ipText.get_i() + 20);
    vpDisplay::displayText(Idisp, ipText, std::string("Measured"), vpColor::black);
    vpDisplay::flush(Idisp);
    vpTime::wait(40);
#endif
  }
  std::cout << "Press Enter to quit..." << std::endl;
  std::cin.get();

  if (d != nullptr) {
    delete d;
  }
  return 0;
}
#else
int main()
{
  std::cout << "vpUnscentedKalman is only available if you compile ViSP in C++11 standard or higher." << std::endl;
  return 0;
}
#endif
