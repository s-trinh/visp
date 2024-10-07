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
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the articular frame
 */

/*!
  \file servoAfma6FourPoints2DArtVelocity.cpp
  \example servoAfma6FourPoints2DArtVelocity.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  Afma6 robot (cartesian robot, with 6 degrees of freedom). The velocity is
  computed in articular.  Visual features are the image coordinates of 4 vdot
  points.
*/

#include <iostream>
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_AFMA6) && defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_DISPLAY)

#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/blob/vpDot2.h>
#include <visp3/robot/vpRobotAfma6.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

// Define the object CAD model
// Here we consider 4 black blobs whose centers are located on the corners of a square.
#define L 0.06 // To deal with a 12cm by 12cm square

// Distance between the camera and the square at the desired
// position after visual servoing convergence
#define D 0.5

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  // Log file creation in /tmp/$USERNAME/log.dat
  // This file contains by line:
  // - the 6 computed joint velocities (m/s, rad/s) to achieve the task
  // - the 6 measured joint velocities (m/s, rad/s)
  // - the 6 measured joint positions (m, rad)
  // - the 8 values of s - s*

  // Get the user login name
  std::string username = vpIoTools::getUserName();

  // Create a log filename to save velocities...
  std::string logdirname = "/tmp/" + username;

  // Test if the output path exist. If no try to create it
  if (vpIoTools::checkDirectory(logdirname) == false) {
    try {
      // Create the dirname
      vpIoTools::makeDirectory(logdirname);
    }
    catch (...) {
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Cannot create " << logdirname << std::endl;
      return EXIT_FAILURE;
    }
  }
  std::string logfilename = logdirname + "/log.dat";

  // Open the log file name
  std::ofstream flog(logfilename.c_str());

  try {
    vpRealSense2 rs;
    rs2::config config;
    unsigned int width = 640, height = 480, fps = 60;
    config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, fps);
    config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
    config.enable_stream(RS2_STREAM_INFRARED, width, height, RS2_FORMAT_Y8, fps);
    rs.open(config);

    vpImage<unsigned char> I;

    // Warm up camera
    for (size_t i = 0; i < 10; ++i) {
      rs.acquire(I);
    }

    std::shared_ptr<vpDisplay> d = vpDisplayFactory::createDisplay(I, 100, 100, "Current image");

    vpDisplay::display(I);
    vpDisplay::flush(I);

    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << " Test program for vpServo " << std::endl;
    std::cout << " Eye-in-hand task control, velocity computed in the joint space" << std::endl;
    std::cout << " Use of the Afma6 robot " << std::endl;
    std::cout << " task : servo 4 points on a square with dimension " << L << " meters" << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;

    std::vector<vpDot2> dot(4);

    std::cout << "Click on the 4 dots clockwise starting from upper/left dot..." << std::endl;
    for (size_t i = 0; i < dot.size(); ++i) {
      dot[i].initTracking(I);
      vpImagePoint cog = dot[i].getCog();
      vpDisplay::displayCross(I, cog, 10, vpColor::blue);
      vpDisplay::flush(I);
    }

    vpRobotAfma6 robot;
    vpCameraParameters::vpCameraParametersProjType projModel = vpCameraParameters::perspectiveProjWithDistortion;

    // Load the end-effector to camera frame transformation obtained
    // using a camera intrinsic model with distortion
    robot.init(vpAfma6::TOOL_INTEL_D435_CAMERA, projModel);

    // Get camera intrinsics
    vpCameraParameters cam;
    robot.getCameraParameters(cam, I);

    // Sets the current position of the visual feature
    std::vector<vpFeaturePoint> s(4);
    for (size_t i = 0; i < s.size(); ++i) {
      vpFeatureBuilder::create(s[i], cam, dot[i]); // retrieve x,y  of the vpFeaturePoint structure
    }

    // Sets the desired position of the visual feature
    vpFeaturePoint s_d[4];

    s_d[0].buildFrom(-L/D, -L/D, D);
    s_d[1].buildFrom(+L/D, -L/D, D);
    s_d[2].buildFrom(+L/D, +L/D, D);
    s_d[3].buildFrom(-L/D, +L/D, D);

    // Define the task
    // - we want an eye-in-hand control law
    // - joint velocity are computed
    // - Interaction matrix is computed with the current visual features
    vpServo task;
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);

    // We want to see a point on a point
    for (size_t i = 0; i < s.size(); ++i) {
      task.addFeature(s[i], s_d[i]);
    }

    // Set task proportional gain
    task.setLambda(0.4);

    // Set the camera to end-effector velocity twist matrix transformation
    vpVelocityTwistMatrix c_V_e;
    robot.get_cVe(c_V_e);
    task.set_cVe(c_V_e);

    // Set the Jacobian (expressed in the end-effector frame)
    vpMatrix e_J_e;
    robot.get_eJe(e_J_e);
    task.set_eJe(e_J_e);

    // Display task information
    task.print();

    // Initialise the velocity control of the robot
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    std::cout << "\nHit CTRL-C to stop the loop...\n" << std::flush;

    bool quit = false;
    while (!quit) {
      // Acquire a new image from the camera
      rs.acquire(I);

      // Display this image
      vpDisplay::display(I);

      // For each point...
      for (size_t i = 0; i < dot.size(); ++i) {
        // Achieve the tracking of the dot in the image
        dot[i].track(I);
        // Update the point feature from the dot location
        vpFeatureBuilder::create(s[i], cam, dot[i]);
      }

      // Get the Jacobian of the robot
      robot.get_eJe(e_J_e);

      // Update this jacobian in the task structure. It will be used to
      task.set_eJe(e_J_e);

      // Compute the visual servoing skew vector (as a joint velocity)
      // qdot = -lambda * L^+ * cVe * eJe * (s-s*)
      vpColVector qdot = task.computeControlLaw();

      // Display the current and desired feature points in the image display
      vpServoDisplay::display(task, cam, I);

      // Apply the computed joint velocities to the robot
      robot.setVelocity(vpRobot::JOINT_STATE, qdot);

      // Save velocities applied to the robot in the log file
      // qdot[0], qdot[1], qdot[2] correspond to joint translation velocities in m/s
      // qdot[3], qdot[4], qdot[5] correspond to joint rotation velocities in rad/s
      flog << qdot[0] << " " << qdot[1] << " " << qdot[2] << " " << qdot[3] << " " << qdot[4] << " " << qdot[5] << " ";

      // Get the measured joint velocities of the robot
      vpColVector qdot_mes;
      robot.getVelocity(vpRobot::JOINT_STATE, qdot_mes);
      // Save measured joint velocities of the robot in the log file:
      // - qdot_mes[0], qdot_mes[1], qdot_mes[2] correspond to measured joint translation velocities in m/s
      // - qdot_mes[3], qdot_mes[4], qdot_mes[5] correspond to measured joint rotation velocities in rad/s
      flog << qdot_mes[0] << " " << qdot_mes[1] << " " << qdot_mes[2] << " " << qdot_mes[3] << " " << qdot_mes[4] << " " << qdot_mes[5] << " ";

      // Get the measured joint positions of the robot
      vpColVector q;
      robot.getPosition(vpRobot::JOINT_STATE, q);
      // Save measured joint positions of the robot in the log file
      // - q[0], q[1], q[2] correspond to measured joint translation
      //   positions in m
      // - q[3], q[4], q[5] correspond to measured joint rotation
      //   positions in rad
      flog << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << q[4] << " " << q[5] << " ";

      // Save feature error (s-s*) for the 4 feature points. For each feature
      // point, we have 2 errors (along x and y axis).  This error is
      // expressed in meters in the camera frame
      flog << (task.getError()).t() << std::endl;

      vpDisplay::displayText(I, 20, 20, "Click to quit...", vpColor::red);
      if (vpDisplay::getClick(I, false)) {
        quit = true;
      }
      // Flush the display
      vpDisplay::flush(I);
    }

    // Close the log file
    flog.close();

    // Display task information
    task.print();

    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    // Close the log file
    flog.close();
    std::cout << "Visual servo failed with exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}

#else
int main()
{
  std::cout << "You do not have an afma6 robot connected to your computer..." << std::endl;
  return EXIT_SUCCESS;
}

#endif
