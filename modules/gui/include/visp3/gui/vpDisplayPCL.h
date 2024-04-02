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
 * Display a point cloud using PCL library.
 */

#ifndef _vpDisplayPCL_h_
#define _vpDisplayPCL_h_

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_THREADS)

#include <mutex>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

class VISP_EXPORT vpDisplayPCL
{
public:
  explicit vpDisplayPCL();

  void flush();

  void run(std::mutex &mutex, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud);
  ;
  void setVerbose(bool verbose);
  void stop();

private:
  bool m_stop;
  bool m_flush_viewer;
  bool m_verbose;
};

#endif

#endif
