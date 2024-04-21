/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * TODO: .
 */

/*!
  \file vpWriterExecutor.h
  \brief TODO: .
*/

#ifndef _vpWriterExecutor_h_
#define _vpWriterExecutor_h_

#include <visp3/core/vpConfig.h>
#include <thread>
#include <vector>
#include <visp3/core/vpWriterWorker.h>

/*!
  \class vpWriterExecutor
*/

class VISP_EXPORT vpWriterExecutor
{
public:
  // vpWriterExecutor(const std::vector<std::reference_wrapper<std::shared_ptr<vpWriterWorker>>> &workers)
  //   : m_writer_workers(workers), m_threads()
  // { }

  vpWriterExecutor(const std::vector<std::shared_ptr<vpWriterWorker>> &workers)
    : m_writer_workers(workers), m_threads()
  {
    for (const auto &worker : m_writer_workers) {
      m_threads.emplace_back(&vpWriterWorker::run, worker);
    }
  }

  void join()
  {
    for (auto &thread : m_threads) {
      thread.join();
    }
  }

private:
  // std::vector<std::reference_wrapper<std::shared_ptr<vpWriterWorker>>> m_writer_workers;
  std::vector<std::shared_ptr<vpWriterWorker>> m_writer_workers;
  std::vector<std::thread> m_threads;
};

#endif
