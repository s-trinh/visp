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
 * Single or multi-threaded executor, which will call the runOnce() method of the different input workers.
 */

/*!
  \file vpWriterExecutor.h
  \brief Single or multi-threaded executor, which will call the runOnce() method of the different input workers.
*/

#ifndef _vpWriterExecutor_h_
#define _vpWriterExecutor_h_

#include <visp3/core/vpConfig.h>
#include <thread>
#include <vector>
#include <visp3/core/vpConcurrentQueue.h>
#include <visp3/core/vpWriterWorker.h>

/*!
  \class vpWriterExecutor
*/

class VISP_EXPORT vpWriterExecutor
{
public:
  vpWriterExecutor(const std::shared_ptr<vpWriterWorker> &worker, bool single_thread = false)
    : m_writer_workers(), m_threads()
  {
    m_writer_workers.push_back(worker);
    m_threads.emplace_back(&vpWriterWorker::run, worker);
  }

  vpWriterExecutor(const std::vector<std::shared_ptr<vpWriterWorker>> &workers, bool single_thread = false)
    : m_single_thread(single_thread), m_writer_workers(workers), m_threads()
  {
    if (m_single_thread) {
      m_threads.emplace_back(&vpWriterExecutor::run, this);
    }
    else {
      for (const auto &worker : m_writer_workers) {
        m_threads.emplace_back(&vpWriterWorker::run, worker);
      }
    }
  }

  void join()
  {
    for (auto &thread : m_threads) {
      thread.join();
    }
  }

private:
  void run()
  {
    for (const auto &worker : m_writer_workers) {
      worker->init();
    }

    bool cancel = false;
    while (!cancel) {
      for (const auto &worker : m_writer_workers) {
        // bitwise OR assignemnt
        // https://stackoverflow.com/a/9021094
        cancel |= !worker->runOnce();
      }
    }
  }

  bool m_single_thread;
  std::vector<std::shared_ptr<vpWriterWorker>> m_writer_workers;
  std::vector<std::thread> m_threads;
};

#endif
