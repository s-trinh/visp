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
 * Video worker to execute code inside runOnce():
 *   - pop data from the concurrent queue (thread-safe queue)
 *   - still images are supported (e.g. JPEG, PNG)
 *   - see vpVideoWriter class
 */

/*!
  \file vpVideoStorageWorker.h
  \brief See https://stackoverflow.com/a/37146523 for inspiration
  Video worker to execute code inside runOnce():
    - pop data from the concurrent queue (thread-safe queue)
    - still images are supported (e.g. JPEG, PNG)
    - see vpVideoWriter class
*/

#ifndef vpVideoStorageWorker_h
#define vpVideoStorageWorker_h

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpConcurrentQueue.h>
#include <visp3/core/vpWriterWorker.h>
#include <visp3/io/vpVideoWriter.h>

template <class T, class Container = std::deque<vpImage<T>>> class VISP_EXPORT vpVideoStorageWorker : public vpWriterWorker
{
public:
  vpVideoStorageWorker(const std::vector<std::reference_wrapper<vpConcurrentQueue<vpImage<T>, Container>>> &queues,
    const std::vector< std::string > &filenames)
    : m_filenames(filenames), m_images(), m_is_opened_vec(), m_queues(queues), m_writers()
  {
    assert(!m_queues.empty());
    assert(m_queues.size() == m_filenames.size());
  }

  vpVideoStorageWorker(const std::reference_wrapper<vpConcurrentQueue<vpImage<T>, Container>> &queue,
    const std::string &filenames)
    : m_filenames(), m_images(), m_is_opened_vec(), m_queues(), m_writers()
  {
    m_filenames.emplace_back(filenames);
    m_queues.emplace_back(queue);
  }

  void init() override
  {
    m_images.resize(m_queues.size());
    m_is_opened_vec.resize(m_queues.size());
    m_writers.resize(m_queues.size());

    for (size_t i = 0; i < m_filenames.size(); i++) {
      m_writers[i].setFileName(m_filenames[i]);
      m_is_opened_vec[i] = false;
    }
  }

  // Thread main loop
  void run() override
  {
    init();

    while (runOnce());
  }

  bool runOnce() override
  {
    try {
      for (size_t i = 0; i < m_queues.size(); i++) {
        m_images[i] = m_queues[i].get().pop();

        if (!m_is_opened_vec[i]) {
          m_writers[i].open(m_images[i]);
          m_is_opened_vec[i] = true;
        }

        m_writers[i].saveFrame(m_images[i]);
      }
    }
    catch (typename vpConcurrentQueue<vpImage<T>, Container>::vpCancelled_t &) {
      return false;
    }

    return true;
  }

private:
  std::vector<std::string> m_filenames;
  std::vector<vpImage<T>> m_images;
  std::vector<bool> m_is_opened_vec;
  std::vector< std::reference_wrapper<vpConcurrentQueue<vpImage<T>, Container>> > m_queues;
  std::vector<vpVideoWriter> m_writers;
};

#endif
