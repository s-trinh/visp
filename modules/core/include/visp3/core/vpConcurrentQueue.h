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
 * C++ std::queue with mutex for concurrent IO operations.
 */

/*!
  \file vpConcurrentQueue.h
  \brief C++ std::queue with mutex for concurrent IO operations, see https://stackoverflow.com/a/37146523.
*/

#ifndef vpConcurrentQueue_h
#define vpConcurrentQueue_h

#include <queue>
#include <condition_variable>
#include <visp3/core/vpConfig.h>

template <class T, class Container = std::deque<T>> class VISP_EXPORT vpConcurrentQueue
{
public:
  struct vpCancelled_t
  { };

  vpConcurrentQueue()
    : m_cancelled(false), m_cond(), /*m_data(),*/ m_maxQueueSize(1024), m_mutex(), m_queue()
  { }

  void cancel()
  {

  }

  // Push the data into the queue (FIFO)
  void push(const T &data)
  {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_queue.push(data);

    // Pop extra images in the queue
    while (m_queue.size() > m_maxQueueSize) {
      m_queue.pop();
    }

    m_cond.notify_one();
  }

  // Pop the image to save from the queue (FIFO)
  T &pop()
  {
    std::unique_lock<std::mutex> lock(m_mutex);

    while (m_queue.empty()) {
      if (m_cancelled) {
        throw vpCancelled_t();
      }

      m_cond.wait(lock);

      if (m_cancelled) {
        throw vpCancelled_t();
      }
    }

    // TODO:
    // m_data = m_queue.front();
    // m_queue.pop();

    // return m_data;

    T &data = m_queue.front();
    m_queue.pop();

    return data;
  }

  void setMaxQueueSize(size_t max_queue_size)
  {
    m_maxQueueSize = max_queue_size;
  }

private:
  bool m_cancelled;
  std::condition_variable m_cond;
  // T m_data;
  size_t m_maxQueueSize;
  std::mutex m_mutex;
  std::queue<T> m_queue;
};

#endif
