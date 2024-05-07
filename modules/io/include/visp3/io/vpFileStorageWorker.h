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
 * Binary file worker to execute code inside runOnce():
 *   - pop data from the concurrent queue (thread-safe queue)
 *   - write data to a file in binary format
 */

/*!
  \file vpFileStorageWorker.h
  \brief See https://stackoverflow.com/a/37146523 for inspiration
  Binary file worker to execute code inside runOnce():
    - pop data from the concurrent queue (thread-safe queue)
    - write data to a file in binary format
*/

#ifndef vpFileStorageWorker_h
#define vpFileStorageWorker_h

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpWriterWorker.h>
#include <visp3/core/vpConcurrentQueue.h>

template <class T, class Container = std::deque<std::vector<T>>> class VISP_EXPORT vpFileStorageWorker : public vpWriterWorker
{
public:
  vpFileStorageWorker(const std::vector<std::reference_wrapper<vpConcurrentQueue<std::vector<T>, Container>>> &queues,
    const std::vector< std::string > &filenames, const std::vector<const char *> &ptr_header_vec,
    const std::vector<const char *> &header_size_vec, bool write_header, bool force_little_endian = false)
    : m_filenames(filenames), m_data_vec(), m_queues(queues), m_ptr_header_vec(ptr_header_vec),
    m_header_size_vec(header_size_vec), m_write_header(write_header), m_force_little_endian(force_little_endian)
  {
    assert(!m_queues.empty());
    assert(m_queues.size() == m_filenames.size());
    if (m_write_header) {
      assert(m_queues.size() == m_ptr_header_vec.size());
      assert(m_queues.size() == m_header_size_vec.size());
    }
  }

  vpFileStorageWorker(const std::reference_wrapper<vpConcurrentQueue<std::vector<T>, Container>> &queue,
    const std::string &filename, const char *ptr_header, size_t header_size, bool m_write_header,
    bool force_little_endian = false)
    : m_filenames(), m_data_vec(), m_queues(), m_ptr_header_vec(), m_header_size_vec(),
    m_force_little_endian(force_little_endian)
  {
    m_filenames.emplace_back(filename);
    m_queues.emplace_back(queue);
    if (m_write_header) {
      m_ptr_header_vec.emplace_back(ptr_header);
      m_header_size_vec.emplace_back(header_size);
    }
  }

  void init() override
  {
    m_data_vec.resize(m_queues.size());
    m_iter = 0;
  }

  // Thread main loop
  void run() override
  {
    init();

    while (runOnce());
  }

  // Code to be executed each time a new data is fed into a concurrent queue
  // Return false when abort is called
  bool runOnce() override
  {
    try {
      for (size_t i = 0; i < m_queues.size(); i++) {
        m_data_vec[i] = m_queues[i].get().pop();
        char filename[FILENAME_MAX];
        snprintf(filename, FILENAME_MAX, m_filenames[i].c_str(), m_iter);
        std::ofstream writer(filename, std::ios::out | std::ios::binary);

        if (m_write_header) {
          // Write data header
          // WARNING: header data must already be stored in little-endian format if endianness is needed
          writer.write(m_ptr_header_vec[i], m_header_size_vec[i]);
        }

        if (m_force_little_endian) {
          for (const auto &data : m_data_vec[i]) {
            vpIoTools::writeBinaryValueLE(writer, data);
          }
        }
        else {
          const std::vector<T> &ptr_data = m_data_vec[i];
          writer.write(reinterpret_cast<const char *>(&ptr_data[0]), ptr_data.size()*sizeof(T));
        }
      }

      m_iter++;
    }
    catch (typename vpConcurrentQueue<std::vector<T>, Container>::vpCancelled_t &) {
      return false;
    }

    return true;
  }

private:
  std::vector<std::string> m_filenames;
  std::vector<std::vector<T>> m_data_vec;
  std::vector< std::reference_wrapper<vpConcurrentQueue<std::vector<T>, Container>> > m_queues;
  std::vector<const char *> m_ptr_header_vec;
  std::vector<size_t> m_header_size_vec;
  bool m_write_header;
  bool m_force_little_endian;
  int m_iter;
};

#endif
