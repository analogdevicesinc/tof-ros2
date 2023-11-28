/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef SAFEDATAACCESS_H
#define SAFEDATAACCESS_H

#include <condition_variable>
#include <list>
#include <mutex>
#include <vector>
#define VECSIZE 10

template <class T>
class SafeDataAccess
{
public:
  SafeDataAccess() : m_ready(false), m_currentIndex(0) {}
  ~SafeDataAccess() = default;

  /**
   * @brief Add element in vector. Just first 10 elements counts because
   * the vector is has circular access.
   */
  void populateData(T element) { m_vector.push_back(element); }

  /**
   * @brief get next element of the vector.
   */
  T getNextElement()
  {
    // iterate 10 items in vector and if currentindex is 9 then
    // go to the element at pozition zero
    if ((m_currentIndex + 1) % VECSIZE >= 0 && (m_currentIndex + 1) % VECSIZE < VECSIZE) {
      return m_vector[(m_currentIndex + 1) % VECSIZE];
    }
    return nullptr;
  }

  /**
   * @brief Call this function when you want to add elements in vector
   * It set a value to false and the listen threads can wait until an
   * element is added to vector.
   */
  void setReadytoStart() { m_ready = false; }

  /**
   * @brief Add element in vector. This function is called by thread that put
   * rezults of T elements in vector and after the element was added to vector
   * notify the consumer threads.
   */
  void addElement(T element)
  {
    m_ready = false;
    std::unique_lock<std::mutex> lock(m_mutex);
    m_vector[(m_currentIndex + 1) % VECSIZE] = element;
    m_currentIndex = (m_currentIndex + 1) % VECSIZE;
    m_ready = true;
    lock.unlock();
    m_cv.notify_all();
  }

  /**
   * @brief Get current element of the vector where the result was added
   * This function is called by consumers threads.
   */
  T getCurrentElement()
  {
    std::unique_lock<std::mutex> lock(m_mutex);
    while (!m_ready) m_cv.wait(lock);
    return m_vector[m_currentIndex];
  }

private:
  std::vector<T> m_vector;
  std::mutex m_mutex;
  std::condition_variable m_cv;
  bool m_ready;
  int m_currentIndex;
};

#endif  // SAFEDATAACCESS_H
