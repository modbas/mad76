/**
  * @brief C++ template class Task of MBSAFE
  *    
  * Copyright (C) 2024, Frank Traenkle, Hochschule Heilbronn
  * 
  * This file is part of MAD.
  * MAD is free software: you can redistribute it and/or modify it under the terms 
  * of the GNU General Public License as published by the Free Software Foundation,
  * either version 3 of the License, or (at your option) any later version.
  * MAD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY 
  * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  * See the GNU General Public License for more details.
  * You should have received a copy of the GNU General Public License along with MAD.
  * If not, see <https://www.gnu.org/licenses/>.
  */

#pragma once

#include <pthread.h>
#include <limits.h>
#include <cstdint>
#include <functional>
#include <mutex>
#include <cassert>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include "Platform.hpp"

namespace mbsafe
{

/**
 * @brief The Task class implements a RT thread
 */
template <std::size_t StackSize, int Priority> class Task
{
public:
  using TaskFunction = std::function<void(void)>;

  Task(TaskFunction taskFunction, const int64_t periodMicro) noexcept
    : taskFunction { taskFunction }, periodMicro { periodMicro }
  {
    static_assert(Priority >= minPriority && Priority <= maxPriority, "RT priority must be in the range of 1 to 98. If Priority==0 then we start a non-RT task.");
    assert(periodMicro >= 0LL);
    assert(periodMicro < INT64_MAX / NSEC_PER_USEC);

    Platform::increaseTaskCount();
  }

  ~Task() noexcept
  {
    Platform::decreaseTaskCount();
  }

  bool start() noexcept
  {
    bool ret = true;
    int rc = 0;

    pthread_attr_t attr;

    if (pthread_attr_init(&attr) != 0) {
      ret = false;
      BOOST_LOG_TRIVIAL(error) << "pthread_attr_init() failed: (" << errno << ") " << strerror(errno);
    }
    if (ret && pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN + StackSize) != 0) {
      ret = false;
      BOOST_LOG_TRIVIAL(error) << "pthread_attr_setstacksize() failed: (" << errno << ") " << strerror(errno);
    }
    if (ret && (rc = pthread_create(&thread, &attr, threadHelper, this))) {
      ret = false;
      BOOST_LOG_TRIVIAL(error) << "pthread_create() failed: rc=" << rc << " (" << errno << ") " << strerror(errno);
    }
    if (ret) {
      BOOST_LOG_TRIVIAL(info) << "thread started with realtime priority " << Priority << " and period " << periodMicro << " µs" ;
    }

    return ret;
  }

  bool stop() noexcept
  {
    bool ret = true;

    if (running) {
      running = false;
      if (pthread_join(thread, nullptr) != 0) {
        ret = false;
        BOOST_LOG_TRIVIAL(error) << "pthread_join() failed: (" << errno << ") " << strerror(errno);
      }
      if (ret) {
         BOOST_LOG_TRIVIAL(info) << "thread stopped";
      }
    }

    return ret;
  }

  void triggerStep(const int64_t offsetMicro) noexcept
  {
    {
      std::lock_guard<std::mutex> lock(tsMutex);
      clock_gettime(CLOCK_MONOTONIC, &ts);
      ts.tv_nsec += offsetMicro * NSEC_PER_USEC;
      normalizeTimespec(ts);
    }
    pthread_kill(thread, SIGRTMIN); // trigger syncWithTime
  }

private:
  static constexpr int sched = SCHED_FIFO;
  static constexpr int minPriority = 0;
  static constexpr int maxPriority = 98;
  static constexpr int64_t NSEC_PER_SEC = 1000000000LL;
  static constexpr int64_t NSEC_PER_USEC = 1000LL;
  static constexpr int64_t USEC_PER_SEC = 1000000LL;
  TaskFunction taskFunction;
  const int64_t periodMicro { 0L };
  pthread_t thread;
  bool running { false };
  struct timespec ts;
  std::mutex tsMutex;

  bool prefaultStack() noexcept
  {
    volatile char buffer[StackSize];

    for (std::size_t idx = 0ULL; idx < StackSize; idx += static_cast<std::size_t>(sysconf(_SC_PAGESIZE))) {
      buffer[idx] += 1;
    }

    return true;
  }

  void* taskEntry() noexcept
  {
    Platform::setPrio(SCHED_FIFO, Priority);
    Platform::logPagefaultCount("caused by creating thread", ">=0", ">=0");
    Platform::prefaultStack<StackSize>();
    Platform::logPagefaultCount("caused by using thread stack", "0", "0");

    running = true;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    while (running && Platform::isRunning()) {
      syncWithTime();
      taskFunction();
    }

    BOOST_LOG_TRIVIAL(info) << "taskEntry exits";

    return nullptr;
  }

  void syncWithTime() noexcept
  {    
    if (periodMicro > 0LL) {
      // sync
      if (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == 0) {
        std::lock_guard<std::mutex> lock(tsMutex);
        ts.tv_nsec += periodMicro * NSEC_PER_USEC;
        normalizeTimespec(ts);
      }
    }
  }

  void normalizeTimespec(struct timespec& ts) noexcept
  {
    // Normalize the time structure to calculate the next shot
    while (ts.tv_nsec >= NSEC_PER_SEC) {
      ts.tv_nsec -= NSEC_PER_SEC;
      ++ts.tv_sec;
    }
  }

  static void* threadHelper(void* context) noexcept
  {
    return static_cast<Task<StackSize, Priority>*>(context)->taskEntry();
  }
};
}
