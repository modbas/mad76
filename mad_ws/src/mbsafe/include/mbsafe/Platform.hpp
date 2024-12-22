/**
  * @brief C++ class Platform of MBSAFE
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

#include <string.h>
#include <stdio.h>
#include <sys/utsname.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>
#include <malloc.h>
#include <pthread.h>
#include <csignal>
#include <cstdint>
#include <string>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mbsafe
{

/**
 * @brief The Platform class provides general Linux routines.
 */
class Platform
{
public:
  static bool init(const size_t processMemorySize,
                   const bool realtime = true,
                   const bool initSyncBarrier = true) noexcept;

  static bool exit() noexcept
  {
    BOOST_LOG_TRIVIAL(info) << "Process terminated by user. Will stop all tasks and exit now ...";
    running = false;
    return true;
  }

  static bool isRunning() noexcept
  {
    return running && rclcpp::ok();
  }

  static void increaseTaskCount() noexcept;
  static void decreaseTaskCount() noexcept;

  static void waitForBarrier() noexcept
  {
    pthread_barrier_wait(&barrier);
  }

  static void logPagefaultCount(const std::string& message, const std::string& allowedMaj, const std::string& allowedMin) noexcept;
  static bool setPrio(const int sched, const int priority) noexcept;

  template <size_t StackSize> static bool prefaultStack() noexcept
  {
    volatile char buffer[StackSize];

    for (size_t idx = 0U; idx < StackSize; idx += static_cast<size_t>(sysconf(_SC_PAGESIZE))) {
      buffer[idx] += 1;
    }

    return true;
  }


private:
  static uint32_t taskCount;
  static bool running;
  static bool initSyncBarrier;
  static pthread_barrier_t barrier;

  static bool prefaultStack(const size_t stackSize) noexcept;
  static void sigintCallbackHandler(const int) noexcept;
  static void sigusr1CallbackHandler(const int) noexcept;
  static bool initLogging() noexcept;
  static bool checkPreemptRt() noexcept;
  static bool configureMallocBehavior() noexcept;
  static bool prefaultProcessMemory(const size_t processMemorySize) noexcept;
};
}
