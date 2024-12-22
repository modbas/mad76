/**
  * @brief C++ class Platform implementation of MBSAFE
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

#include "mbsafe/Platform.hpp"

namespace mbsafe {

uint32_t Platform::taskCount = 0U;
bool Platform::initSyncBarrier = true;
pthread_barrier_t Platform::barrier;
bool Platform::running = false;

bool Platform::init(const size_t processMemorySize,
                    const bool realtime,
                    const bool initSyncBarrier) noexcept
{
  if (running) {
    BOOST_LOG_TRIVIAL(info) << "mbsafe::Platform is already running. Re-initialization is aborted.";
    return false;
  }
  BOOST_LOG_TRIVIAL(info)  << "mbsafe::Platform, Copyright (C) 2019, Frank Traenkle, http://www.modbas.de";
  if (checkPreemptRt()) {
    BOOST_LOG_TRIVIAL(info) << "Linux is running in realtime";
  } else {
    if (realtime) {
      BOOST_LOG_TRIVIAL(error) << "Linux is not running in realtime";
      return false;
    } else {
      BOOST_LOG_TRIVIAL(info) << "Linux is not running in realtime";
    }
  }
  if (configureMallocBehavior()) {
    logPagefaultCount("caused by mlockall()", ">=0", ">=0");
  } else {
    BOOST_LOG_TRIVIAL(error) << "Configuration of malloc behavior failed. You may have to set memlock to unlimited in /etc/security/limits.conf";
    return false;
  }
  if (prefaultProcessMemory(processMemorySize)) {
    logPagefaultCount("caused by malloc() and touch", ">=0", ">=0");
  } else {
    BOOST_LOG_TRIVIAL(error) << "Reservation of process memory by malloc failed";
    return false;
  }
  if (prefaultProcessMemory(processMemorySize)) {
    logPagefaultCount("caused by 2nd malloc() and touch", "0", "0");
    BOOST_LOG_TRIVIAL(info) << "look at output of ps -leyf and see that RSS is now about " << processMemorySize / (1024U*1024U) << " MB";
  } else {
    BOOST_LOG_TRIVIAL(error) << "2nd reservation of process memory by malloc failed";
    return false;
  }
//  if (signal(SIGINT, Platform::sigintCallbackHandler) == SIG_ERR) {
//    std::cerr << "signal(SIGINT, Platform::sigintCallbackHandler) failed: : (" << errno << ") " << strerror(errno);
//    return false;
//  }
  if (signal(SIGRTMIN, Platform::sigusr1CallbackHandler) == SIG_ERR) {
    BOOST_LOG_TRIVIAL(error) << "signal(SIGRTMIN, Platform::sigintCallbackHandler) failed: : (" << errno << ") " << strerror(errno);
    return false;
  }
  running = true;
  Platform::initSyncBarrier = initSyncBarrier;
  BOOST_LOG_TRIVIAL(info) << "platform is started and running with realtime=" << realtime << " and initSyncBarrier=" << initSyncBarrier;

  return true;
}

void Platform::increaseTaskCount() noexcept
{
  ++taskCount;
  if (taskCount > 1) {
    if (pthread_barrier_destroy(&barrier) != 0) {
      BOOST_LOG_TRIVIAL(error) << "pthread_barrier_destroy() failed: : (" << errno << ") " << strerror(errno);
    }
  }
  if (pthread_barrier_init(&barrier, nullptr, taskCount) != 0) {
    BOOST_LOG_TRIVIAL(error) << "pthread_barrier_init() failed: : (" << errno << ") " << strerror(errno);
  }
}

void Platform::decreaseTaskCount() noexcept
{
  --taskCount;
  if (taskCount == 0) {
    if (pthread_barrier_destroy(&barrier) == 0) {
      BOOST_LOG_TRIVIAL(info) << "init sync barrier destroyed";
    } else {
      BOOST_LOG_TRIVIAL(error) << "pthread_barrier_init() failed: : (" << errno << ") " << strerror(errno);
    }
  }
}

void Platform::logPagefaultCount(const std::string& message, const std::string& allowedMaj, const std::string& allowedMin) noexcept
{
  static long last_majflt = 0;
  static long last_minflt = 0;
  struct rusage usage;

  getrusage(RUSAGE_SELF, &usage);
  BOOST_LOG_TRIVIAL(info) << "pagefaults " << message
                          << ". major: " << usage.ru_majflt - last_majflt << " (allowed " << allowedMaj << ") "
                          << " minor: " << usage.ru_minflt - last_minflt << " (allowed " << allowedMin << ") ";
  last_majflt = usage.ru_majflt;
  last_minflt = usage.ru_minflt;
}

bool Platform::setPrio(const int sched, const int priority) noexcept
{
  const int minPriority = 0;
  const int maxPriority = 98;

  if (sched != SCHED_FIFO && sched != SCHED_RR) {
    BOOST_LOG_TRIVIAL(error) << "only SCHED_FIFO or SCHED_RR are supported";
    return false;
  }
  if (priority < minPriority || priority > maxPriority) {
    BOOST_LOG_TRIVIAL(error) << "prority " << priority << " exceeds valid range ["
              << minPriority << "," << maxPriority << "]";
    return false;
  }

  if (priority > 0) {
    // if Priority is set to 0 we have a non-realtime task
    struct sched_param param;
    param.sched_priority = priority;
    if (sched_setscheduler(0, sched, &param) != 0) {
      BOOST_LOG_TRIVIAL(error) << "sched_setscheduler(0, " << priority << ", " << sched
                << ") failed: (" << errno << ") " << strerror(errno);
      return false;
    }
  }

  return true;
}

void Platform::sigintCallbackHandler(const int) noexcept
{
  exit();
}

void Platform::sigusr1CallbackHandler(const int) noexcept
{
}

bool Platform::checkPreemptRt() noexcept
{
  struct utsname u;
  bool crit1 = false;
  bool crit2 = false;
  FILE *fd = nullptr;
  uname(&u);
  crit1 = (strcasestr(u.version, "PREEMPT RT") != nullptr) | (strcasestr(u.version, "PREEMPT_RT") != nullptr);
  if ((fd = fopen("/sys/kernel/realtime", "r")) != nullptr) {
    int flag = 0;
    crit2 = ((fscanf(fd, "%d", &flag) == 1) && (flag == 1));
    fclose(fd);
  }
  return (crit1 && crit2);
}

bool Platform::configureMallocBehavior() noexcept
{
  bool ret = true;

  logPagefaultCount("inital count", ">=0", ">=0");
  if (ret && mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    //    ret = false;
    BOOST_LOG_TRIVIAL(info) << "mlockall(MCL_CURRENT | MCL_FUTURE) failed: (" << errno << ") " << strerror(errno);
  }
  if (ret && mallopt(M_TRIM_THRESHOLD, -1) == 0) {
    ret = false;
    BOOST_LOG_TRIVIAL(error) << "mallopt(M_TRIM_THRESHOLD, -1) failed";
  }
  if (ret && mallopt(M_MMAP_MAX, 0) == 0) {
    ret = false;
    BOOST_LOG_TRIVIAL(error) << "mallopt(M_MMAP_MAX, 0) failed";
  }

  return ret;
}

bool Platform::prefaultProcessMemory(const size_t processMemorySize) noexcept
{
  bool ret = true;
  char* buffer = static_cast<char*>(malloc(processMemorySize));
  if (buffer != nullptr) {
    for (size_t idx = 0U; idx < processMemorySize; idx += static_cast<size_t>(sysconf(_SC_PAGESIZE))) {
      buffer[idx] = 0;
    }
    free(buffer);
    ret = true;
  }
  return ret;
}

}
