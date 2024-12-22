/**
  * @brief C++ class CheckpointJitterMonitor of MBSAFE
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

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include "Task.hpp"
#include "MeasureQuantity.hpp"
#include "CheckpointSequence.hpp"
#include "CheckpointMonitor.hpp"

namespace mbsafe
{

class CheckpointJitterMonitor : public CheckpointMonitor
{
public:
  CheckpointJitterMonitor() = delete;

  explicit CheckpointJitterMonitor(rclcpp::Node& node,
                                   CheckpointSequence& cpSeq,
                                   const CheckpointType cpId,
                                   const double minLimit,
                                   const double maxLimit,
                                   const int32_t debounceErrorMax = 0, const int32_t debounceRecoverMax = 0)
    : dt { "CpJitterMon " + std::to_string(cpId), minLimit, maxLimit, debounceErrorMax, debounceRecoverMax },
      node { node }, cpSeq { cpSeq }, cpId { cpId },
      dtMicro { static_cast<int64_t>(maxLimit * 1e6) }
  {
  }

  virtual ~CheckpointJitterMonitor()
  {
    watchdogTask.stop();
  }

  virtual void init(const CheckpointType cp)
  {
    (void)cp; // unused param
  }

  virtual void update(const CheckpointType cp)
  {
#ifndef MBNORT
    if (cp == cpId) {
      nowTime = rclcpp::Time(cpSeq.at(cpId).time);
      if (firstCall == true) {
        watchdogTask.start();
      } else {
        watchdogTask.triggerStep(dtMicro);
      }
    }
#endif
  }

  void log()
  {
    RCLCPP_INFO(node.get_logger(), "%s: health=%u dt=%5.3lfms min=%5.3lf max=%5.3lf avg=%5.3lf errcnt=%3lu",
                dt.health().name().c_str(), dt.health().health(), 1000.0 * dt.curValue(), 1000.0 * dt.minValue(), 1000.0 * dt.maxValue(),
                1000.0 * dt.avgValue(), dt.health().errorCount());
  }

  virtual Health& health()
  {
    return dt.health();
  }

private:
  MeasureQuantity<double> dt;
  rclcpp::Node& node;
  CheckpointSequence& cpSeq;
  const CheckpointType cpId;
  rclcpp::Time nowTime;
  rclcpp::Time oldTime;
  const int64_t dtMicro { 0L };
  mbsafe::Task<10000UL, 92> watchdogTask { std::bind(&CheckpointJitterMonitor::watchdogTimeout, this), dtMicro };
  bool firstCall { true };

  void watchdogTimeout()
  {
    if (firstCall) {
      firstCall = false;
    } else {
      rclcpp::Duration dur { nowTime - oldTime };
      if (dur.seconds() <= 0.0) {
        // no new message received -> watchdog timeout
        dur = node.now() - oldTime;
      }
      dt.update(dur.seconds());
    }
    oldTime = nowTime;
  }
};

}
