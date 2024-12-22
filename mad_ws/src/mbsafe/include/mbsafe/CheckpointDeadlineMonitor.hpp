/**
  * @brief C++ class CheckpointDeadlineMonitor of MBSAFE
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
#include "MeasureQuantity.hpp"
#include "CheckpointSequence.hpp"
#include "CheckpointMonitor.hpp"

namespace mbsafe
{

class CheckpointDeadlineMonitor : public CheckpointMonitor
{
public:
  CheckpointDeadlineMonitor() = delete;

  explicit CheckpointDeadlineMonitor(rclcpp::Node& node,
                                     CheckpointSequence& cpSeq,
                                     const CheckpointType ticId, const CheckpointType tacId,
                                     const double minLimit, const double maxLimit,
                                     const int32_t debounceErrorMax = 0, const int32_t debounceRecoverMax = 0)
    : dt { "CpDeadlineMon " + std::to_string(ticId) + "->" + std::to_string(tacId), minLimit, maxLimit,
           debounceErrorMax, debounceRecoverMax },
      node { node }, cpSeq { cpSeq }, ticId { ticId }, tacId { tacId }
  {
  }

  virtual ~CheckpointDeadlineMonitor()
  {
  }

  virtual void init(const CheckpointType cp)
  {
    (void)cp; // unused param
  }

  virtual void update(const CheckpointType cp)
  {
#ifndef MBNORT
    if (cp == tacId) {
      const rclcpp::Duration dur =
          rclcpp::Time(cpSeq.at(tacId).time) - cpSeq.at(ticId).time;
      const double dtVal { dur.seconds() };
      dt.update(dtVal);
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
  const uint64_t ticId;
  const uint64_t tacId;
};

}
