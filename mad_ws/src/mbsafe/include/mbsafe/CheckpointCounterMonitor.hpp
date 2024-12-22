/**
  * MODBAS
  * mbsafe
  * CheckpointCounterMonitor
  *
  * Copyright (C) 2019, Frank Traenkle, http://www.modbas.de
  */

#ifndef MBSAFE_CHECKPOINT_COUNTER_MONITOR_HPP_
#define MBSAFE_CHECKPOINT_COUNTER_MONITOR_HPP_

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include "MeasureQuantity.hpp"
#include "CheckpointSequence.hpp"
#include "CheckpointMonitor.hpp"

namespace mbsafe
{

class CheckpointCounterMonitor : public CheckpointMonitor
{
public:
  CheckpointCounterMonitor() = delete;

  explicit CheckpointCounterMonitor(rclcpp::Node& node,
                                   CheckpointSequence& cpSeq,
                                   const CheckpointType cpId)
    : dt { "CpCounterMon " + std::to_string(cpId), 1, 1 },
      node { node }, cpSeq { cpSeq }, cpId { cpId }
  {
  }

  virtual ~CheckpointCounterMonitor()
  {
  }

  virtual void init(const CheckpointType cp)
  {
    (void)cp; // unused param
  }

  virtual void update(const CheckpointType cp)
  {
    if (cp == cpId) {
      if (firstCall == true) {
        firstCall = false;
      } else {
        dt.update(static_cast<int64_t>(cpSeq.at(cp).seqctr) - static_cast<int64_t>(oldCounter));
      }
      oldCounter = cpSeq.at(cp).seqctr;
    }
  }

  void log()
  {
    RCLCPP_INFO(node.get_logger(), "%s: health=%u counterDiff=%03ld min=%03ld max=%03ld errcnt=%3lu",
                dt.health().name().c_str(), dt.health().health(), dt.curValue(), dt.minValue(),
                dt.maxValue(), dt.health().errorCount());
  }

  virtual Health& health()
  {
    return dt.health();
  }

private:
  MeasureQuantity<int64_t> dt;
  rclcpp::Node& node;
  CheckpointSequence& cpSeq;
  const CheckpointType cpId;
  uint64_t oldCounter { 0ULL };
  bool firstCall { true };
};

}

#endif
