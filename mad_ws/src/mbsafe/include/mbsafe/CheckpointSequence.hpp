/**
  * @brief C++ class CheckpointSequence of MBSAFE
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
#include <list>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "mbsafemsgs/msg/checkpoint_sequence.hpp"
#include "CheckpointMonitor.hpp"
#include "CombinedHealth.hpp"
#include "SingleHealth.hpp"
#include "mbsafe-tp.hpp"

namespace mbsafe
{

class CheckpointSequence
{
public:
  CheckpointSequence() = delete;

  explicit CheckpointSequence(rclcpp::Node& node, const CheckpointMonitor::CheckpointType maxCpId)
    : health_ { std::string(node.get_name()) + " CheckpointSequence" }, node { node }
  {
    health_.registerSubHealth(healthMain);
    init(maxCpId);
  }

  void init(const CheckpointMonitor::CheckpointType maxCpId)
  {
    msg_ = mbsafemsgs::msg::CheckpointSequence();
    msg_.cpseq.resize(maxCpId+1ULL);
    msg_.cur_cpid = CheckpointMonitor::cpUndefined;
  }

  void init(mbsafemsgs::msg::CheckpointSequence& msgArg)
  {
    if (msgArg.cpseq.size() == msg_.cpseq.size()) {
      msg_ = msgArg;

      // init registered monitors
      for (auto monitor : monitorList)
      {
        monitor->init(msg_.cur_cpid);
      }
    } else {
      healthMain.update(false, "fatal error: invalid size of received checkpoint sequence",
                        "size is " + std::to_string(msgArg.cpseq.size()) +
                        + " but is expected to be " + std::to_string(msg_.cpseq.size()));
    }
  }

  void update(const CheckpointMonitor::CheckpointType id, const uint64_t seqctr,
              const rclcpp::Time& time)
  {
    if (id < msg_.cpseq.size()) {
      uint32_t uint32ArrayArg[8] = { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };
      tracepoint(mbsafe, cp, seqctr, id, uint32ArrayArg);
      mbsafemsgs::msg::Checkpoint cpMsg;
      cpMsg.nodename = node.get_name();
      cpMsg.seqctr = seqctr;
      cpMsg.time = time;
      msg_.cpseq.at(id) = cpMsg;
      msg_.cur_cpid = id;

      // update registered monitors
      for (auto monitor : monitorList)
      {
        monitor->update(id);
      }
    } else {
      healthMain.update(false, "fatal error: invalid checkpoint id",
                        "new checkpoint id " + std::to_string(id) +
                        + " exceeds checkpoint size " + std::to_string(msg_.cpseq.size()));
    }
  }

  void update(const CheckpointMonitor::CheckpointType id, const uint64_t seqctr)
  {
    update(id, seqctr, node.now());
  }

  void update(const CheckpointMonitor::CheckpointType id)
  {
    if (msg_.cur_cpid < msg_.cpseq.size()) {
      update(id, msg_.cpseq.at(msg_.cur_cpid).seqctr);
    } else {
      healthMain.update(false, "fatal error: invalid id of last checkpoint",
                        "last checkpoint id " + std::to_string(msg_.cur_cpid) +
                        + " exceeds checkpoint size " + std::to_string(msg_.cpseq.size())
                        + ", new id would be " + std::to_string(id));
    }
  }

  mbsafemsgs::msg::CheckpointSequence& message()
  {
    msg_.health = health_.health();
    return msg_;
  }

  mbsafemsgs::msg::Checkpoint& at(const CheckpointMonitor::CheckpointType id)
  {
    return msg_.cpseq.at(id);
  }

  void registerMonitor(CheckpointMonitor& monitor)
  {
    monitorList.push_back(&monitor);
    health_.registerSubHealth(monitor.health());
  }

  bool empty()
  {
    return (msg_.cur_cpid == CheckpointMonitor::cpUndefined);
  }

  CombinedHealth& health()
  {
    return health_;
  }

private:
  SingleHealth healthMain { "MBSAFE main system" };
  CombinedHealth health_;
  rclcpp::Node& node;
  mbsafemsgs::msg::CheckpointSequence msg_;
  std::list<CheckpointMonitor*> monitorList;
};

}
