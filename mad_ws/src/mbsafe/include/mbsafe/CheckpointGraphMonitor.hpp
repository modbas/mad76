/**
  * @brief C++ class CheckpointGraphMonitor of MBSAFE
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

#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "SingleHealth.hpp"
#include "CheckpointMonitor.hpp"

namespace mbsafe
{

class CheckpointGraphMonitor : public CheckpointMonitor
{
public:
  using SuccessorsType = std::unordered_set<CheckpointType>;
  using GraphType = std::unordered_map<CheckpointType, SuccessorsType>;

  CheckpointGraphMonitor() = delete;

  explicit CheckpointGraphMonitor(rclcpp::Node& node,
                                  const GraphType& graph)
    :  node { node }, graph { graph }
  {
  }

  virtual ~CheckpointGraphMonitor()
  {
  }

  /**
   * @brief init resets current checkpoint
   * @param cp
   */
  virtual void init(const CheckpointType cp)
  {
    if (graph.find(cp) == graph.end()) {
      health_.update(false, "fatal error", "checkpoint " + std::to_string(cp) + " is not defined");
    } else {
      curCp = cp;
    }
  }

  virtual void update(const CheckpointType cp)
  {
    if (curCp == cpUndefined) {
      // initial state
      init(cp);
    } else {
      // check if cp is a valid successor of curCp
      const SuccessorsType& successors = graph.at(curCp);
      if (successors.find(cp) == successors.end()) {
        health_.update(false, "fatal error",
                       "checkpoint " + std::to_string(curCp) + " has no successor " + std::to_string(cp));
      } else {
        curCp = cp;
      }
    }
  }

  virtual Health& health()
  {
    return health_;
  }

private:
  SingleHealth health_ { "CpGraphMon" };
  rclcpp::Node& node;
  const GraphType graph;
  CheckpointType curCp { cpUndefined };
};

}
