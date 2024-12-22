/**
  * @brief C++ class CheckpointGraph for checkpoint graph monitoring
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
#include "mbsafe/CheckpointGraphMonitor.hpp"

class CheckpointGraph
{
public:
  enum Checkpoint : mbsafe::CheckpointMonitor::CheckpointType
  {
    CameraAcquisition, CameraStart, CameraPublish, VisionStart, VisionPublish,
    LocateStart, LocatePublish, CtrlStart, CtrlPublish, RcStart,
    RcPublish
    // , BehaviorStart, BehaviorPublish, VisionSwitchCallback0, VisionSwitchCallback1,
    // VisionSwitchTestCallback,
    // SmcStart, SmcWrite, SmcSample, SmcWeight, SmcReduction, SmcRead, SmcPublish
  };
  static const Checkpoint lastCheckpoint { Checkpoint::RcPublish };
  static const mbsafe::CheckpointGraphMonitor::GraphType graph;
  static constexpr double maxDeadline { 24.0e-3 };
};

const mbsafe::CheckpointGraphMonitor::GraphType CheckpointGraph::graph
{{
  { Checkpoint::CameraAcquisition, { Checkpoint::CameraStart } },
  { Checkpoint::CameraStart, { Checkpoint::CameraPublish } },
  { Checkpoint::CameraPublish, { Checkpoint::CameraAcquisition, Checkpoint::VisionStart } },
  { Checkpoint::VisionStart, { Checkpoint::VisionPublish } },
  { Checkpoint::VisionPublish, { Checkpoint::CameraStart, Checkpoint::VisionStart, Checkpoint::LocateStart } },
  { Checkpoint::LocateStart, { Checkpoint::LocatePublish } },
  { Checkpoint::LocatePublish, { Checkpoint::LocateStart, Checkpoint::CtrlStart } }, // Checkpoint::BehaviorStart } },
  { Checkpoint::CtrlStart, { Checkpoint::CtrlPublish } },
  { Checkpoint::CtrlPublish, { Checkpoint::CtrlStart, Checkpoint::RcStart } },
  { Checkpoint::RcStart, { Checkpoint::RcPublish } },
  { Checkpoint::RcPublish, { Checkpoint::RcStart } }
  // ,{ Checkpoint::BehaviorStart, { Checkpoint::BehaviorPublish } },
  // { Checkpoint::BehaviorPublish, { Checkpoint::BehaviorStart } }
}};
