/**
  * @brief C++ class Car for object tracking
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

#include <array>
#include <cstdint>
#include <string>
#include "mbmad/SgDiff.hpp"

#ifdef MAD24
#include "mbmad/CarParameters24.hpp"
#else
#include "mbmad/CarParameters.hpp"
#endif

#include "mbmad/Vector.hpp"
#include "mbmadmsgs/msg/car_outputs.hpp"
#include "mbmadmsgs/msg/car_outputs_ext.hpp"

namespace mbmad {

class Car
{
public:
  uint32_t diagErrCnt { 0U };
  std::string diagErrMsg;

  Car(mbmadmsgs::msg::CarOutputs& msg)
  {
    msgExt.carid = msg.carid;
  }

  void update(const uint64_t newSeqctr, mbmadmsgs::msg::CarOutputs& msg, rclcpp::Time& camTime)
  {
    msgExt.s = msg.s;
    msgExt.psi = msg.psi;
    if (newSeqctr != seqctr + 1ULL && !firstCall) {
      const float dt = (camTime - timekm1).seconds();  
      diagErrMsg = "message lost car " + std::to_string(msg.carid) + " seqctr=" + std::to_string(newSeqctr) 
        + " lostmessages=" + std::to_string(newSeqctr - seqctr) + " dt=" + std::to_string(dt) + " v=" + std::to_string(msgExt.v);
      ++diagErrCnt;
      degrade();      
    } else {
      ingrade();
    }
    Vector<float> vv { 0.0F, 0.0F };  
    for (std::size_t idx = 0U; idx < sgdiff.size(); ++idx) {
      vv.s[idx] = sgdiff[idx].filter(msg.s[idx]) / CarParameters::p()->Tva;
    }
    float v = vv.abs();
    msgExt.beta = Utils::normalizeRad(std::atan2(vv.s[1], vv.s[0]) - msg.psi);
    if (std::fabs(msgExt.beta) > Utils::pi * 0.5F) {
      v = -v;
    }

    if (std::fabs(v) > vValidMax) {
      diagErrMsg = "invalid speed car=" + std::to_string(msg.carid) + " v=" + std::to_string(v);
      ++diagErrCnt;
      
    // } else if (!(msgExt.prob < 1.0F) &&
    //             std::fabs(v - msgExt.v) > aValidMax * dt) {
    //   diagErrMsg = "invalid acceleration car=" + std::to_string(msg.carid) + " v=" + std::to_string(v) + " vkm1=" + std::to_string(msgExt.v);
    //   ++diagErrCnt;
    //   msgExt.prob -= probDecay;
    //   if (msgExt.prob < 0.0F) {
    //     msgExt.prob = 0.0F;
    //   }
    } else {
      msgExt.v = v;
    }
    seqctr = newSeqctr;          
    timekm1 = camTime;
    firstCall = false;    
  }

  /**
   * @brief verify if car has been updated in current time step
   */
  void updateVerify(const uint64_t newSeqctr)
  {
    if (newSeqctr > 0ULL && newSeqctr != seqctr) {
      degrade();
    } 
  }

  mbmadmsgs::msg::CarOutputsExt& message()
  {
    return msgExt;
  }

private:
  const float vValidMax { 3.0F }; // maximum valid speed [ m/s ]
  const float aValidMax { 20.0F }; // maximum valid acceleration [ m/s^2 ]
  uint64_t seqctr { 0ULL };
  mbmadmsgs::msg::CarOutputsExt msgExt;
  const float probDecay { 0.1F };
  rclcpp::Time timekm1;
  bool firstCall { true };
  
  void degrade()
  {
    if (msgExt.prob > 0.0F) {
      msgExt.prob -= probDecay;
      if (msgExt.prob < 0.0F) {
        msgExt.prob = 0.0F;
      }
    }
    for (std::size_t idx = 0U; idx < sgdiff.size(); ++idx) {
      sgdiff[idx].reset();
    }
  }

  void ingrade()
  {
    if (msgExt.prob < 1.0F) {
      msgExt.prob += probDecay;
      if (msgExt.prob > 1.0F) {
        msgExt.prob = 1.0F;
      }
    }
  } 


#ifdef MAD24
  std::array<SgDiff, 2> sgdiff { SgDiff(), SgDiff() };  
#else
  std::array<SgDiff, 2> sgdiff { SgDiff(2U), SgDiff(2U) };
#endif
};

}