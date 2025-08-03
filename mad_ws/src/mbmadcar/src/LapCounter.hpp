/**
  * @brief C++ class LabCounter for lap counting
  *    
  * Copyright (C) 2025, Frank Traenkle, Hochschule Heilbronn
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
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include "mbmad/Spline.hpp"
#include "OperationModeFsm.hpp"

namespace mbmad {

class LapCounter
{
  public:
    LapCounter(rclcpp::Node& node) noexcept
      : node(node)
    {
    }

    LapCounter(const LapCounter& that) = delete;

    void reset() noexcept
    {
      lost = true;
      laptime = 9999.0F;
      avgspeed = 0.0F;          
      crashctr = 0U;  
      crashctrkm1 = 0U;        
    }

    void step(const Spline& spline, const uint8_t laneid, const float wx, const OperationModeFsm& opModeFsm) noexcept
    {
      if (lost) {
        if (std::holds_alternative<StateNormal>(opModeFsm.getState())) {
          lapctr = 0U;
          wxkm1 = wx;
          lost = false;
        }
      } else {
        if (std::holds_alternative<StateSafetyHalt>(opModeFsm.getState())) {
          lost = true;
          ++crashctr;
        } else {
          if (wx < maxDeltaX && wxkm1 > xekm1 - maxDeltaX) { // lap completed
            ++lapctr;
            if (lapctr > 1U && crashctr == crashctrkm1) { // >=2 laps without crash
              laptime = static_cast<float>((node.now() - tic).seconds());
              avgspeed = wxkm1 / laptime;
            }
            crashctrkm1 = crashctr;
            tic = node.now();
          } else if (laneid == laneidkm1 && std::fabs(wx - wxkm1) > maxDeltaX) { // shortcut taken (compare only works if there is no lane change)
            ++crashctr;
          }
          wxkm1 = wx;
        }
      }
      laneidkm1 = laneid; // store laneid and xe in case of lane changes          
      xekm1 = spline.breaks.back();          
    }

    uint32_t getLapCtr() const noexcept
    {
      return lapctr;
    }

    uint32_t getCrashCtr() const noexcept
    {
      return crashctr;
    }

    float getLapTime() const noexcept
    {
      return laptime;
    }

    float getAvgSpeed() const noexcept
    {
      return avgspeed;
    }

    float getCurrentLapTime() const noexcept
    {
      float time { 0.0F };
      if (lapctr >= 1U) {
        time = static_cast<float>((node.now() - tic).seconds());
      }
      return time;
    }

  private:
    rclcpp::Node& node;
    const float maxDeltaX { 0.2F }; // maximum distance between 2 sampling points
    bool lost { true };
    uint32_t lapctr { 0U };
    uint32_t crashctr { 0U };
    uint32_t crashctrkm1 { 0U };
    float wxkm1 { 0.0F };
    uint8_t laneidkm1 { 0U };
    float xekm1 { 0.0F };
    float laptime { 9999.0F };
    float avgspeed { 0.0F };
    rclcpp::Time tic;
};

}
