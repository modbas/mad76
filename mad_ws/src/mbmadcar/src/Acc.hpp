/**
  * @brief C++ class Acc for adaptive cruise control and passing maneuvers
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

#include <limits>
#include <iostream>

#ifdef MAD24
#include "mbmad/CarParameters24.hpp"
#else
#include "mbmad/CarParameters.hpp"
#endif

#include "mbmad/Spline.hpp"
#include "mbmadmsgs/msg/car_outputs_ext_list.hpp"


namespace mbmad {

class Acc
{
  public:
    Acc() noexcept
    {
    }

    Acc(const Acc& that) = delete;

    /**
     * @brief init initializes the ACC
     */
    void init()
    {
      initialized = true;
    }

    /**
     * @brief ACC step
     * @param vmax maximum speed
     * @param spline reference path
     * @param carOutputsExtList car outputs
     * @param selfV own speed
     * @param selfX own x position
     * @param selfEy own lateral deviation
     * @param selfKappa own curvature
     * @param leadDist distance to leading car
     * @param leadV speed of leading car
     * @param otherDist distance to other car
     * @param faultLateral lateral fault detected
     * @return manipulation signal pedals
     */
    void step(const float vmax, std::array<Spline, 2>& splines, const mbmadmsgs::msg::CarOutputsExtList& carOutputsExtList,
      const float selfV, const float selfX, const float selfEy, const float selfKappa, 
      float& vref, float& leadDist, float& leadV,
      float& otherDist, bool& faultLateral)
    {
      // return variables
      vref = 0.0F;
      leadDist = 0.0F;
      leadV = 0.0F;
      otherDist = 0.0F;
      faultLateral = false;

      if (initialized) {
        detectLeadingCar(vmax, splines.at(lane), carOutputsExtList, selfX, selfEy, selfKappa, leadDist, leadV, otherDist, faultLateral);

        // required distance to leading car
        float dmin = dminAnyCase + std::fabs(selfV) * Tt;
        dmin += (selfV - leadV) / amaxBrake;     
        // if (v > 0.0F && leadV > 0.0F) { // forward drive
        //   if (leadV < selfV) { // lead car is slower than ego car
        //     dmin += (selfV - leadV) / amaxBrake;     
        //   }
        // } else if (selfV < 0.0F && leadV < 0.0F) { // reverse drive
        //   if (leadV > selfV) { // lead car is slower than ego car
        //     dmin += (leadV - selfV) / amaxBrake;     
        //   }
        // } else { // collosion drive
        //   dmin += std::fabs(leadV + selfV) / amaxBrake;
        // }

        // FSM Lane Switching
        if (fsmLaneSwitchingState) {
          if (std::fabs(selfEy) < selfEyMax) { // ego car has reached new lane
            fsmLaneSwitchingState = false; // --> re-enable lane switching
          }
        }


        // FSM ACC
        if (fsmAccState) { // ACC active
          if (leadV > 0.0F) {
            vref = leadVScale * leadV;
          } else {
            vref = 0.1F;
          }
          if (vref > vmax) {
            vref = vmax;
          }
          if (leadDist < dminAnyCase) {
            vref = 0.0F; // safety halt
          }
          // switch lane
          if (!fsmLaneSwitchingState && !splines.at(1).breaks.empty() && !faultLateral
              && (otherDist > dminAnyCase || otherDist < passingMinDist)) {
            if (lane == 0U) {
              lane = 1U;
            } else {
              lane = 0U;
            }
            fsmLaneSwitchingState = true;
          }
          if (leadDist > acc2ccScale * p->size.at(0) + dmin) {
            fsmAccState = false;
          }
        } else { // ACC inactive (cruise control)
          vref = vmax;          
          if (!fsmLaneSwitchingState && !splines.at(1).breaks.empty() && !faultLateral) {
            if (lane == 1U && otherDist < passingMinDist) { // switch back lane after passing
              lane = 0U;
            } else if (lane == 0U && leadDist <= dmin + 0.5F * p->size.at(0) && leadDist > 0.0F && selfV > leadV) { // switch lane before ACC
              lane = 1U;
            }
            fsmLaneSwitchingState = true;
          }          
          if (selfV > leadV && leadDist <= dmin && leadDist >= cc2accScale * p->size.at(0)) {
            fsmAccState = true;
          }
        }
        

#ifdef _MAD_DEBUG
        if (p->carid == 0U) { 
          std::cout << "accState=" << fsmAccState << " laneSwitch=" << fsmLaneSwitchingState << " lane=" << static_cast<int>(lane) << " leadV=" << leadV << " vref=" << vref << std::endl;
        }
#endif
      }
    }

    /**
     * @brief getLane returns the lane
     * @return lane
     */ 
    uint8_t getLane() const noexcept
    {
      return lane;
    }

  private:
    CarParameters const * const p = CarParameters::p();
    const float leadEyMax = p->size.at(1) + 10e-3;
    const float selfKappaMax = 12.0F;
    const float selfEyMax = 0.5F * p->size.at(1);
    const float dminAnyCase = p->size.at(0) + 10e-3F;
    const float acc2ccScale = 2.0F;
    const float cc2accScale = -0.2F;
    const float Tt = 2.0F * p->Tt;
    const float amaxBrake = 0.3F * p->k / p->T;
    const float leadVScale = 0.98F;
    const float passingMinDist = -3.0F * p->size.at(0);
    bool initialized { false };
    bool fsmAccState { false }; // if true then ACC is active, if false then CC is active
    bool fsmLaneSwitchingState { false }; // if true then lane switching is active, if false then car is in lane
    uint8_t lane { 0U };
    
    /**
     * @brief ACC detect leading car
     * @param vmax maximum speed
     * @param spline reference path
     * @param carOutputsExtList car outputs
     * @param selfX own x position
     * @param selfEy own lateral deviation
     * @param selfKappa own curvature
     * @param leadDist distance to leading car
     * @param leadV speed of leading car
     * @param otherDist distance to other car
     * @param faultLateral lateral fault detected
     * @return manipulation signal pedals
     */  
    void detectLeadingCar(const float vmax, Spline& spline, const mbmadmsgs::msg::CarOutputsExtList& carOutputsExtList,
      const float selfX, const float selfEy, const float selfKappa, float& leadDist, float& leadV,
      float& otherDist, bool& faultLateral)
    {
      leadDist = std::numeric_limits<float>::infinity();
      leadV = 0.0F;
      otherDist = std::numeric_limits<float>::infinity();
      
      faultLateral = (std::fabs(selfKappa) > selfKappaMax || (std::fabs(selfEy) > selfEyMax && !fsmLaneSwitchingState));
      for (auto& car : carOutputsExtList.list) {
        if (car.carid != p->carid && car.prob > 0.0F) {
          std::array<float,2> s;
          std::array<float,2> sd;
          std::array<float,2> sdd;
          float dist { 0.0F };
          float x { 0.0F };
          int idx = spline.getNearest(car.s, x, dist);
          spline.interpolate(x, s, sd, sdd, idx);
          otherDist = x - selfX;
          if (otherDist < -0.5F * spline.breaks.back()) {
            otherDist += spline.breaks.back();
          } else if (otherDist > 0.5F * spline.breaks.back()) {
            otherDist -= spline.breaks.back();
          }
          if (vmax < 0.0F) { // ego car is in reverse drive
            otherDist = -otherDist;
          }
          const float psi = std::atan2(sd.at(1), sd.at(0));
          const float ey = -(car.s.at(0) - s.at(0)) * std::sin(psi) + (car.s.at(1) - s.at(1)) * std::cos(psi);
          const float eydiff = ey - selfEy;
#ifdef _MAD_DEBUG
          if (p->carid == 0U) {
            std::cout << "eydiff=" << eydiff << " ey=" << ey << " selfEy=" << selfEy << " selfKappa" << selfKappa << " otherDist=" 
              << otherDist << " faultLateral="  << faultLateral << std::endl;
          }
#endif
          if (std::fabs(eydiff) < leadEyMax || faultLateral) {
            if (otherDist >= -p->size.at(0) && otherDist < leadDist) {
              leadDist = otherDist;
              leadV = car.v;
            }
          }          
        }
      }
    }
};

}