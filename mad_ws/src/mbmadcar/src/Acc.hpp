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

///
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
///

//#define _MAD_DEBUG

namespace mbmad {

struct DetectedCar {
  float dist = std::numeric_limits<float>::infinity();
  float v = 0.0F;
  bool sameLane = false;
};

class Acc
{
  public:
    Acc(rclcpp::Node& node) noexcept
    : node(node){}

    Acc(const Acc& that) = delete;

    void init()
    {
      if (!initialized) {
        rclcpp::QoS qos { rclcpp::KeepLast(1) };
        qos.best_effort().durability_volatile();
        pubDebugAcc = node.create_publisher<std_msgs::msg::Float32MultiArray>("accdebug", qos);
        RCLCPP_INFO(node.get_logger(), "ACC logger initialized");
      }
      initialized = true;
    }

    void step(const float vmax, std::array<Spline, 2>& splines, const mbmadmsgs::msg::CarOutputsExtList& carOutputsExtList,
      const float selfV, const float selfX, const float selfEy, const float selfKappa, float& vref)
    {
      // return variables
      vref = 0.0F;
      bool selfFaultLateral = false;

      if (initialized) {
        detectCars(vmax, splines.at(lane), carOutputsExtList, selfX, selfEy, selfKappa, selfFaultLateral);

        // required distance to leading car
        float dmin = dminAnyCase + std::fabs(selfV) * (Tt + reactTimeBuffer);   // distance car travels during deadtime and timebuffer
        dmin += std::pow(selfV - leadCar.v, 2) / (2.0F * amaxBrake);            // distance car travels during maximal deceleration // before: dmin += (selfV-leadCar.v) / amaxBreak

        // FSM Lane Switching
        if (fsmLaneSwitchingState) { // lane switching state or lead car is too close
          if (std::fabs(selfEy) < selfEyMax) { // ego car has reached new lane
            fsmLaneSwitchingState = false; // --> re-enable lane switching
          }
        }

        bool canSwitchLane =  !fsmLaneSwitchingState &&
                              !splines.at(1).breaks.empty() &&
                              !selfFaultLateral;

        // FSM ACC
        if (fsmAccState) { // ACC active
          vref = leadVScale * leadCar.v;

          // crawl speed
          if (leadCar.v <= 0.0F) {
            vref = crawlSpeed; // if lead car is not moving, then crawl speed
          }

          // saturation
          if (vref > vmax) {
            vref = vmax;
          }

          // safety halt
          if (leadCar.dist < dminAnyCase) {
            vref = 0.0F;
          }

          // switch lane

          bool shouldOvertake = lane == 0U && 
                                leadCar.dist > dminAnyCase && 
                                !(rearCar.v > vref && rearCar.dist > passingMinDist); // rearcar is not disturbing

          if (canSwitchLane && shouldOvertake) {
            lane = 1U;
            fsmLaneSwitchingState = true;
          }

          float accDeactivateDist = acc2ccScale * p->size.at(0) + dmin;

          if (leadCar.dist > accDeactivateDist || !leadCar.sameLane) {
            fsmAccState = false;
          }
        }
        else { // ACC inactive (cruise control)
          vref = vmax;

          // Determine if lane switching is allowed
          bool canSwitchBack= lane == 1U &&
                              (rearCar.dist < passingMinDist || rearCar.sameLane) &&
                              leadCar.dist > 2.0F*dmin;

          bool prepareAccActivation =  lane == 0U &&
	                              leadCar.dist <= (dmin + 2.0F * p->size.at(0)) &&
                                      selfV > leadCar.v &&
                                      !(rearCar.v > selfV && rearCar.dist > passingMinDist); // rearcar is not disturbing


          // lane switch
          if (canSwitchLane) {
            if (canSwitchBack) {
              lane = 0U;
              fsmLaneSwitchingState = true;
            } else if (prepareAccActivation) {
              lane = 1U;
              fsmLaneSwitchingState = true;
            }
          }
          bool shouldTurnAccOn =  selfV > leadCar.v &&
                                  leadCar.sameLane &&
                                  leadCar.dist <= dmin;

          if (shouldTurnAccOn) fsmAccState = true;
        }

        publishDebugInfo(selfFaultLateral);
      }
    }

    uint8_t getLane() const noexcept {
      return lane;
    }

 private:
    CarParameters const * const p = CarParameters::p();
    const float sameLaneEyMax = p->size.at(1) + 10e-3F;            // variable for maximum lateral offset for other car to be considered as being in the same lane // before: p->size.at(1) + 15e-3
    const float selfKappaMax = 12.0F;
    const float selfEyMax = 0.5F * p->size.at(1);
    const float dminAnyCase = p->size.at(0) + 30e-3F;            // 50 mm
    const float acc2ccScale = 2.0F;
    const float cc2accScale = -0.2F;
    const float Tt = 2.0F * p->Tt;
    const float reactTimeBuffer = 0.5F * Tt;                     // variable for more safety, to increase dmin
    const float amaxBrake = 0.3F * p->k / p->T;
    const float leadVScale = 0.98F;                               // before: 0.98F
    const float passingMinDist = -3.0F * p->size.at(0);
    const float crawlSpeed = 0.05F;                              // crawl speed for ACC // before: 0.1F
    bool initialized { false };
    bool fsmAccState { false };                                  // if true then ACC is active, if false then CC is inactive
    bool fsmLaneSwitchingState { false };                        // if true then lane switching is active, if false then car is in lane
    uint8_t lane { 0U };

    // car detection variables
    DetectedCar leadCar {std::numeric_limits<float>::infinity(), 0.0F, true}; // leading car
    DetectedCar rearCar {-std::numeric_limits<float>::infinity(), 0.0F, true}; // car behind ego car

    ///
    rclcpp::Node& node;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pubDebugAcc;
    ///

    /**
     * @brief ACC detect leading car
     * @param vmax maximum speed
     * @param spline reference path
     * @param carOutputsExtList car outputs
     * @param selfX own x position
     * @param selfEy own lateral deviation
     * @param selfKappa own curvature
     * @param selfFaultLateral lateral fault detected
     * @return manipulation signal pedals
     */

    void detectCars(const float vmax, Spline& spline, const mbmadmsgs::msg::CarOutputsExtList& carOutputsExtList,
      const float selfX, const float selfEy, const float selfKappa, bool& selfFaultLateral)
    {
      // Initialize output values
      float carDist = std::numeric_limits<float>::infinity();

      leadCar.dist = std::numeric_limits<float>::infinity();
      rearCar.dist = -std::numeric_limits<float>::infinity();

      std::vector<DetectedCar> leadCars;

      // Determine if there's a lateral driving fault
      selfFaultLateral = (std::fabs(selfKappa) > selfKappaMax) || (std::fabs(selfEy) > selfEyMax);

      for (const auto& car : carOutputsExtList.list) {
        // Skip self and invalid detections
        if (car.carid == p->carid || car.prob <= 0.0F) continue;

        // Find nearest point on spline to the other car
        float splineX = 0.0F, distance = 0.0F;
        int splineIdx = spline.getNearest(car.s, splineX, distance);

        std::array<float, 2> s, sd, sdd;
        spline.interpolate(splineX, s, sd, sdd, splineIdx);

        // Compute longitudinal distance along track
        carDist = splineX - selfX;
        const float trackLength = spline.breaks.back();

        // Handle wrap-around if track is circular
        if (carDist < -0.5F * trackLength) {
          carDist += trackLength;
        } else if (carDist > 0.5F * trackLength) {
          carDist -= trackLength;
        }

        // Reverse drive correction
        if (vmax < 0.0F) {
          carDist = -carDist;
        }

        // Compute lateral offset (ey) and  of other car relative to spline
        float psi = std::atan2(sd.at(1), sd.at(0));
        float ey = -(car.s.at(0) - s.at(0)) * std::sin(psi) +
                    (car.s.at(1) - s.at(1)) * std::cos(psi);

        float eyDiff = ey - selfEy; // lateral offset difference to ego car
        bool isInSameLane = (std::fabs(eyDiff) < sameLaneEyMax); // || selfFaultLateral;

        bool isAhead = (carDist > 0.0F);
        bool isBehind = (carDist < 0.0F) && (carDist > rearCar.dist);

        if(isAhead){
          leadCars.push_back(DetectedCar{carDist, car.v, isInSameLane});
        }

        if (isBehind) {
          rearCar = {carDist, car.v, isInSameLane};
        }
      }

      if (!leadCars.empty()) {
        // --- Sort leadCars by distance (ascending) ---
        std::sort(leadCars.begin(), leadCars.end(),[](const DetectedCar& a, const DetectedCar& b) {return a.dist < b.dist;});
        
        leadCar = leadCars[0]; // default: pick closest

        if (leadCars.size() >= 2) {
          const float closeDistanceThreshhold = 2.0F * p->size.at(0);
          bool closePair = std::fabs(leadCars[0].dist - leadCars[1].dist) < closeDistanceThreshhold;
          bool differentLanes = (leadCars[0].sameLane != leadCars[1].sameLane);

          if (closePair && differentLanes) {
            // Pick the one in the egos lane
            if (leadCars[0].sameLane) {
              leadCar = leadCars[0];
            } else if (leadCars[1].sameLane) {
              leadCar = leadCars[1];
            }
          }
        }
      }
    }

    void publishDebugInfo(bool faultLateral)
    {
      std_msgs::msg::Float32MultiArray msg;
      msg.data = {
        static_cast<float>(fsmAccState),
        static_cast<float>(fsmLaneSwitchingState),
        static_cast<float>(lane),
        static_cast<float>(leadCar.sameLane),
        leadCar.v,
        leadCar.dist,
	static_cast<float>(faultLateral)
      };
      pubDebugAcc->publish(msg);
    }

};
}
