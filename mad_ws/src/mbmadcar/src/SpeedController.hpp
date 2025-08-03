/**
  * @brief C++ class SpeedController for speed control as part of motion control
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

#include <std_msgs/msg/float32_multi_array.hpp>
#ifdef MAD24
#include "mbmad/CarParameters24.hpp"
#else
#include "SmithPredictor.hpp"
#include "mbmad/CarParameters.hpp"
#endif

#include "OperationModeFsm.hpp"

namespace mbmad {

class SpeedController
{
  public:
    SpeedController(rclcpp::Node& node) noexcept
    : node(node)
    {
      rclcpp::QoS qos { rclcpp::KeepLast(1) };
      qos.best_effort().durability_volatile();
      pubDebug = node.create_publisher<std_msgs::msg::Float32MultiArray>("speeddebug", qos);   
    }

    SpeedController(const SpeedController& that) = delete;

    /**
     * @brief execute one single control step
     * @param[in] vref reference speed
     * @param[in] v current speed
     * @param[in] Ts sampling time
     * @return manipulation signal pedals
     */
    float step(const OperationModeFsm& opModeFsm, const float vref, const float v)
    {
      float u { 0.0F }; // manipulation signal

      if (std::holds_alternative<StateNormal>(opModeFsm.getState())) {
        float e { vref - v }; // control deviation
#ifndef MAD24            
        const float esmith = smithPredictor.step(ukm1); // Smith predictor
        e -= esmith;
#endif
        if (!sat) {
          ui += dt * e / Ti; // integral part
        }
        u = kp * (e + ui);               
        // saturation
        sat = false;
        if (vref >= 0.0F) {
          if (u > umax) {
            u = umax;
            sat = true;
          } else if (u < 0.0F) {
            u = 0.0F;
            ui = 0.0F; // reset for switching direction
            sat = true;
          }
        } else {
          if (u < umin) {
            u = umin;
            sat = true;
          } else if (u > 0.0F) {
            u = 0.0F;
            ui = 0.0F; // reset for switching direction
            sat = true;
          }
        }  
#ifndef MAD24
        ukm1 = u; // store for Smith predictor
#endif
        // compensate deadzone
        if (u >= 0.0F) {
          u += frictionPos;
        } else if (u < 0.0F) {
          u += frictionNeg;
        }        
        // debug signals for rqt
        std_msgs::msg::Float32MultiArray msg;
        msg.data = { vref, v, u, ui, e };
        pubDebug->publish(msg);
      } else if (std::holds_alternative<StateDegraded>(opModeFsm.getState())) {
        // keep integral part, omit proportional part
        u = kp * ui;
      } else {
        // safety halt
        u = 0.0F;
        ui = 0.0F;
      }
      
      return u;
    }

  private:
    CarParameters const * const p { CarParameters::p() };
    rclcpp::Node& node;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pubDebug;
    const float frictionPos { p->uFrictionKd0 }; // positive friction
    const float frictionNeg { -p->uFrictionKd0 }; // negative friction
    const float umax { p->uMax - p->uFrictionKd0 }; // maximum manipulation signal
    const float umin { -p->uMax + p->uFrictionKd0 }; // minimum manipulation signal
    const float dt { p->Tva }; // sampling time
#ifdef MAD24
    const float Ti { 247e-3F }; // integral time
    const float kp { 0.344F }; // gain  
#else
    const float Tw { 200e-3F }; // time constant of closed loop
    const float Ti { p->T }; // integral time (dynamic compensation)
    const float kp { p->T / (p->k * Tw) }; // gain    
    SmithPredictor smithPredictor { static_cast<uint32_t>(p->uTt / dt), p->k * (1.0F - std::exp(-dt / p->T)), std::exp(-dt / p->T) }; // Smith predictor
    float ukm1 { 0.0F }; // previous control signal for Smith predictor
#endif
    bool sat { false }; // output saturation detected
    float ui { 0.0F }; // integral part
};

}