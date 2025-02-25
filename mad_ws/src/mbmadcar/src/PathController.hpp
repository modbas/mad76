/**
  * @brief C++ class PathConroller for path following control as part of motion control
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

#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#ifdef MAD24
#include "mbmad/CarParameters24.hpp"
#else
#include "mbmad/CarParameters.hpp"
#endif
#include "mbmad/Spline.hpp"
#include "mbmad/Utils.hpp"
#include "OperationModeFsm.hpp"

namespace mbmad {

class PathController
{
  public:
    PathController(rclcpp::Node& node) noexcept
    : node(node)
    {
    }

    PathController(const PathController& that) = delete;

    /**
     * @brief init creates reference spline from waypoints data
     */
    void init()
    {
      if (!initialized) {
        rclcpp::QoS qos { rclcpp::KeepLast(1) };
        qos.best_effort().durability_volatile();
        pubDebug = node.create_publisher<std_msgs::msg::Float32MultiArray>("pathdebug", qos);      
      }      
      initialized = true;
    }

    /**
     * @brief execute one single control step
     * @param[in] spline reference path
     * @param[in] s current car position
     * @param[in] v current car yaw angle
     * @param[in] Ts sampling time
     * @param[out] ey lateral deviation
     * @param[out] wcscmp curvature
     * @return manipulation signal steering
     */
    float step(Spline& spline, const OperationModeFsm& opModeFsm, const std::array<float,2> s, const float psi,
               const float v, float& ey, float& wkappa)
    {
      float deltan { 0.0F };
      
      if (initialized) {
        // maneuver received and thus spline is initialized
        if (std::holds_alternative<StateNormal>(opModeFsm.getState())) {
          stepReference(spline, s, wx, ws, wpsi, wkappa, v);
          float delta = stepFeedback(s, psi, ws, wpsi, v, ey);
          delta += stepFeedforward(wkappa);

          deltan = delta / p->deltaMax * deltanmax;
          if (deltan > deltanmax) {
            deltan = deltanmax;
          } else if (deltan < deltanmin) {
            deltan = deltanmin;
          }
        } else if (std::holds_alternative<StateDegraded>(opModeFsm.getState())) {
          deltan = deltan_km1;        
        } // else safety halt deltan = 0.0F
      }
      deltan_km1 = deltan;

      return deltan;
    }

    // global variables for reference message publishing
    float wx { 0.0F };
    std::array<float,2> ws { {} };
    float wpsi { 0.0F };

  private:
    CarParameters const * const p = CarParameters::p();  
    rclcpp::Node& node;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pubDebug;
#ifdef MAD24
    const float Tw { 300e-3F }; // closed-loop time constant [ s ]
    const float deltanmax { p->deltanMax }; // maximum manipulation signal
    const float deltanmin { -deltanmax }; // minimum manipulation signal
    const float Tt { 100.0e-3F }; // lookahead time        
#else
    const float Tw { 300e-3F }; // closed-loop time constant [ s ]
    const float deltanmax { p->deltanMax }; // maximum manipulation signal
    const float deltanmin { -deltanmax }; // minimum manipulation signal
    const float Tt { p->Tt }; // lookahead time    
#endif
    const float ky { p->l / (Tw*Tw) };
    const float kpsi { 2.0F * p->l / Tw };
    bool initialized { false };
    float deltan_km1 { 0.0F }; // steering angle in previous time step

    /**
     * @brief stepReference generates reference
     * @param spline reference path
     * @param s current position
     * @param ws reference position
     * @param wpsi reference yaw angle
     * @param wkappa reference curvature
     */
    void stepReference(Spline& spline,
                       const std::array<float,2>& s,
                       float& wx,
                       std::array<float,2>& ws, float& wpsi,
                       float& wkappa,
                       const float v)
    {
      std::array<float,2> wsd;
      std::array<float,2> wsdd;
      float dist { 0.0F };
      int idx = spline.getNearest(s, wx, dist);
      spline.interpolate(wx, ws, wsd, wsdd, idx);
      wpsi = std::atan2(wsd.at(1), wsd.at(0));

      float wxlookahead = wx + v * Tt;
      std::array<float,2> dummy;
      if (spline.boundaryCondition == Spline::BoundaryCondition::periodic) {
        if (wxlookahead > spline.breaks.back()) {
          wxlookahead -= spline.breaks.back();
        } else if (wxlookahead < 0.0F) {
          wxlookahead += spline.breaks.back();
        }
      }
      spline.interpolate(wxlookahead, dummy, wsd, wsdd, -1);
      wkappa = std::sqrt(wsdd.at(0)*wsdd.at(0) + wsdd.at(1)*wsdd.at(1));
      const float binormal = wsd.at(0)*wsdd.at(1) - wsdd.at(0)*wsd.at(1);
      if (binormal < 0.0F) {
        wkappa = -wkappa;
      }

#ifdef MAD24
      // if distance to the reference point is too large, move in to the
      // direction of the reference point
      if (dist > 4.0F * p->l) {
        wpsi = std::atan2(ws.at(1) - s.at(1), ws.at(0) - s.at(0));
        wkappa = 0.0F;
      }
#endif
    }

    /**
     * @brief stepFeedback feedback controller
     * @param s current position
     * @param psi current yaw angle
     * @param ws reference position
     * @param wpsi reference yaw angle
     * @param v reference speed
     * @param ey lateral deviation
     * @return steering angle delta
     */
    float stepFeedback(const std::array<float,2>& s,
                       const float psi,
                       const std::array<float,2>& ws,
                       const float wpsi,
                       const float v,
                       float &ey)
    {
      ey =  (s.at(1)-ws.at(1)) * std::cos(wpsi) - (s.at(0)-ws.at(0)) * std::sin(wpsi);
      const float psie = Utils::normalizeRad(psi - wpsi);

      float delta { 0.0F };
      float v1 { v };
      if (v1 >= 0.0F && v1 < p->speedMin) {
        v1 = p->speedMin;
      } else if (v1 < 0.0F && v1 > -p->speedMin) {
        v1 = -p->speedMin;
      }
      delta = -(ky/v1*ey + kpsi*psie) / v1;
      std::vector<float> debug { ey, psie, delta * p->deltanMax / p->deltaMax };
      std_msgs::msg::Float32MultiArray msg;
      msg.data = debug;
      pubDebug->publish(msg);
      //RCLCPP_INFO(node.get_logger(), "ey=%06.3f psie=%06.3f, delta=%06.3f", ey, psie, delta);

      return delta;
    }

    /**
     * @brief stepFeedforward feedforward controller
     * @param wkappa
     * @return steering angle delta
     */
    float stepFeedforward(const float wkappa)
    {
      const float delta = std::atan(wkappa * p->l);
      // std::vector<float> debug { wkappa, p->l, p->deltaMax, delta/p->deltaMax };
      // std_msgs::msg::Float32MultiArray msg;
      // msg.data = debug;
      // pubDebug->publish(msg);
      return delta;
    }
};

}
