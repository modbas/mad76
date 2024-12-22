/**
  * @brief C++ class CarParameters for vehicle dynamics parameters (MAD in 1:76 scale)
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

#include "mbmad/Utils.hpp"
#include <array>
#include <vector>
#include <vector>
#include <string>
#include <cstdint>

namespace mbmad
{

using state_t = std::array<float, 4>; /**< The state vector type of the vehicle bicycle model */
using stateext_t = std::array<float, 7>; /**< The state vector type of the vehicle dynamics model */

/**
 * @brief The CarParameters class defines all car parameters
 */
class CarParameters
{
public:
    enum class ModelType { bicycle = 0, dynamics = 1 }; /**< Enum to switch between bicycle and dynamics model */

    /**
     * @brief CarParameters constructor used by singleton
     */
    CarParameters() noexcept;

    /**
     * @brief Use this static method to access the parameters globally
     * @return
     */
    static inline CarParameters* p()
    {
      return singleton;
    }

    /**
     * @brief Retrieves carid from end of ROS node namespace
     * @param ns namespace
     */
    void setCarid(const std::string& ns)
    {
      carid = static_cast<int32_t>(std::strtol(&ns[ns.size()-1], nullptr, 10));
    }

    // Car Model Type
    ModelType modelType { ModelType::bicycle };

    // max. number of virtual cars
    static constexpr uint32_t carCnt { 2U };
    // max. number of real cars
    static constexpr uint32_t realCarCnt { 1U };
    // size [ m ]
    const std::array<float,2> size {{ 49e-3F, 22e-3F }}; // {{ 250e-3F, 150e-3F }}; //{{ 348e-3F, 164e-3F }};
    // distance of center relative to rear axle [ m ]
    const float center { 16.25e-3F };
    // longitudinal dynamics gain [ m/s ]
    const float k { 3.2F };
    // open-loop longitudinal dynamics time constant [ s ]
    const float T { 90.0e-3F };
    // open-loop dead time of longitudinal dynamics [ s ]
    const float uTt { 125e-3F };
    // open-loop dead time of steering [ s ]
    const float deltaTt { uTt };
    // image processing dead time
    const float outputTt { 0e-3F };
    // total dead time [ s ]
    const float Tt { uTt + outputTt };
    // longitudinal friction
    const float uFrictionPos { 0.04F };
    const float uFrictionNeg { -0.035F };
    // pedals max
    const float uMax { 0.2F };
    // normalized delta max [ 1 ]
    const float deltanMax { 0.9F }; 
    // delta max [ rad ]
    const float deltaMax { Utils::deg2rad(20.0F) }; 
    // wheel base [ m ]
    const float l { 32.5e-3F };
    // minimum speed [ m/s ]
    const float speedMin { 0.05F };
    // maximum speed [ m/s ]
    float speedMax { k };
    // maximum speed limit [ m/s ]
    const float speedMaxLimit { speedMax };
    // steering adaption clockwise
    const float adaptcw { 0.000F };
    // steering adaption counterclockwise
    const float adaptccw { 0.049F };

    // track
    static constexpr float a1total { 0.935F }; // total track width [ m ]
    static constexpr float a2total { 0.490F }; // total track height [ m ]

    // planning
    const int32_t planHorizon = 120; // number of waypoints
    const bool replanEnable = false; // enable replanning during control

    // dynamics model
    const float m { 13e-3F }; // mass [ kg ]
    const float J { 3e-6F }; // moment of inertia m*(b^2+h^2)/12 [ kg*m^2 ]
    const float lf { 16.25e-3F }; // distance COG to front axle [ m ]
    const float lr { 16.25e-3F }; // distance COG to rear axle [ m ]

    // tyre coefficients of Pacejka Model (Magic Formula)
    const float Br { 0.7F }; // rear: stiffness factor
    const float Cr { 2.0F }; // rear: shape factor
    const float Dr { 2.5F }; // rear: peak value
    const float Er { -0.05F }; // rear: shape factor
    const float Bf { 0.7F }; // front: stiffness factor
    const float Cf { 2.0F }; // front: shape factor
    const float Df { 2.0F }; // front: peak value
    const float Ef { -0.1F }; // front: shape factor


    // // lateral control
    // const float lateralAccMax { 3.0F }; // maximum lateral acceleration [ m/s^2 ]
    // const float lateralTw { 200e-3F}; // closed-loop longitudinal dynamics time constant [ s ]

    // // speed control
    // const float speedLookahead { T * 0.4F }; // lookahead for speed control [ s ]
    // const float speedKp { 0.238F }; // speed control gain
    // const float speedTi { 140e-3F }; // speed control integral time

    // sampling time [ s ]
    static constexpr float Ta { 5e-3F };

    // downsample for car marker publishing
    const int32_t realtimeDownsample = static_cast<int32_t>(50e-3F / Ta);
    const int32_t nonrealtimeDownsample = static_cast<int32_t>(50e-3F / Ta);

    // downsample for carouputs message of madvision
    static constexpr int32_t caroutputsDownsample = 5; // so that caroutputsext sampling time is 25ms

    // downsample for carstate message of madrc2
    static constexpr int32_t carstateDownsample = 200; // so that carstate sampling time rate is 1000ms

    // vision sampling time [ s ]
    static constexpr float Tva { static_cast<float>(caroutputsDownsample) * Ta };

    // display
    static constexpr float displayScale = 0.25F;

    //xExtension to cut and lengthen the planes Spline
    const int idxExtension { 7 }; // add arc length at start and end for path following control
    const float xExtension { static_cast<float>(idxExtension) * 0.02F };


    // user mission limits
    const float vmax { 2.0F };
    const float vmin { 0.1F };
    const uint32_t lapCountMax { 20U };

    // message names
    const std::vector<std::string> carState {
      { "connected", "disconnect", "error", "drive",
        "drive_slow", "charge" }
    };

    const std::vector<std::string> carErr {
      { "none", "calib", "overcurrent", "rn4020_detect_firmware",
        "steer_control", "carin_msg_watchdoc", "unknown" }
    };

    const std::vector<std::string> maneuverState {
      { "waiting", "safetyhalt", "safetystop", "driving",
        "halt", "charging", "locating", "batteryempty?", "error" }
    };

    // initial conditions v0, s10, s20, psi0
    state_t x0 { { 0.0F, 0.5F, 0.30F, 0.0F } };


    // car id
    int32_t carid = 0;

private:
    static CarParameters* singleton; /**< This class is a singleton */
};

}
