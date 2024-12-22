/**
  * @brief C++ class PostionCOntroller for longitudinal control as part of motion control
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

#ifdef MAD24
#include "mbmad/CarParameters24.hpp"
#else
#include "mbmad/CarParameters.hpp"
#endif

namespace mbmad {

class PositionController
{
  public:
    PositionController() noexcept
    {
    }

    PositionController(const PositionController& that) = delete;

    /**
     * @brief initialize controller
     * @param[in] vref reference speed
     * @param[in] xref reference position
     * @param[in] x current position
     */
    void init(const float vref, const float xref, const float x)
    {
      t = 0.0F;
      this->xref = xref;
      this->vref = vref;
      this->xramp = x;
    }

    /**
     * @brief execute one single control step
     * @param[in] vref reference speed
     * @param[in] xref reference position
     * @param[in] v current speed
     * @param[in] Ts sampling time
     * @return manipulation signal speed
     */
    float step(const float x, const float Ts)
    {
      const float xe { xramp - x }; // control deviation
      float u { kp * xe }; // + kvp * vref }; // P-controller + feedforward controller

      // saturate
      if (u > CarParameters::p()->k) {
        u = CarParameters::p()->k;
      } else if (u < -CarParameters::p()->k) {
        u = -CarParameters::p()->k;
      }

      // ramp reference signal
      xramp += Ts * vref;
      if ((vref > 0.0F && xramp > xref)
          || (vref < 0.0F && xramp < xref)) {
        // saturate xramp
        xramp = xref;
        vref = 0.0F;
      }

      return u;
    }

  private:
    const float kp { 1.0F }; // P-controller gain [ 1/s ]
    float t { 0.0F }; // time started by init [ s ]
    float vref { 0.0F }; // reference speed from maneuver [ m/s ]
    float xref { 0.0F }; // reference position from maneuver [ m ]
    float xramp { 0.0F }; // ramped reference signal x [ m ]
};

}
