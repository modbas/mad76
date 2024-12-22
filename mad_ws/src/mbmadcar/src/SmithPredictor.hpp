/**
  * @brief C++ class SmithPredictor for delay compensation in speed control
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

#include "mbmad/CarParameters.hpp"
#include <cstdint>
#include <deque>

namespace mbmad {

class SmithPredictor
{
  public:
    SmithPredictor(const uint32_t d, const float b1, const float a1)  noexcept
    : b1 { b1 }, a1 { a1 }
    {
        yQueue.resize(d, 0.0F);
    }

    SmithPredictor(const SmithPredictor& that) = delete;

    /**
     * @brief execute one single step
     * @param[in] uk control signal
     * @return loopback signal (input to add block)
     */
    float step(const float ukm1)
    {
        const float yk = b1 * ukm1 + a1 * ykm1;
        ykm1 = yk;
        const float yDelayed = yQueue.back();
        yQueue.pop_back();
        yQueue.push_front(yk);
        return yk - yDelayed;
    }

  private:
    const float b1 { 0.0F }; // numerator coefficients
    const float a1 { 0.0F }; // denominator coefficients
    float ykm1 { 0.0F }; // previous output signal
    std::deque<float> yQueue; // delayed output signal for loopback
};

} // namespace mbmad