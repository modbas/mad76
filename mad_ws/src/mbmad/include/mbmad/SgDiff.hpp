/**
  * @brief C++ class SgDiff for signal differentiation by Savitzky-Golay filter
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

#include <deque>
#include <array>
#include <cstdint>
#include <iostream>

namespace mbmad {

class SgDiff
{
  public:
    SgDiff() noexcept
    {
    }

    SgDiff(const std::size_t order) noexcept
      : order { order }
    {
    }

    void reset() noexcept
    {
      x.clear();
    }

    float filter(const float u) noexcept
    {
      x.push_back(u);
      while (x.size() > order) {
        x.pop_front();
      }
      const std::size_t order = x.size();
      if (order >= 2U) {
        // use backward differences or Savitzky-Golay
        y = 0.0F;
        for (uint8_t idx = 0U; idx < order; ++idx) {
          y += b[order-1][idx] * x.at(idx);
        }
      } // else: return previous value

      return y;
    }

   friend std::ostream& operator<<(std::ostream& os, const SgDiff& sgdiff);
   
  private:
    //const std::array<float, 4> b { { 0.45F, -0.85F, -0.65F, 1.05F } };
    static constexpr std::size_t orderMax = 7U;
    const float b[orderMax][orderMax] =
        { { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },  // 1 point --> no differentation possible
          { -1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F }, // 2 points --> backward differences
          { -0.5F, 0.0F, 0.5F, 0.0F, 0.0F, 0.0F, 0.0F }, // 3 points --> sgsdf(-2:0, 1, 1, 0, 0)
          { 0.45F, -0.85F, -0.65F, 1.05F, 0.0F, 0.0F, 0.0F }, // 4 points --> sgsdf(-3:0, 2, 1, 0, 0)
          { 0.371428571428572F, -0.385714285714287F, -0.571428571428573F, -0.185714285714286F, 0.771428571428572F, 0.0F, 0.0F }, // 5 points --> sgsdf(-4:0, 2, 1, 0, 0)
          { 0.303571428571430F, -0.175000000000000F, -0.385714285714287F, -0.328571428571429F, -0.003571428571429F, 0.589285714285715F, 0.0F }, // 6 points -> sgsdf(-5:0, 2, 1, 0, 0)
          { -0.305555555555571F, 0.484126984127005F, 0.305555555555571F, -0.285714285714292F,  -0.734126984127009F, -0.48412698412700F, 1.019841269841301F } }; // 7 points -> sgsdf(-6:0, 3, 1, 0)
    const std::size_t order { orderMax };
    std::deque<float> x;
    float y = 0.0F;
};

std::ostream& operator<<(std::ostream& os, const SgDiff& sgdiff)
{
  for (const auto& xi : sgdiff.x) {
    os << xi << " ";
  }
  return os;
}

}
