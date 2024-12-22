/**
  * @brief C++ class MeasureQuantity of MBSAFE
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
#include <cstdint>
#include <cmath>
#include <string>
#include "SingleHealth.hpp"
#include "String.hpp"

namespace mbsafe
{

template <typename DataType> class MeasureQuantity
{
public:
  MeasureQuantity() = delete;

  explicit MeasureQuantity(const std::string& name, const DataType& minLimit, const DataType& maxLimit,
                           const int32_t debounceErrorMax = 0, const int32_t debounceRecoverMax = 0)
    : health_ { name, debounceErrorMax, debounceRecoverMax }, minLimit { minLimit }, maxLimit { maxLimit }
  {
  }

  void update(const DataType& value)
  {
    // measure
    ++measureCnt;
    curVal = value;
    if (value < minVal) {
      minVal = value;
    }
    if (value > maxVal) {
      maxVal = value;
    }    
    const double deviation = static_cast<double>(value) - avgVal;
    avgVal += deviation / static_cast<double>(measureCnt);
    // Welford's online algorithm (https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance)
    moment2 += deviation * (static_cast<double>(value) - avgVal);
    stdVal = std::sqrt(moment2 / static_cast<double>(measureCnt));

    // qualify
    std::string status { String::sprintf("dt=%8.6lf min=%8.6lf max=%8.6lf avg=%8.6lf",
                                         static_cast<double>(curValue()), static_cast<double>(minValue()),
                                         static_cast<double>(maxValue()), static_cast<double>(avgValue())) };
    if (value < minLimit || value > maxLimit) {
      health_.update(false, status, std::to_string(value) + " is not in range [" + std::to_string(minLimit) + "," +  std::to_string(maxLimit) + "]");
    } else {
      health_.update(true, status, "");
    }
  }

  void reset()
  {
    measureCnt = 0ULL;
    health_.reset();
    minVal = {};
    maxVal = {};
    avgVal = 0.0;
    moment2 = 0.0;
    stdVal = 0.0;
  }

  DataType curValue() const
  {
    return curVal;
  }

  DataType minValue() const
  {
    return minVal;
  }

  DataType maxValue() const
  {
    return maxVal;
  }

  double avgValue() const
  {
    return avgVal;
  }

  double stdValue() const
  {
    return stdVal;
  }

  Health& health()
  {
    return health_;
  }

private:
  SingleHealth health_;
  const DataType minLimit { std::numeric_limits<DataType>::min() };
  const DataType maxLimit { std::numeric_limits<DataType>::max() };
  DataType curVal { };
  DataType minVal { std::numeric_limits<DataType>::max() };
  DataType maxVal { std::numeric_limits<DataType>::min() };
  double avgVal { 0.0 };
  double stdVal { 0.0 };
  double moment2 { 0.0 };
  uint64_t measureCnt { 0ULL };
};

}
