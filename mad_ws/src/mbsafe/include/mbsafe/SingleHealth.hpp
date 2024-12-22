/**
  * @brief C++ class SingleHealth of MBSAFE
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

#include <cstdint>
#include <string>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include "Health.hpp"

namespace mbsafe {

class SingleHealth : public Health
{
public:
  SingleHealth() = delete;

  explicit SingleHealth(const std::string& name, const int32_t debounceErrorMax = 0, const int32_t debounceRecoverMax = 0)
    : Health { name }, debounceErrorMax { debounceErrorMax }, debounceRecoverMax { debounceRecoverMax }
  {
  }

  virtual ~SingleHealth()
  {
  }

  void update(const bool healthArg, const std::string& statusArg,
              const std::string& errorArg)
  {
    status_ = statusArg;
    // FSM
    if (health_ == true) {
      if (debounceCtr > 0) {
        // state 0: health == true and debouncing new error
        if (healthArg == true) {
          // transition: health regained -> state 1
          debounceCtr = 0;
        } else {
          // transition: no health -> state 0 or state 3
          --debounceCtr;
          if (debounceCtr == 0) {
            updateError(false, errorArg);
          }
        }
      } else {
        // state 1: health == true and no debouncing
        if (healthArg == false) {
          // transition: no health -> state 0 or state 3
          debounceCtr = debounceErrorMax;
          if (debounceCtr == 0) {
            updateError(false, errorArg);
          }
        }
      }
    } else { // health == false
      if (debounceCtr > 0) {
        // state 2: health == false and debouncing auto recovery
        if (healthArg == true) {
          // transition: health regained -> state 1 or state 2
          --debounceCtr;
          if (debounceCtr == 0) {
            recover();
          }
        } else {
          // transition: no health -> state 3
          debounceCtr = 0;
          updateError(false, errorArg);
        }
      } else {
        // state 3: health == false and no debouncing
        if (healthArg == true) {
          // transition: health regained ->
          //   -> state 2 if debounceRecoverMax > 0
          //   -> state 3 if debounceRecoverMax == 0
          debounceCtr = debounceRecoverMax;
        } else {
          updateError(false, errorArg);
        }
      }
    }
    ++measureCnt_;
  }

  virtual void recover()
  {
    health_ = true;
  }

  virtual void reset()
  {
    health_ = true;
    measureCnt_ = 0ULL;
    errorCnt_ = 0ULL;
  }

  virtual bool health() const
  {
    return health_;
  }

  virtual uint64_t measureCount() const
  {
    return measureCnt_;
  }

  virtual uint64_t errorCount() const
  {
    return errorCnt_;
  }

  virtual std::string status() const
  {
    return status_;
  }

  virtual std::string lastError() const
  {
    return lastError_;
  }

private:
  const int32_t debounceErrorMax { 0 };
  const int32_t debounceRecoverMax { 0 };
  int32_t debounceCtr { 0 };
  std::string status_ { "ok" };
  std::string lastError_ { "none" };
  bool health_ { true };
  uint64_t measureCnt_ { 0ULL };
  uint64_t errorCnt_ { 0ULL };

  void updateError(const bool healthArg, const std::string& errorArg)
  {
    if (healthArg == false) {
      health_ = false;
      lastError_ = errorArg;
      ++errorCnt_;
      BOOST_LOG_TRIVIAL(debug) << name() << " STATUS=" << status() << " ERROR=" << errorArg << " ERRCNT=" << errorCnt_;
   }
  }
};

}
