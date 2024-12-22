/**
  * @brief C++ class Health of MBSAFE
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

namespace mbsafe {

class Health
{
public:
  Health() = delete;

  explicit Health(const std::string& name)
    : name_ { name }
  {
  }

  virtual ~Health()
  {
  }

  virtual void recover() = 0;

  virtual void reset() = 0;

  virtual bool health() const = 0;

  virtual uint64_t measureCount() const = 0;

  virtual uint64_t errorCount() const = 0;

  const std::string& name()
  {
    return name_;
  }

  virtual std::string status() const = 0;

  virtual std::string lastError() const = 0;

protected:
  const std::string name_;
};

}
