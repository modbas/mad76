/**
  * @brief C++ class CombinedHealth of MBSAFE
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
#include <list>
#include <string>
#include <sstream>
#include <memory>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <boost/log/trivial.hpp>
#include "Health.hpp"

namespace mbsafe {

class CombinedHealth : public Health
{
public:
  CombinedHealth() = delete;

  explicit CombinedHealth(const std::string& name)
    : Health { name }
  {
  }

  virtual ~CombinedHealth()
  {
  }

  void registerSubHealth(Health& health)
  {
    healthList.push_back(&health);
  }

  /*void update(const bool healthArg)
    {
      if (healthArg == false) {
        health_ = false;
        ++errorCnt_;
        BOOST_LOG_TRIVIAL(error) << name() << " STATUS=" << status() << " ERROR=" << errorArg << " ERRCNT=" << errorCnt_;
      }
      ++measureCnt_;
    }*/

  virtual void recover()
  {
    for (auto h : healthList) {
      h->recover();
    }
  }

  virtual void reset()
  {
    for (auto h : healthList) {
      h->reset();
    }
  }

  virtual bool health() const
  {
    bool ret { true };
    for (auto h : healthList) {
      ret = (ret && h->health());
    }
    return ret;
  }

  virtual uint64_t measureCount() const
  {
    uint64_t ret { 0ULL };
    for (auto h : healthList) {
      ret += h->measureCount();
    }
    return ret;
  }

  virtual uint64_t errorCount() const
  {
    uint64_t ret { 0ULL };
    for (auto h : healthList) {
      ret += h->errorCount();
    }
    return ret;
  }

  virtual std::string status() const
  {
    std::stringstream ss;
    for (auto h : healthList) {
      ss << h->name() << ":" << h->status() << " ";
    }
    return ss.str();
  }

  virtual std::string lastError() const
  {
    std::stringstream ss;
    for (auto h : healthList) {
      ss << h->name() << ":" << h->lastError() << " ";
    }
    return ss.str();
  }

  void diag(diagnostic_updater::DiagnosticStatusWrapper& stat) const
  {
    if (health() == true) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "health is ok");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "health is not ok");
    }
    for (auto h : healthList)
    {
      stat.add<bool>(h->name(), h->health());
      stat.add<std::string>(h->name() + " status", h->status());
      stat.add<std::string>(h->name() + " error", h->lastError());
      stat.add<uint64_t>(h->name() + " errorCount", h->errorCount());
    }
    stat.add<uint64_t>("measureCount", measureCount());
    stat.add<uint64_t>("errorCount", errorCount());
  }

private:
  std::list<Health*> healthList;
};

}
