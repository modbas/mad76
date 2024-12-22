/**
  * @brief C++ class OperationModeFSM for finite state machine representing operation modes
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
#include <iostream>
#include "mbmadmsgs/msg/car_inputs.hpp"
#include "mbsafe/Fsm.hpp"

namespace mbmad {

struct EventLocationGained { };

struct EventLocationDegraded { };

struct EventLocationLost { };

struct StateNormal {
  StateNormal() { }
  static const uint8_t id { mbmadmsgs::msg::CarInputs::OPMODE_NORMAL };
};

struct StateSafetyHalt {
  StateSafetyHalt() { }
  static const uint8_t id { mbmadmsgs::msg::CarInputs::OPMODE_SAFETYHALT };
};

struct StateDegraded {
  StateDegraded() { }
  static const uint8_t id { mbmadmsgs::msg::CarInputs::OPMODE_DEGRADED };
};

using State = std::variant<StateSafetyHalt // default
                          , StateNormal, StateDegraded>;

class OperationModeFsm : public mbsafe::Fsm<OperationModeFsm, State>
{
public:
  auto getStateId() const {
    return std::visit(
          [&](auto& s) -> uint8_t { return s.id; },
          getState());
  }

  template<typename State, typename Event>
  auto onEvent(State&, const Event&)
  {
    std::cerr << "invalid transition" << std::endl;
    return std::nullopt;
  }

  auto onEvent(StateNormal&, const EventLocationLost&)
  {
    return StateSafetyHalt {};
  }

  auto onEvent(StateNormal&, const EventLocationDegraded&)
  {
    return StateDegraded {};
  }
  
  auto onEvent(StateNormal&, const EventLocationGained&)
  {
    return std::nullopt;
  }

  auto onEvent(StateSafetyHalt&, const EventLocationGained&)
  {
    return StateNormal {};
  }

  auto onEvent(StateSafetyHalt&, const EventLocationDegraded&)
  {
    return StateDegraded {};
  }

  auto onEvent(StateSafetyHalt&, const EventLocationLost&)
  {
    return std::nullopt;
  }

  auto onEvent(StateDegraded&, const EventLocationGained&)
  {
    return StateNormal {};
  }

  auto onEvent(StateDegraded&, const EventLocationDegraded&)
  {
    return std::nullopt;
  }

  auto onEvent(StateDegraded&, const EventLocationLost&)
  {
    return StateSafetyHalt {};
  }

};

}
