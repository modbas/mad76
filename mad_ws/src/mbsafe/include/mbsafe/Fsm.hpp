/**
  * @brief C++ termpate class Fsm for finite-state machines of MBSAFE
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

#include <optional>
#include <variant>
#include <utility>

namespace mbsafe
{

template<typename Derived, typename StateVariant>
class Fsm {

public:
  const StateVariant& getState() const
  {
    return state_;
  }

  StateVariant& getState()
  {
    return state_;
  }

  template<typename Event>
  void dispatch(Event&& event)
  {
    Derived& child = static_cast<Derived&>(*this);
    auto newState = std::visit(
                        [&](auto& s) -> std::optional<StateVariant> { return child.onEvent(s, std::forward<Event>(event)); },
                        state_);
    if (newState) {
      state_ = *std::move(newState);
    }
  }

private:
  StateVariant state_;
};

}
