/**
  * @brief LTTNG tracepoint of MBSAFE
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
 
#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER mbsafe

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "./mbsafe/mbsafe-tp.hpp"

#if !defined(_MBSAFE_TP_HPP) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _MBSAFE_TP_HPP

#include <lttng/tracepoint.h>
#include <stdint.h>

TRACEPOINT_EVENT(
    mbsafe,
    cp,
    TP_ARGS(
        uint64_t, seqCtrArg,
        uint64_t, cpIdArg,
        uint32_t*, uint32ArrayArg
    ),
    TP_FIELDS(
        ctf_integer(uint64_t, seqCtr, seqCtrArg)
        ctf_integer(uint64_t, cpId, cpIdArg)
        ctf_array(uint32_t, uint32Array, uint32ArrayArg, 8)  // [ channelId, carId, ... ]
    )
)

#endif /* _MBSAFE_TP_HPP */

#include <lttng/tracepoint-event.h>
