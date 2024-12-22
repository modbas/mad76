/**
  * @brief C++ class String of MBSAFE
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

#include <cstdio>
#include <string>
#include <cassert>

namespace mbsafe
{

class String
{
public:
  template< typename... Args >
  static std::string sprintf( const char* format, Args... args )
  {
    int length = std::snprintf( nullptr, 0, format, args... );
    assert( length >= 0 );

    char* buf = new char[length + 1];
    std::snprintf( buf, length + 1, format, args... );

    std::string str( buf );
    delete[] buf;
    return str;
  }

};

}