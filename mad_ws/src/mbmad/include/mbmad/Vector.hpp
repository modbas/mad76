/**
  * @brief C++ template class Vector for 2D vectors
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

#include <array>
#include <cmath>
#include <cstdint>

namespace mbmad {

/**
 * Vector template class
 */
template<class T> class Vector
{
public:
  /**
   * @brief Vector
   * @param[in] s0 x-coordinate
   * @param[in] s1 y-coordinate
   */
  Vector(const T s0, const T s1) noexcept
    : s { s0, s1 }
  {
  }

  /**
   * @brief Vector
   * @param[in] s0 x-coordinate
   * @param[in] s1 y-coordinate
   */
  Vector(const std::array<T,2>& s) noexcept
    : s { s }
  {
  }

  /**
   * @brief Vector copy constructor
   * @param[in] rhs Second vector
   * @return this
   */
  Vector(const Vector& rhs) noexcept
  {
    s[0] = rhs.s[0];
    s[1] = rhs.s[1];
  }


  /**
   * @brief operator =
   * @param[in] rhs Second vector
   * @return this
   */
  Vector& operator=(const Vector& rhs)
  {
    s[0] = rhs.s[0];
    s[1] = rhs.s[1];
    return *this;
  }

  /**
   * @brief operator -= to subtract two vectors
   * @param[in] rhs Second vector
   * @return difference
   */
  Vector& operator-=(const Vector& rhs)
  {
    s[0] -= rhs.s[0];
    s[1] -= rhs.s[1];
    return *this;
  }

  /**
   * @brief operator - to subtract two vectors
   * @param[in] lhs First vector
   * @param[in] rhs Second vector
   * @return difference
   */
  friend Vector operator-(Vector lhs, const Vector& rhs)
  {
    lhs -= rhs;
    return lhs;
  }

  /**
   * @brief operator += to add two vectors
   * @param[in] rhs Second vector
   * @return difference
   */
  Vector& operator+=(const Vector& rhs)
  {
    s[0] += rhs.s[0];
    s[1] += rhs.s[1];
    return *this;
  }

  /**
   * @brief operator /= to divide vector by scalar
   * @param[in] d scalar divisor
   * @return dot product
   */
  Vector& operator/=(const T d)
  {
    s[0] /= d;
    s[1] /= d;
    return *this;
  }

  /**
   * @brief operator + to add two vectors
   * @param[in] lhs First vector
   * @param[in] rhs Second vector
   * @return difference
   */
  friend Vector operator+(Vector lhs, const Vector& rhs)
  {
    lhs += rhs;
    return lhs;
  }

  /**
   * @brief operator * to multiply two vectors (dot product)
   * @param[in] lhs First vector
   * @param[in] rhs Second vector
   * @return dot product
   */
  friend T operator*(Vector lhs, const Vector& rhs)
  {
    return lhs.s[0] * rhs.s[0] + lhs.s[1] * rhs.s[1];
  }
  /**
   * @brief operator * to multiply vector by scalar
   * @param[in] lhs First vector
   * @param[in] d scalar multifier
   * @return Vector multified with scalar
   */
  friend Vector operator*(Vector lhs, const T d)
  {
    lhs.s[0] *= d;
    lhs.s[1] *= d;
    return lhs;
  }
  /**
   * @brief operator * to multiply vector by scalar
   * @param[in] d scalar multifier
   * @param[in] lhs First vector
   * @return Vector multified with scalar
   */
  friend Vector operator*(const T d, Vector lhs)
  {
    return lhs*d;
  }

  /**
   * @brief operator / to divide vector by scalar
   * @param[in] d scalar divisor
   * @return dot product
   */
  friend Vector& operator/(Vector lhs, const T d)
  {
    lhs.s[0] /= d;
    lhs.s[1] /= d;
    return lhs;
  }

  /**
   * @brief <
   * @param[in] lhs First vector
   * @param[in] rhs Second vector
   * @return bool
   */
  friend bool operator< (const Vector& lhs, const Vector& rhs)
  {
    bool compare;
    compare = lhs.s[0] < rhs.s[0];
    if(lhs.s[0]== rhs.s[0])
    {
      compare = lhs.s[1] < rhs.s[1];
    }
    return compare;
  }


  /**
   * @brief dist computes distance of two vectors
   * @param[in] rhs Second vector
   * @return distance
   */
  inline float dist(const Vector& rhs) const
  {
    Vector diff = rhs - *this;
    return diff.abs();
  }

  /**
   * @brief abs computes vector length
   * @return vector length
   */
  inline float abs() const
  {
    return std::sqrt(static_cast<float>(s[0])*static_cast<float>(s[0]) +
        static_cast<float>(s[1])*static_cast<float>(s[1]));
  }

  /**
   * @brief center computes center point between two positions (vectors)
   * @param[in] other Second vector
   * @param[out] c Center point
   */
  inline void center(const Vector& other, Vector<float>& c) const
  {
    c.s[0] = 0.5F * (static_cast<float>(s[0]) + static_cast<float>(other.s[0]));
    c.s[1] = 0.5F * (static_cast<float>(s[1]) + static_cast<float>(other.s[1]));
  }


  /**
   * @brief s The two coordinates of the vector
   */
  std::array<T, 2> s { };
};

}
