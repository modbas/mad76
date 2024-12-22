/**
  * @brief C++ class Spline for cubic splines
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

#include <vector>
#include <array>
#include <cmath>
#include <cstdint>
#include "TridiagonalSystem.hpp"

namespace mbmad {

/**
 * @brief The Spline class for cubic splines
 */
class Spline
{
public:
  constexpr static const int dim = 2; /**< spline dimensions on planar surface */
  constexpr static const int order = 3; /**< cubic spline */

  enum class BoundaryCondition { natural, periodic }; /**< boundary conditions of cubic spline */

  /**
   * @brief Spline constructor
   */
  explicit Spline() noexcept;

  /**
   * @brief Spline constructor interpolates waypoints
   * @param[in] breaks List of arc lengths of waypoints [ m ]
   * @param[in] vals0 List of x-coordinates of waypoints [ m ]
   * @param[in] vals1 List of y-coordinates of waypoints [ m ]
   * @param[in] segmentIds Id of segment at every waypoint
   * @param[in] period Boundary condition
   */
  explicit Spline(const std::vector<float>& breaks,
                  const std::vector<float>& vals0,
                  const std::vector<float>& vals1,
                  const std::vector<uint32_t>& segmentIds,
                  const bool periodic = true) noexcept;

  /**
   * @brief pushPoint adds one waypoint to internal list of waypoints
   * @param[in] brk Arc length of waypoint [ m ]
   * @param[in] val Coordinates of waypoint [ m ]
   * @param[in] segmentId Segment id of waypoint
   */
  void pushPoint(const float brk, const std::array<float, dim>& val,
                 const uint32_t segmentId = 0U);

  /**
   * @brief reserve preallocates memory for waypoints
   * @param[in] pieces_cnt expected number of waypoints
   */
  void reserve(const int pieces_cnt);

  /**
   * @brief computeCoefficients creates the spline. To be called after creating all waypoints.
   * @param[in] bc Boundary conditions
   */
  void computeCoefficients(const BoundaryCondition bc = BoundaryCondition::periodic);

  /**
   * @brief interpolate interpolates on the spline
   * @param[in] x The arc length of the point to be interpolated [ m ]
   * @param[out] y The coordinates of the interpolated point [ m ]
   * @param[out] yd The first derivative of the coordinates (tangential vector) [ 1 ]
   * @param[out] ydd The second derivative of the coordinates (normal vector) [ 1/m ]
   * @param[in] pieceIdx Optional waypoint index of interval on spline to speed up interpolation (default -1: search for interval on spline)
   */
  void interpolate(float x, std::array<float, dim>& y,
                   std::array<float, dim>& yd, std::array<float, dim>& ydd,
                   const int pieceIdx = -1);

  /**
   * @brief getNearest returns nearest point on spline (point which has minimal distance to y)
   * @param[in] y Coordinates of point next to spline (e.g., car position) [ m ]
   * @param[out] x Arc length of nearest point on spline [ m ]
   * @param[out] dist Approximate distance to spline. Do not use for control functions! [ m ]
   * @return Waypoint index of corresponding spline interval or -1 if spline is empty
   */
  int getNearest(const std::array<float, dim>& y, float& x, float& dist);

  /**
   * @brief getNearestWaypoint returns nearest waypoint on spline
   * @param[in] y Coordinates of point next to spline (e.g., car position) [ m ]
   * @param[out] x Arc length of nearest waypoint on spline [ m ]
   * @param[out] dist Approximate distance to spline. Do not use for control functions! [ m ]
   * @return Waypoint index or -1 if spline is empty
   */
  int getNearestWaypoint(const std::array<float, dim>& y, float& x, float& dist);

  /**
   * @brief replaceWaypoints replaces and appends waypoints
   * @param[in] fromIdx Index of starting waypoint in sequence to be replaced
   * @param[in] breaks Arc lengths
   * @param[in] vals0 s1-coordinates
   * @param[in] vals1 s2-coordinates
   * @param[in] segmentIds Segement ids
   * @return True on success
   */
  bool replaceWaypoints(const int fromIdx,
                        const std::vector<float>& breaks,
                        const std::vector<float>& vals0,
                        const std::vector<float>& vals1,
                        const std::vector<uint32_t>& segmentIds)
  {
    bool ret { false };
    if (fromIdx <= 0) {
      std::size_t len = breaks.size();
      this->breaks = breaks;
      this->segmentIds = segmentIds;
      this->vals.resize(len);
      for (std::size_t i = 0; i < breaks.size(); ++i) {
        this->vals.at(i).at(0) = vals0.at(i);
        this->vals.at(i).at(1) = vals1.at(i);
      }
      ret = true;
    } else if (fromIdx < static_cast<int>(this->breaks.size())) {
      float x0 = this->breaks.at(fromIdx);
      std::size_t len = fromIdx + breaks.size();
      this->breaks.resize(len);
      this->vals.resize(len);
      this->segmentIds.resize(len);
      for (std::size_t i = 0U; i < breaks.size(); ++i) {
        this->breaks.at(fromIdx + i) = breaks.at(i) + x0;
        this->vals.at(fromIdx + i).at(0) = vals0.at(i);
        this->vals.at(fromIdx + i).at(1) = vals1.at(i);
        this->segmentIds.at(fromIdx + i) = segmentIds.at(i);
      }
      ret = true;
    }
    return ret;
  }

  /**
   * @brief popWaypoints removes waypoints at beginning of waypoint sequence
   * @param toIdx Index of last waypoint to be removed
   * @return True on success
   */
  bool popWaypoints(const int toIdx)
  {
    bool ret { false };
    if (toIdx < static_cast<int>(this->breaks.size())) {
      breaks.erase(breaks.begin(), breaks.begin() + toIdx);
      vals.erase(vals.begin(), vals.begin() + toIdx);
      segmentIds.erase(segmentIds.begin(), segmentIds.begin() + toIdx);
      ret = true;
    }
    return ret;
  }

  /**
     * @brief getValVector extract one coordinate of the waypoints
     * @param val List of waypoints values of this one coordinate [ m ]
     * @param idx The coordinate index, may be 0 or 1 for planar splines
     */
  void getValVector(std::vector<float>& val, int32_t idx)
  {
    val.resize(breaks.size());
    for (int32_t i = 0; i < static_cast<int>(breaks.size()); ++i) {
      val[i] = vals[i][idx];
    }
  }

  std::vector<float> breaks; /**< internal arc lengths of waypoints [ m ] */
  std::vector<std::array<float, dim>> vals; /**< internal coordinates of waypoints [ m ] */
  std::vector<std::array<std::array<float,order+1>, dim>> coefs; /**< internal polynomial coefficients of spline intervals */
  std::vector<uint32_t> segmentIds; /**< segment id of each break */
  BoundaryCondition boundaryCondition { BoundaryCondition::natural }; /**< boundary conditions of spline */


private:
  /**
   * @brief binarySearch searches for spline interval (called from interpolate)
   * @param[in] x Arc length of interpolated point
   * @return Waypoint index of spline interval
   */
  int binarySearch(const float x);

  /**
   * @brief getDistance Computes distance of point y to waypoint with index i
   * @param[in] y Point coordinates [ m ]
   * @param[in] i Waypoint index
   * @return distance [ m ]
   */
  float getDistance(const std::array<float, dim>& y, const int i)
  {
    const std::array<float, dim> yd { { y.at(0)-vals.at(i).at(0), y.at(1)-vals.at(i).at(1) } };
    return std::sqrt(yd.at(0)*yd.at(0) + yd.at(1)*yd.at(1));
  }

  /**
   * @brief getNearestInternal is an auxiliary of getNearest and getNearestWaypoint
   * @param[in] y Coordinates of point next to spline (e.g., car position) [ m ]
   * @return Waypoint index of corresponding spline interval or -1 if spline is empty
   */
  int getNearestInternal(const std::array<float, dim>& y);

  /**
   * @brief computeCoefficientsNatural Computes spline coefficient in case of natural boundary conditions
   */
  void computeCoefficientsNatural();

  /**
   * @brief computeCoefficientsNatural Computes spline coefficient in case of periodic boundary conditions (circular tracks)
   */
  void computeCoefficientsPeriodic();
};

}
