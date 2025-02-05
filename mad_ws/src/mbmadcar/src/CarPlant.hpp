/**
  * @brief C++ class CarPlant for vehicle dynamics simulation
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

#ifdef MAD24
#include "mbmad/CarParameters24.hpp"
#else
#include "mbmad/CarParameters.hpp"
#endif
#include <boost/numeric/odeint.hpp>
#include <deque>
#include <functional>

using namespace std::placeholders;

namespace mbmad {

class CarPlant
{
  public:
    using InputsType = std::array<float, 2>; /**< type definition for model inputs (array of floats with 1 element) */
    using StatesType = std::array<float, 7>; /**< type definition for model states */
    using OutputsType = std::array<float, 6>; /**< type definition for model outputs */

    CarPlant() noexcept
    {
      for (std::size_t i = 0U; i < p->x0.size(); ++i) {
        x[i] = p->x0[i];
      }
      for (std::size_t i = p->x0.size(); i < x.size(); ++i) {
        x[i] = 0.0F;
      }
      uFifo.resize(static_cast<size_t>((p->uTt - p->Tva) / p->Ta), 0.0F);
      deltaFifo.resize(static_cast<size_t>((p->deltaTt - p->Tva) / p->Ta), 0.0F);
    }

    CarPlant(const CarPlant& that) = delete;

    void init() noexcept
    {
      t = 0.0;
      for (std::size_t i = 0U; i < p->x0.size(); ++i) {
        x[i] = p->x0[i];
      }
      for (std::size_t i = p->x0.size(); i < x.size(); ++i) {
        x[i] = 0.0F;
      }
    }

    float getT() noexcept
    {
      return t;
    }

    /**
     * @brief execute one single integration step
     * @param[in] u input vector
     * @param[out] y output vector
     * @param[in] dt sampling time
     */
    void step(const InputsType& u, OutputsType& y) noexcept
    {
      uDelayed = uFifo.front();
      uFifo.pop_front();
      uFifo.push_back(std::fmin(std::fmax(u.at(0), -p->uMax), p->uMax));
      deltaDelayed = deltaFifo.front();
      deltaFifo.pop_front();
      deltaFifo.push_back(std::fmin(std::fmax(u.at(1), -p->deltanMax), p->deltanMax));
      solver.do_step(std::bind(&CarPlant::ode, this, _1, _2, _3), x, static_cast<double>(t), static_cast<double>(p->Ta));
      t += p->Ta;
      y.at(0) = x.at(1) - p->center * std::cos(x.at(3));
      y.at(1) = x.at(2) - p->center * std::sin(x.at(3));
      y.at(2) = x.at(3);
      float delta = 0.0F;
      float tanDeltaMod = 0.0F;
      float beta = 0.0F;
      computeBeta(deltaDelayed, delta, tanDeltaMod, beta);
      y.at(3) = beta;
      y.at(4) = x.at(0);
      y.at(5) = x.at(6);
    }

  private:
    CarParameters const * const p { CarParameters::p() };
    float t = 0.0F; /**< simulation time */
    StatesType x { {} }; /**< state signal */
    boost::numeric::odeint::runge_kutta4<StatesType> solver;
    std::deque<float> uFifo;
    float uDelayed = 0.0F;
    std::deque<float> deltaFifo;
    float deltaDelayed = 0.0F;

    void computeBeta(const float deltan, float& delta, float& tanDeltaMod, float& beta) noexcept
    {
      delta = p->deltaMax * deltan / p->deltanMax;
      tanDeltaMod = p->lr * std::tan(delta) / p->l;
      beta = std::atan(tanDeltaMod);
    }

    void ode(const StatesType& x, StatesType& xd, const float t) noexcept
    {
      (void)t; // unused parameter
      float u = 0.0F;

      // friction at standstill and in curves
      const float und = p->uFrictionKd0 + p->uFrictionKd1 * std::fabs(uDelayed * deltaDelayed);
      if (uDelayed > und) {
          u = uDelayed - und;
      } else if (uDelayed < -und) {
          u = uDelayed + und;
      } else {
          u = 0.0F;
      }

      const float deltan = deltaDelayed;

      float delta = 0.0F;
      float beta = 0.0F;
      float tanDeltaMod = 0.0F;
      computeBeta(deltan, delta, tanDeltaMod, beta);

      if (p->modelType == CarParameters::ModelType::dynamics
          && std::fabs(x.at(0)) > p->speedMin) { // not stand still
        // nonlinear vehicle dynamics model (Section 5.5)
        float alphaf = -std::atan((x.at(5) + p->lf * x.at(4)) / x.at(0)) + delta;
        float alphar = -std::atan((x.at(5) - p->lr * x.at(4)) / x.at(0));
        if (x.at(0) < 0.0F) {
          alphaf = -alphaf;
          alphar = -alphar;
        }
#ifdef MAD24
        const float Ff = p->Df * std::sin(p->Cf * std::atan(p->Bf * (1.0F - p->Ef) * alphaf - p->Ef * std::atan(p->Bf * alphaf)));
        const float Fr = p->Dr * std::sin(p->Cr * std::atan(p->Br * (1.0F - p->Er) * alphar - p->Er * std::atan(p->Br * alphar)));
#else
        const float Ff = p->cf * alphaf;
        const float Fr = p->cr * alphar;
#endif
        // ODE
        xd.at(0) = -Ff * std::sin(delta) / p->m + x.at(5) * x.at(4)
            + (-x.at(0) + p->k * u) / p->T; // vc1=vr: rear wheel speed
        xd.at(1) = x.at(0) * std::cos(x.at(3)) - x.at(5) * std::sin(x.at(3)); // s1
        xd.at(2) = x.at(0) * std::sin(x.at(3)) + x.at(5) * std::cos(x.at(3)); // s2
        xd.at(3) = x.at(4); // psi
        xd.at(4) = (Ff * p->lf * std::cos(delta) - Fr * p->lr) / p->J; // dot{psi}
        xd.at(5) = (Ff * std::cos(delta) + Fr - p->m * x.at(0) * x.at(4)) / p->m; // vc2
        xd.at(6) = x.at(0); // arc length x of rear axle
      } else { // stand still or kinematics model
        // nonlinear kinematics model (Section 5.4)
        const float vc1 = x.at(0);
        float psid = 0.0F;
        if (vc1 >= 0.0F) {
          psid =  vc1 * std::tan(delta) / (p->l + vc1*vc1 * p->EG);
        } else {
          psid =  vc1 * std::tan(delta) / (p->l - vc1*vc1 * p->EG);
        }
        const float vc2 = p->lr * psid;
        xd.at(0) = (-x.at(0) + p->k * u) / p->T; // COG speed
        xd.at(1) = vc1 * std::cos(x.at(3)) - vc2 * std::sin(x.at(3)); // s1
        xd.at(2) = vc1 * std::sin(x.at(3)) + vc2 * std::cos(x.at(3)); // s2
        xd.at(3) = psid; // psi
        xd.at(4) = xd.at(0) * std::tan(delta) / p->l; // dot{psi}
        xd.at(5) = p->lr * xd.at(4); // vc2
        xd.at(6) = x.at(0) * std::cos(x.at(3)); // arc length x of rear axle
      }
    }
};

}