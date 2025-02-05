/**
  * @brief ROS2 node RcNode for GPIO / SPI MAD76 IO control
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

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <cstring>
#include <vector>
#include <string>
#include <cstdint>

#include <chrono>
#include <thread>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include "mbsafe/CheckpointSequence.hpp"
#include "mbsafe/CheckpointDeadlineMonitor.hpp"
#include "mbsafe/CheckpointJitterMonitor.hpp"
#include "mbmadmsgs/msg/car_inputs.hpp"
#include "mbmad/CheckpointGraph.hpp"
#include "mbmad/CarParameters.hpp"

namespace mbmad
{

  /**
   * @brief The RcNode class
   */
  class RcNode : public rclcpp::Node
  {
  public:
    RcNode()
        : Node{"rcnode", "/mad/rc"}
    {
      cpSeq.registerMonitor(cpJitterMonReceive);
      cpSeq.registerMonitor(cpDeadlineMonPublish);
      cpSeq.registerMonitor(cpGraphMon);

      const uint8_t spiCmdPedals = 0x11;
      const uint8_t spiCmdSteering = 0x12;
      const uint8_t spiValNeutral = 0x7F;
      for (uint32_t id = 0U; id < CarParameters::p()->carCnt; ++id)
      {
        bufPedals[2 * id] = spiCmdPedals;
        bufPedals[2 * id + 1] = spiValNeutral;
        bufSteering[2 * id] = spiCmdSteering;
        bufSteering[2 * id + 1] = spiValNeutral;
      }
    }

    bool init()
    {
      boost::log::core::get()->set_filter(
          boost::log::trivial::severity >= boost::log::trivial::error);
      steeringOffsets = declare_parameter<std::vector<double>>("steeringoffsets", steeringOffsets);

      // create subscribers and publishers
      rclcpp::QoS qos{rclcpp::KeepLast(1)};
      qos.best_effort().durability_volatile();
      for (uint32_t id = 0U; id < CarParameters::p()->carCnt; ++id)
      {
        subInputs.push_back(
            create_subscription<mbmadmsgs::msg::CarInputs>("/mad/car" + std::to_string(id) + "/carinputs",
                                                           qos,
                                                           std::bind(&RcNode::inputsCallback, this, std::placeholders::_1)));
      }

      // diagnostics updater
      diagUpdater.setHardwareID(get_name());
      diagUpdater.add("CheckpointSequence", this, &RcNode::diagCheckpointSequence);
      diagUpdater.add("comm", this, &RcNode::diagCom);

      // Initialize WiringPi and SPI
      if (wiringPiSetup() == -1)
      {
        diagComErrText = strerror(errno);
        ++diagComErrCnt;
        BOOST_LOG_TRIVIAL(error) << diagComErrText;
        return false;
      }

      if (wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED) == -1)
      {
        diagComErrText = strerror(errno);
        ++diagComErrCnt;
        BOOST_LOG_TRIVIAL(error) << diagComErrText;
        return false;
      }

      if (wiringPiSetupGpio() == -1)
      {
        diagComErrText = strerror(errno);
        ++diagComErrCnt;
        BOOST_LOG_TRIVIAL(error) << diagComErrText;
        return false;
      }

      // config CS pin
      // pinMode(SPI_CS_PIN, OUTPUT);
      // digitalWrite(SPI_CS_PIN, HIGH);

      // config power pin
      for (auto pin : POWER_PINS)
      {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);        
      }
      
      // write zeros to steering, pedals
      for (uint8_t id = 0U; id < CarParameters::p()->carCnt; ++id)
      {
        writeSpi(id, 0.0F, 0.0F);
      }

      // switch on power
      for (auto pin : POWER_PINS)
      {
        digitalWrite(pin, HIGH);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // wait to protect against undervoltage
      }
      
      spiInitialized = true;
      return true;
    }

    bool step()
    {
      return true;
    }

    bool exit()
    {
      for (uint8_t id = 0U; id < CarParameters::p()->carCnt; ++id)
      {
        writeSpi(id, 0.0F, 0.0F);
      }

      // switch off power
      for (auto pin : POWER_PINS)
      {
        digitalWrite(pin, LOW);
      }

      return true;
    }

  private:
    static constexpr int SPI_CHANNEL = 0;
    static constexpr int SPI_SPEED = 1000000;
    static constexpr std::array<int, 4> POWER_PINS = { 25, 23, 24, 18 }; // { GPIO25, pin22 ; GPIO23, pin16 ; GPIO24, pin18 ; GPIO18, pin12 }
    // static constexpr int SPI_CS_PIN = 23;

    mbsafe::CheckpointSequence cpSeq{*this, CheckpointGraph::lastCheckpoint};
    mbsafe::CheckpointJitterMonitor cpJitterMonReceive{*this, cpSeq,
                                                       CheckpointGraph::Checkpoint::RcStart,
                                                       0.5 * static_cast<double>(mbmad::CarParameters::p()->Tva),
                                                       1.5 * static_cast<double>(mbmad::CarParameters::p()->Tva)};
    mbsafe::CheckpointDeadlineMonitor cpDeadlineMonPublish{*this, cpSeq,
                                                           CheckpointGraph::Checkpoint::CameraStart,
                                                           CheckpointGraph::Checkpoint::RcPublish,
                                                           0.0, CheckpointGraph::maxDeadline};
    mbsafe::CheckpointGraphMonitor cpGraphMon{*this, CheckpointGraph::graph};
    std::vector<rclcpp::Subscription<mbmadmsgs::msg::CarInputs>::SharedPtr> subInputs;
    diagnostic_updater::Updater diagUpdater{this, 1.0};
    std::vector<double> steeringOffsets;
    bool spiInitialized = false;
    uint32_t diagComErrCnt = 0U;
    std::string diagComErrText;
    uint8_t bufPedals[2 * mbmad::CarParameters::carCnt];
    uint8_t bufSteering[2 * mbmad::CarParameters::carCnt];

    inline uint8_t convertFloat32(float u)
    {
      if (u > 1.0F)
      {
        u = 1.0F;
      }
      else if (u < -1.0F)
      {
        u = -1.0F;
      }
      int16_t y = static_cast<int16_t>(std::floor(u * 128.0F + 0.5F));
      if (y < -128)
      {
        y = -128;
      }
      else if (y > 127)
      {
        y = 127;
      }
      return static_cast<uint8_t>(y + 128);
    }

    void inputsCallback(const mbmadmsgs::msg::CarInputs::SharedPtr msg)
    {
      if (msg->cpseq.cpseq.empty() == false)
      {
        cpSeq.init(msg->cpseq);
        cpSeq.update(CheckpointGraph::Checkpoint::RcStart);
        cpSeq.update(CheckpointGraph::Checkpoint::RcPublish);
      }
      if (spiInitialized)
      {
        if (msg->carid < CarParameters::p()->carCnt)
        {
          writeSpi(msg->carid, msg->pedals, msg->steering);
        }
      }
    }

    void diagCom(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
      if (diagComErrText != "")
      {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "new communication error");
      }
      else
      {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "no communication error");
      }
      stat.add("LastErrorMessage", diagComErrText);
      stat.addf("ErrorCount", "%u", diagComErrCnt);
    }

    void diagCheckpointSequence(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
      cpSeq.health().diag(stat);
    }

    void writeSpi(const uint8_t carid, const float pedals, const float steering)
    {
      const uint32_t id = CarParameters::p()->carCnt - 1U - carid;
      if (id >= CarParameters::p()->carCnt)
      {
        diagComErrText = "carid " + std::to_string(id) + "out of range";
        ++diagComErrCnt;
      }
      else
      {
        float steeringModified = steering;
        if (id < steeringOffsets.size())
        {
          steeringModified += steeringOffsets[id];
        }
        // RCLCPP_INFO(get_logger(), "carid=%u steering=%5.3f modified=%5.3f", id, steering, steeringModified);
        bufPedals[2 * id + 1] = convertFloat32(pedals);
        bufSteering[2 * id + 1] = convertFloat32(steeringModified);
        uint8_t bufPedalsLocal[2 * mbmad::CarParameters::carCnt];
        uint8_t bufSteeringLocal[2 * mbmad::CarParameters::carCnt];
        std::memcpy(bufPedalsLocal, bufPedals, 2 * CarParameters::p()->carCnt);
        std::memcpy(bufSteeringLocal, bufSteering, 2 * CarParameters::p()->carCnt);
        // digitalWrite(SPI_CS_PIN, LOW);
        wiringPiSPIDataRW(SPI_CHANNEL, bufPedalsLocal, 2 * CarParameters::p()->carCnt);
        //rclcpp::sleep_for(std::chrono::microseconds(1000));
        // digitalWrite(SPI_CS_PIN, HIGH);
        // digitalWrite(SPI_CS_PIN, LOW);
        wiringPiSPIDataRW(SPI_CHANNEL, bufSteeringLocal, 2 * CarParameters::p()->carCnt);
        // digitalWrite(SPI_CS_PIN, HIGH);
      }
    }
  };

}
