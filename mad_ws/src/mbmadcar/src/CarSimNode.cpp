/**
  * @brief ROS2 node carsimnode for vehicle dynamics simulation in SIL
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

#include <memory>
#include <cstdint>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include "mbsafe/Platform.hpp"
#include "mbsafe/Task.hpp"
#include "mbmadmsgs/msg/car_inputs.hpp"
#include "mbmadmsgs/msg/car_outputs_ext.hpp"
#ifdef MAD24
#include "mbmad/CarParameters24.hpp"
#else
#include "mbmad/CarParameters.hpp"
#endif
#include "CarPlant.hpp"

namespace mbmad
{

class CarSimNode : public rclcpp::Node
{
public:
  CarSimNode()
    : Node { "carsimnode", "/mad/car0" }
  {
    boost::log::core::get()->set_filter(
            boost::log::trivial::severity >= boost::log::trivial::info
            );
    readParameters();
//    diagUpdater.setHardwareID(get_name());
//    diagUpdater.add("CheckpointSequence", this, &CarSimNode::diagCheckpointSequence);
    rclcpp::QoS qos { rclcpp::KeepLast(1) };
    qos.best_effort().durability_volatile();
    rclcpp::QoS qosReliable { rclcpp::KeepLast(1) };
    qosReliable.reliable();
    
    subInputs = create_subscription<mbmadmsgs::msg::CarInputs>(
      "carinputs", qos, std::bind(&CarSimNode::carInputsCallback, this, std::placeholders::_1));
    pubOutputsExt = create_publisher<mbmadmsgs::msg::CarOutputsExt>("caroutputsext", qosReliable);
    task.start();
  }

  void init()
  {
    system.init();
  }

  virtual ~CarSimNode()
  {
    task.stop();
  }

private:
  void carInputsCallback(const mbmadmsgs::msg::CarInputs::SharedPtr msg)
  {
    u.at(0) = msg->pedals;
    u.at(1) = msg->steering;
    //task.triggerStep(static_cast<int64_t>(dt * 1.5 * 1.0e6)); // switch to receive mode, fallback: timer activates in 30ms
  }

  void step()
  {
    mbmad::CarPlant::OutputsType y;
    system.step(u, y);
    msgOutputsExt.time = this->now();
    msgOutputsExt.carid = static_cast<uint8_t>(CarParameters::p()->carid);
    msgOutputsExt.s.at(0) = y.at(0);
    msgOutputsExt.s.at(1) = y.at(1);
    msgOutputsExt.psi = y.at(2);
    msgOutputsExt.beta = y.at(3);
    msgOutputsExt.v = y.at(4);
    msgOutputsExt.x = y.at(5);
    msgOutputsExt.prob = 1.0F;
    pubOutputsExt->publish(msgOutputsExt);
  }

//  void diagCheckpointSequence(diagnostic_updater::DiagnosticStatusWrapper& stat)
//  {
//    cpSeq.health().diag(stat);
//  }

  void readParameters()
  {
    CarParameters::p()->setCarid(this->get_namespace());
    x0 = declare_parameter<std::vector<double>>("x0", x0);
    if (x0.size() == CarParameters::p()->x0.size()) {
      for (std::size_t idx = 0; idx < x0.size(); ++idx) {
        CarParameters::p()->x0.at(idx) = static_cast<float>(x0.at(idx));
      }
    }
  }

  static constexpr double dt = static_cast<double>(CarParameters::Ta);
  static constexpr int64_t dtMicro = static_cast<int64_t>(dt * 1e6);
//  diagnostic_updater::Updater diagUpdater { get_node_base_interface(),
//        get_node_logging_interface(), get_node_parameters_interface(),
//        get_node_timers_interface(), get_node_topics_interface(),
//        1.0 };
  rclcpp::Subscription<mbmadmsgs::msg::CarInputs>::SharedPtr subInputs;
  rclcpp::Publisher<mbmadmsgs::msg::CarOutputsExt>::SharedPtr pubOutputsExt;
  mbsafe::Task<1000UL, 91> task { std::bind(&CarSimNode::step, this), dtMicro };
  CarPlant system;
  CarPlant::InputsType u;
  std::vector<double> x0;

  mbmadmsgs::msg::CarOutputsExt msgOutputsExt;
};

}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  if (mbsafe::Platform::init(1000ULL, false, false)
        && mbsafe::Platform::setPrio(SCHED_FIFO, 90)
        && mbsafe::Platform::prefaultStack<1000ULL>()) {
    {
      std::shared_ptr<mbmad::CarSimNode> node = std::make_shared<mbmad::CarSimNode>();
      node->init();
      rclcpp::spin(node);
    }
    mbsafe::Platform::exit();
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
