/**
  * @brief ROS2 node visionsimnode for simulating the camera in SIL
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
#include "mbsafe/CheckpointSequence.hpp"
#include "mbsafe/CheckpointDeadlineMonitor.hpp"
#include "mbsafe/CheckpointJitterMonitor.hpp"
#include "mbsafe/Task.hpp"
#include "mbmadmsgs/msg/car_outputs_list.hpp"
#include "mbmadmsgs/msg/car_outputs_ext.hpp"
#ifdef MAD24
#include "mbmad/CarParameters24.hpp"
#else
#include "mbmad/CarParameters.hpp"
#endif
#include "mbmad/CheckpointGraph.hpp"

namespace mbmad
{

class VisionSimNode : public rclcpp::Node
{
public:
  VisionSimNode()
    : Node { "visionsimnode", "/mad/vision" }
  {
    boost::log::core::get()->set_filter(
            boost::log::trivial::severity >= boost::log::trivial::info
            );
    cpSeq.registerMonitor(cpJitterMonReceive);
    cpSeq.registerMonitor(cpDeadlineMonPublish);
    cpSeq.registerMonitor(cpGraphMon);
//    diagUpdater.setHardwareID(get_name());
//    diagUpdater.add("CheckpointSequence", this, &CarSimNode::diagCheckpointSequence);
    rclcpp::QoS qos { rclcpp::KeepLast(1) };
    qos.best_effort().durability_volatile();
    rclcpp::QoS qosReliable { rclcpp::KeepLast(1) };
    qosReliable.reliable();
    pubOutputsList = create_publisher<mbmadmsgs::msg::CarOutputsList>("caroutputs", qos);
    msgOutputsList.list.resize(CarParameters::p()->carCnt);
    for (uint32_t id = 0U; id < CarParameters::p()->carCnt; ++id) {
      msgOutputsList.list.at(id).carid = id;
      subOutputsExtList.push_back(
            create_subscription<mbmadmsgs::msg::CarOutputsExt>("/mad/car" +  std::to_string(id) + "/caroutputsext",
                                                           qosReliable,
                                                           std::bind(&VisionSimNode::carOutputsExtCallback, this, std::placeholders::_1)));
    }
    task.start();
  }

  virtual ~VisionSimNode()
  {
    task.stop();
  }

private:
  void carOutputsExtCallback(const mbmadmsgs::msg::CarOutputsExt::SharedPtr msg)
  {
    if (msg->carid < CarParameters::p()->carCnt) {
      msgOutputsList.list.at(msg->carid).carid = msg->carid;
      msgOutputsList.list.at(msg->carid).s = msg->s;
      msgOutputsList.list.at(msg->carid).psi = msg->psi;
      msgOutputsList.list.at(msg->carid).prob = 1.0F;
    }
    // for (uint32_t id = 0U; id < CarParameters::p()->carCnt; ++id) {
    //   msgOutputsList.list.at(id).prob -= 0.1F;
    //   if (msgOutputsList.list.at(id).prob < 0.0F) {
    //     msgOutputsList.list.at(id).prob = 0.0F;
    //   }
    // }
  }

  void step()
  {
    cpSeq.init(CheckpointGraph::lastCheckpoint);
    cpSeq.update(CheckpointGraph::Checkpoint::CameraStart, seqCtr);
    cpSeq.update(CheckpointGraph::Checkpoint::CameraPublish);
    cpSeq.update(CheckpointGraph::Checkpoint::VisionStart);
    cpSeq.update(CheckpointGraph::Checkpoint::VisionPublish);
    msgOutputsList.cpseq = cpSeq.message();
    pubOutputsList->publish(msgOutputsList);
    ++seqCtr;
  }

//  void diagCheckpointSequence(diagnostic_updater::DiagnosticStatusWrapper& stat)
//  {
//    cpSeq.health().diag(stat);
//  }

  static constexpr double dt = static_cast<double>(CarParameters::Tva);
  static constexpr int64_t dtMicro = static_cast<int64_t>(dt * 1e6);
//  diagnostic_updater::Updater diagUpdater { get_node_base_interface(),
//        get_node_logging_interface(), get_node_parameters_interface(),
//        get_node_timers_interface(), get_node_topics_interface(),
//        1.0 };
  rclcpp::Publisher<mbmadmsgs::msg::CarOutputsList>::SharedPtr pubOutputsList;
  std::vector<rclcpp::Subscription<mbmadmsgs::msg::CarOutputsExt>::SharedPtr> subOutputsExtList;
  mbsafe::CheckpointSequence cpSeq { *this, CheckpointGraph::lastCheckpoint };
  mbsafe::CheckpointJitterMonitor cpJitterMonReceive { *this, cpSeq,
                                    CheckpointGraph::Checkpoint::VisionStart,
                                    static_cast<double>(mbmad::CarParameters::p()->Tva) - 1.0e-3,
                                    static_cast<double>(mbmad::CarParameters::p()->Tva) + 1.0e-3 };
  mbsafe::CheckpointDeadlineMonitor cpDeadlineMonPublish { *this, cpSeq,
                                    CheckpointGraph::Checkpoint::VisionStart,
                                    CheckpointGraph::Checkpoint::VisionPublish,
                                    0.0, 1.0e-3 };
  mbsafe::CheckpointGraphMonitor cpGraphMon { *this, CheckpointGraph::graph };
  mbsafe::Task<1000UL, 91> task { std::bind(&VisionSimNode::step, this), dtMicro };
  uint64_t seqCtr { 0ULL };

  mbmadmsgs::msg::CarOutputsList msgOutputsList;
};

}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  if (mbsafe::Platform::init(1000ULL, false, false)
        && mbsafe::Platform::setPrio(SCHED_FIFO, 90)
        && mbsafe::Platform::prefaultStack<1000ULL>()) {
    rclcpp::spin(std::make_shared<mbmad::VisionSimNode>());
    mbsafe::Platform::exit();
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
