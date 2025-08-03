/**
  * @brief ROS2 node LocateNode for multi-object tracking
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
#include <unordered_map>
#include <thread>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include "mbsafe/Platform.hpp"
#include "mbsafe/CheckpointSequence.hpp"
#include "mbsafe/CheckpointDeadlineMonitor.hpp"
#include "mbsafe/CheckpointJitterMonitor.hpp"
#include "mbsafe/CheckpointCounterMonitor.hpp"
#include "mbmad/CheckpointGraph.hpp"
#include "mbsafe/Task.hpp"
#include "mbmadmsgs/msg/car_outputs_list.hpp"
#include "mbmadmsgs/msg/car_outputs_ext_list.hpp"
#include "mbmadmsgs/msg/car_obs_list.hpp"
#include "mbmadmsgs/srv/track_get_waypoints.hpp"
#include "mbmad/CarParameters.hpp"
#include "mbmad/Spline.hpp"
#include "Car.hpp"

namespace mbmad
{

using namespace std::chrono_literals;
const std::string namespaceDefault { "/mad/locate" };

/**
* @brief The LocateNode C++ class
*/
class LocateNode : public rclcpp::Node
{
public:
  /**
   * @brief The only constructor which is called on class instantiation
   */
  LocateNode()
    : Node { "locatenode", namespaceDefault }
  {
    cpSeq.registerMonitor(cpJitterMonCameraAcquisition);
    cpSeq.registerMonitor(cpJitterMonReceive);
    cpSeq.registerMonitor(cpCounterMonReceive);
    cpSeq.registerMonitor(cpDeadlineMonPublish);
    cpSeq.registerMonitor(cpGraphMon);
    diagUpdater.setHardwareID(get_name());
    diagUpdater.add("CheckpointSequence", this, &LocateNode::diagCheckpointSequence);
    diagUpdater.add("cars", this, &LocateNode::diagCar);
    msgExtList.list.resize(mbmad::CarParameters::p()->carCnt);
    msgObsList.list.resize(mbmad::CarParameters::p()->carCnt);
  }

  bool init()
  {
    boost::log::core::get()->set_filter(
        boost::log::trivial::severity >= boost::log::trivial::error
        );
    rclcpp::QoS qos { rclcpp::KeepLast(1) };
    //qos.reliable();
    qos.best_effort().durability_volatile();
    std::string namespaceTopic { "/mad/vision" };
    std::string name = this->get_namespace();
    if (name != namespaceDefault && name.length() > namespaceDefault.length()) {
        namespaceTopic = namespaceTopic + name.substr(namespaceDefault.length());
    }
    pubOutputsExt = create_publisher<mbmadmsgs::msg::CarOutputsExtList>("caroutputsext", qos);
    pubObs = create_publisher<mbmadmsgs::msg::CarObsList>("carobs", qos);
    clientWaypoints = create_client<mbmadmsgs::srv::TrackGetWaypoints>("/mad/get_waypoints");

    bool success = getSpline(0.5F, 0.01F, true, spline);
    if (!success) {
      RCLCPP_ERROR(get_logger(), "failed to get center spline");
      return false;
    }

    subOutputs = create_subscription<mbmadmsgs::msg::CarOutputsList>(
      namespaceTopic + "/caroutputs", qos, std::bind(&LocateNode::outputsCallback, this, std::placeholders::_1));
    
    //task.start();
    return true;
  }

private:
  mbsafe::CheckpointSequence cpSeq { *this, CheckpointGraph::lastCheckpoint };
  mbsafe::CheckpointJitterMonitor cpJitterMonCameraAcquisition { *this, cpSeq,
                                    CheckpointGraph::Checkpoint::CameraAcquisition,
                                    0.5 * static_cast<double>(mbmad::CarParameters::p()->Tva),
                                    1.5 * static_cast<double>(mbmad::CarParameters::p()->Tva), 0, 1 };
  mbsafe::CheckpointJitterMonitor cpJitterMonReceive { *this, cpSeq,
                                    CheckpointGraph::Checkpoint::LocateStart,
                                    0.5 * static_cast<double>(mbmad::CarParameters::p()->Tva),
                                    1.5 * static_cast<double>(mbmad::CarParameters::p()->Tva) };
  mbsafe::CheckpointCounterMonitor cpCounterMonReceive { *this, cpSeq,
                                    CheckpointGraph::Checkpoint::LocateStart };
  mbsafe::CheckpointDeadlineMonitor cpDeadlineMonPublish { *this, cpSeq,
                                    CheckpointGraph::Checkpoint::VisionStart,
                                    CheckpointGraph::Checkpoint::LocatePublish,
                                    0.0, 49.0e-3 };
  mbsafe::CheckpointGraphMonitor cpGraphMon { *this, CheckpointGraph::graph };
  diagnostic_updater::Updater diagUpdater { this, 1.0 };
  rclcpp::Subscription<mbmadmsgs::msg::CarOutputsList>::SharedPtr subOutputs;
  rclcpp::Publisher<mbmadmsgs::msg::CarOutputsExtList>::SharedPtr pubOutputsExt;
  rclcpp::Publisher<mbmadmsgs::msg::CarObsList>::SharedPtr pubObs;
  rclcpp::CallbackGroup::SharedPtr callbackGroup;
  rclcpp::Client<mbmadmsgs::srv::TrackGetWaypoints>::SharedPtr clientWaypoints;
  std::shared_ptr<mbmad::Spline> spline;

//  static constexpr double dt = static_cast<double>(CarParameters::Tva);
//  static constexpr int64_t dtMicro = static_cast<int64_t>(dt * 1e6);
//  mbsafe::Task<1000UL, 91, dtMicro> task { std::bind(&LocateNode::step, this) };

  using CarMap = std::unordered_map<uint8_t, Car>;
  CarMap carMap;

  mbmadmsgs::msg::CarOutputsExtList msgExtList;
  mbmadmsgs::msg::CarObsList msgObsList;


  /**
  * @brief callback for /mad/vision/caroutputslist topic
  * @param[in] msg The ROS message
  */
  void outputsCallback(const mbmadmsgs::msg::CarOutputsList::SharedPtr msgList)
  {
    cpSeq.init(msgList->cpseq);
    rclcpp::Time camTime(cpSeq.at(CheckpointGraph::Checkpoint::CameraAcquisition).time);
    cpSeq.update(CheckpointGraph::Checkpoint::LocateStart);
    for (auto& msg : msgList->list) {
      if (msg.prob >= 1.0F) {
        auto car = carMap.find(msg.carid);
        if (car == carMap.end()) {
          // create new car
          carMap.emplace(msg.carid, msg);
        }
        // update existing car
        carMap.at(msg.carid).update(msgList->cpseq.cpseq.at(msgList->cpseq.cur_cpid).seqctr, msg, 
          camTime, spline);        
      }
    }
    // degrade if car has not been detected and publish detected and undetected cars    
    for (auto& car : carMap) {
      car.second.updateVerify(msgList->cpseq.cpseq.at(msgList->cpseq.cur_cpid).seqctr);
      mbmadmsgs::msg::CarOutputsExt msgExt = car.second.message();
      mbmadmsgs::msg::CarObs msgObs = car.second.messageObs();
      if (msgExt.carid < msgExtList.list.size()) {
        msgExtList.list.at(msgExt.carid) = msgExt;
        msgObsList.list.at(msgExt.carid) = msgObs;
      }
    }
    cpSeq.update(CheckpointGraph::Checkpoint::LocatePublish);
    msgExtList.cpseq = cpSeq.message();
    pubOutputsExt->publish(msgExtList);
    msgObsList.cpseq = cpSeq.message();
    pubObs->publish(msgObsList);
  }


  /**
   * @brief step function to execute one sampling step
   */
  void step()
  {
  }


  void diagCar(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    for (auto& car : carMap) {
      if (car.second.diagErrMsg != "") {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "new car location error");
      } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "no car location error");
      }
      stat.add("LastErrorMessage", car.second.diagErrMsg);
      stat.addf("ErrorCount", "%u", car.second.diagErrCnt);
    }
  }

  void diagCheckpointSequence(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    cpSeq.health().diag(stat);
  }

  bool getSpline(const float alpha, const float dx, const bool periodic, std::shared_ptr<Spline>& spline)
  {
    while (!clientWaypoints->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service /mad/get_waypoints. Exiting.");
        return false;
      }
      RCLCPP_INFO(get_logger(), "service /mad/get_waypoints not available, waiting again...");
    }
    mbmadmsgs::srv::TrackGetWaypoints::Request::SharedPtr req =
        std::make_shared<mbmadmsgs::srv::TrackGetWaypoints::Request>();
    req->segment_sequence = { 0 };
    req->alpha = alpha;
    req->dx = dx;

    auto result = clientWaypoints->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto resp = result.get();
      spline = std::make_shared<Spline>(resp->breaks, resp->s1, resp->s2, resp->segments, periodic);
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to call service /mad/get_waypoints");
      return false;
    }
    return true;
  }
};

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  if (mbsafe::Platform::init(1000ULL, false, false)
        && mbsafe::Platform::setPrio(SCHED_FIFO, 90)
        && mbsafe::Platform::prefaultStack<1000ULL>()) {
    std::shared_ptr<mbmad::LocateNode> node = std::make_shared<mbmad::LocateNode>();
    if (node->init()) {
      rclcpp::spin(node);
    }
    mbsafe::Platform::exit();
  }
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}

