/**
  * @brief ROS2 node carctrlnode for motion control and planning
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
#include <iostream>
#include <mutex>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include "mbsafe/Platform.hpp"
#include "mbsafe/CheckpointSequence.hpp"
#include "mbsafe/CheckpointDeadlineMonitor.hpp"
#include "mbsafe/CheckpointJitterMonitor.hpp"
#include "mbsafe/Task.hpp"
#include "mbmadmsgs/msg/car_inputs.hpp"
#include "mbmadmsgs/msg/car_outputs_ext_list.hpp"
#include "mbmadmsgs/msg/drive_maneuver.hpp"
#include "mbmadmsgs/msg/ctrl_reference.hpp"
#include "mbmadmsgs/srv/track_get_waypoints.hpp"
#include "mbmadmsgs/srv/reset_lapcounter.hpp"
#ifdef MAD24
#include "mbmad/CarParameters24.hpp"
#else
#include "mbmad/CarParameters.hpp"
#endif
#include "mbmad/CheckpointGraph.hpp"
#include "SpeedController.hpp"
#include "PositionController.hpp"
#include "PathController.hpp"
#include "Acc.hpp"
#include "LapCounter.hpp" 
#include "OperationModeFsm.hpp"


namespace mbmad
{

using namespace std::placeholders;
using namespace std::chrono_literals;

/**
* @brief The CarCtrlNode C++ class
*/
class CarCtrlNode : public rclcpp::Node
{
public:
  /**
   * @brief The only constructor
   */
  CarCtrlNode() 
  : Node { "carctrlnode", "/mad/car0" }, speedController { *this }, pathController { *this }, acc {*this}
  {
    cpSeq.registerMonitor(cpJitterMonReceive);
    cpSeq.registerMonitor(cpDeadlineMonPublish);
    cpSeq.registerMonitor(cpDeadlineMonReceivePublish);
    cpSeq.registerMonitor(cpGraphMon);
    diagUpdater.setHardwareID(get_name());
    diagUpdater.add("CheckpointSequence", this, &CarCtrlNode::diagCheckpointSequence);
  }

  bool init()
  {

    boost::log::core::get()->set_filter(
            boost::log::trivial::severity >= boost::log::trivial::error
            );
    CarParameters::p()->setCarid(this->get_namespace());
    rclcpp::QoS qos { rclcpp::KeepLast(1) };
    qos.best_effort().durability_volatile();
    rclcpp::QoS qosReliable { rclcpp::KeepLast(1) };
    qosReliable.reliable();
    subManeuver = create_subscription<mbmadmsgs::msg::DriveManeuver>(
      "maneuver", qosReliable, std::bind(&CarCtrlNode::maneuverCallback, this, std::placeholders::_1));
    subManeuverPass = create_subscription<mbmadmsgs::msg::DriveManeuver>(
      "maneuverpass", qosReliable, std::bind(&CarCtrlNode::maneuverPassCallback, this, std::placeholders::_1));
    subOutputsExtList = create_subscription<mbmadmsgs::msg::CarOutputsExtList>(
      "/mad/locate/caroutputsext", qos, std::bind(&CarCtrlNode::outputsExtListCallback, this, std::placeholders::_1));
    subJoy = create_subscription<sensor_msgs::msg::Joy>(
      "joy", qosReliable, std::bind(&CarCtrlNode::joyCallback, this, std::placeholders::_1));
    pubInputs = create_publisher<mbmadmsgs::msg::CarInputs>("carinputs", qos); // 
    pubCtrlReference = create_publisher<mbmadmsgs::msg::CtrlReference>("ctrlreference", qos);   
    srvResetLapcounter = create_service<mbmadmsgs::srv::ResetLapcounter>("reset_lapcounter", std::bind(&CarCtrlNode::resetLapcounter, this, _1, _2, _3));
    task.start();
    return true;
  }

  bool exit()
  {
    mbmadmsgs::msg::CarInputs carInputsMsg;
    carInputsMsg.carid = static_cast<uint8_t>(CarParameters::p()->carid);
    carInputsMsg.cmd = carInputsMsg.CMD_HALT;
    carInputsMsg.pedals = 0.0F;
    carInputsMsg.steering = 0.0F;
    pubInputs->publish(carInputsMsg);
    return true;
  }


private:
  mbsafe::CheckpointSequence cpSeq { *this, CheckpointGraph::lastCheckpoint };
  mbsafe::CheckpointJitterMonitor cpJitterMonReceive { *this, cpSeq,
                                    CheckpointGraph::Checkpoint::CtrlStart,
                                    0.5 * static_cast<double>(mbmad::CarParameters::p()->Tva),
                                    1.5 * static_cast<double>(mbmad::CarParameters::p()->Tva) };
  mbsafe::CheckpointDeadlineMonitor cpDeadlineMonPublish { *this, cpSeq,
                                    CheckpointGraph::Checkpoint::LocatePublish,
                                    CheckpointGraph::Checkpoint::CtrlPublish,
                                    0.0, CheckpointGraph::maxDeadline };
  mbsafe::CheckpointDeadlineMonitor cpDeadlineMonReceivePublish { *this, cpSeq,
                                    CheckpointGraph::Checkpoint::CtrlStart,
                                    CheckpointGraph::Checkpoint::CtrlPublish,
                                    0.0, CheckpointGraph::maxDeadline };
  mbsafe::CheckpointGraphMonitor cpGraphMon { *this, CheckpointGraph::graph };
  diagnostic_updater::Updater diagUpdater { this, 1.0 };
  rclcpp::Subscription<mbmadmsgs::msg::CarOutputsExtList>::SharedPtr subOutputsExtList;
  rclcpp::Subscription<mbmadmsgs::msg::DriveManeuver>::SharedPtr subManeuver;
  rclcpp::Subscription<mbmadmsgs::msg::DriveManeuver>::SharedPtr subManeuverPass;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subJoy;
  rclcpp::Publisher<mbmadmsgs::msg::CarInputs>::SharedPtr pubInputs;
  rclcpp::Publisher<mbmadmsgs::msg::CtrlReference>::SharedPtr pubCtrlReference;
  rclcpp::Service<mbmadmsgs::srv::ResetLapcounter>::SharedPtr srvResetLapcounter;
  
  
  SpeedController speedController; /**< The speed controller */
  PositionController positionController; /**< The position controller */
  PathController pathController; /**< The path controller */
  Acc acc; /**< The ACC controller */
  LapCounter lapCounter { *this }; /**< The lap counter */
  OperationModeFsm opModeFsm; /**< Operation Mode Management */
  std::array<float, 2> s { { } }; /**< The current car position */
  std::array<Spline, 2> splines; /**< The current path splines */
  float prob { 0.0F }; /**< The current car location probability */
  float psi { 0.0F }; /**< The current car yaw angle */
  float v { 0.0F }; /**< The current car speed */
  float x { 0.0F }; /**< The current longitudinal position */

  float xref { 0.0F }; /**< The car reference position */
  float vmax { 0.0F }; /**< The car max. speed from maneuver */
  float vref { 0.0F }; /**< The car reference speed */
  mbmadmsgs::msg::CarOutputsExtList carOutputsExtList; /**< The car outputs list message */
  mbmadmsgs::msg::DriveManeuver::_type_type maneuverType { mbmadmsgs::msg::DriveManeuver::TYPE_HALT };

  const float joySteeringAxis = declare_parameter<int64_t>("joySteeringAxis", 2); /**< The joystick steering axis */
  const float joySpeedAxis = declare_parameter<int64_t>("joySpeedAxis", 1); /**< The joystick steering axis */
  const float joySteeringLevel = declare_parameter<float>("joySteeringLevel", 1.0F); /**< The joystick control overlay level [ 0 ... 1 ]*/
  const float joySpeedMax = declare_parameter<float>("joySpeedMax", 0.5F); /**< The joystick maximal speed setpoint [ m/s ] */
  const float joySpeedPassive = declare_parameter<float>("joySpeedPassive", 0.7F); /**< The joystick speed level to detect passive player and reduce steering control [ 0 ... 1 ]*/
  sensor_msgs::msg::Joy joyMsg; /**< The current joystick message */
  const rclcpp::Duration joyTimeout = 200ms; /**< The joystick timeout */

  static constexpr float dt = static_cast<float>(CarParameters::Tva);
  static constexpr int64_t dtMicro = static_cast<int64_t>(dt * 1e6);
  mbsafe::Task<100000UL, 91> task { std::bind(&CarCtrlNode::step, this), dtMicro };

  std::mutex mutexStep;

  /**
  * @brief callback for /mad/caroutputsext topic
  * @param[in] msg The ROS message
  */
  void outputsExtListCallback(const mbmadmsgs::msg::CarOutputsExtList::SharedPtr msgList)
  { 
    carOutputsExtList = *msgList;
    cpSeq.init(msgList->cpseq);
    cpSeq.update(CheckpointGraph::Checkpoint::CtrlStart);
    for (auto& msg : msgList->list) {
      if (msg.carid == CarParameters::p()->carid) {
        prob = msg.prob;
        s.at(0) = msg.s.at(0);
        s.at(1) = msg.s.at(1);
        psi = msg.psi;
        v = msg.v;
        break;
      }
    }
    task.triggerStep(static_cast<int64_t>(dt * 1.5F * 1.0e6));
  }

  /**
   * @brief callback for /mad/car?/maneuver topic
   * @param[in] msg The ROS message
   */
  void maneuverCallback(const mbmadmsgs::msg::DriveManeuver::SharedPtr msg)
  {
    std::unique_lock<std::mutex> lock(mutexStep);
    splines.at(0) = Spline(msg->breaks, msg->s1, msg->s2, msg->segments, false);
    maneuverType = msg->type;
    xref = msg->xref;
    vmax = msg->vmax;
    positionController.init(vmax, xref, x);
    pathController.init();
    acc.init();
  }

  /**
   * @brief callback for /mad/car?/maneuverpass topic
   * @param[in] msg The ROS message for passing maneuvers
   */
  void maneuverPassCallback(const mbmadmsgs::msg::DriveManeuver::SharedPtr msg)
  {
    std::unique_lock<std::mutex> lock(mutexStep);
    if (splines.at(1).breaks.empty()) {      
      splines.at(1) = Spline(msg->breaks, msg->s1, msg->s2, msg->segments, false);    
    }
  }

  /**
   * @brief callback for /mad/car?/joy topic
   * @param[in] msg The ROS message for joystick control
   */
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    joyMsg = *msg;
  }

  void resetLapcounter(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const mbmadmsgs::srv::ResetLapcounter::Request::SharedPtr request,
    mbmadmsgs::srv::ResetLapcounter::Response::SharedPtr response)
  {
    (void)request_header; // unused param
    (void)request; // unused param
    (void)response; // unused param
    lapCounter.reset();
  }

  /**
   * @brief step function to execute one sampling step
   */
  void step(){

    std::unique_lock<std::mutex> lock(mutexStep);
    float pedals { 0.0F };
    float steering { 0.0F };

    if (prob >= 1.0F) {
      opModeFsm.dispatch<EventLocationGained>(EventLocationGained{});
    } 
    else if (prob <= 0.0F) {
      opModeFsm.dispatch<EventLocationLost>(EventLocationLost{});
    }
    else {
      opModeFsm.dispatch<EventLocationDegraded>(EventLocationDegraded{});
    }

    mbmadmsgs::msg::CarInputs carInputsMsg;
    carInputsMsg.cmd = carInputsMsg.CMD_HALT;
    float ey { 0.0F };
    float wkappa { 0.0F };
    steering = pathController.step(splines.at(acc.getLane()), opModeFsm, s, psi, v, ey, wkappa);

    // subtract joystick timestamp to from current timestamp
    rclcpp::Duration joyTimeDiff = this->now() - rclcpp::Time(joyMsg.header.stamp);
    if (joyTimeDiff <= joyTimeout  && joyMsg.axes.size() > joySteeringAxis && joyMsg.axes.size() > joySpeedAxis) {
      if (joyMsg.axes.at(joySpeedAxis) < joySpeedPassive) { // if player is passive, only thrusts and does not need to steer
        steering += joyMsg.axes.at(joySteeringAxis);
      } else {
        steering = (1.0F - joySteeringLevel) * steering + joySteeringLevel * joyMsg.axes.at(joySteeringAxis); // reduce control overlay at higher speed
      }
      if (steering > 1.0F) {
        steering = 1.0F;
      } else if (steering < -1.0F) {
        steering = -1.0F;
      }
    }

    if (maneuverType == mbmadmsgs::msg::DriveManeuver::TYPE_PATHFOLLOW) {
#ifdef MAD24
      vref = vmax;
#else
      if (joyTimeDiff <= joyTimeout && joyMsg.axes.size() > joySpeedAxis) {
        vref = joyMsg.axes.at(joySpeedAxis) * joySpeedMax;
      } 
      else {
        // float leadDist { 0.0F };
        // float leadV { 0.0F };
        // float otherDist { 0.0F };
        // bool faultLateral { false };
        // bool leadFaultLateral { false };
        acc.step(vmax, splines, carOutputsExtList, v, pathController.wx, ey, wkappa, vref);
      }
#endif
      pedals = speedController.step(opModeFsm, vref, v);
      if (vref >= 0.0F) {
        carInputsMsg.cmd = carInputsMsg.CMD_FORWARD;
      } 
      else {
        carInputsMsg.cmd = carInputsMsg.CMD_REVERSE;
      }
      lapCounter.step(splines.at(acc.getLane()), acc.getLane(), pathController.wx, opModeFsm);
    } else if (maneuverType == mbmadmsgs::msg::DriveManeuver::TYPE_PARK) {
      float vrefPos = positionController.step(x, dt);
      pedals = speedController.step(opModeFsm, vrefPos, v);
      carInputsMsg.cmd = carInputsMsg.CMD_SLOW;
    }
    
    // Send carinputs message
    carInputsMsg.carid = static_cast<uint8_t>(CarParameters::p()->carid);
    carInputsMsg.opmode = opModeFsm.getStateId();
    carInputsMsg.pedals = pedals;
    carInputsMsg.steering = steering;
    
    if (cpSeq.empty() == false) {
      // avoid invalid deadline measurements when there is no camera data (i.e. during startup)
      cpSeq.update(CheckpointGraph::Checkpoint::CtrlPublish);
      carInputsMsg.cpseq = cpSeq.message();
    }
#ifdef XXXXXXXX // too restrictive
    if (cpSeq.health().health() == false) {
      // emergency halt
      carInputsMsg.cmd = carInputsMsg.CMD_HALT;
      carInputsMsg.pedals = 0.0F;
      // heal immediately
      cpSeq.health().recover();
      BOOST_LOG_TRIVIAL(error) << "[ERROR] no health -> emergency halt";
    }
#endif
    pubInputs->publish(carInputsMsg);

    mbmadmsgs::msg::CtrlReference ctrlReferenceMsg;
    ctrlReferenceMsg.carid = static_cast<uint8_t>(CarParameters::p()->carid);
    ctrlReferenceMsg.s.at(0) = pathController.ws.at(0);
    ctrlReferenceMsg.s.at(1) = pathController.ws.at(1);
    ctrlReferenceMsg.psi = pathController.wpsi;
    ctrlReferenceMsg.v = vref;
    ctrlReferenceMsg.x = pathController.wx;
    ctrlReferenceMsg.crashctr = lapCounter.getCrashCtr();
    ctrlReferenceMsg.lapctr = lapCounter.getLapCtr();
    ctrlReferenceMsg.laptime = lapCounter.getLapTime();
    ctrlReferenceMsg.avgspeed = lapCounter.getAvgSpeed();
    ctrlReferenceMsg.currentlaptime = lapCounter.getCurrentLapTime();
    ctrlReferenceMsg.prob = prob;
    pubCtrlReference->publish(ctrlReferenceMsg);
  }

  void diagCheckpointSequence(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    cpSeq.health().diag(stat);
  }

};

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  if (mbsafe::Platform::init(1000ULL, false, false)
        && mbsafe::Platform::setPrio(SCHED_FIFO, 90)
        && mbsafe::Platform::prefaultStack<1000ULL>()) {
    std::shared_ptr<mbmad::CarCtrlNode> node = std::make_shared<mbmad::CarCtrlNode>();
    if (node->init()) {
      rclcpp::spin(node);
      node->exit();
    }
    mbsafe::Platform::exit();
  }
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}

