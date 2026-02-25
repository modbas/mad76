/**
  * MODBAS
  * madcar
  * CarCtrlNode
  *
  * Copyright (C) 2019, Frank Traenkle, http://www.modbas.de
  */

#include <memory>
#include <cstdint>
#include <iostream>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include "mbsafe/Platform.hpp"
#include "mbsafe/CheckpointSequence.hpp"
#include "mbsafe/CheckpointDeadlineMonitor.hpp"
#include "mbsafe/CheckpointJitterMonitor.hpp"
#include "mbsafe/Task.hpp"
#include "mbmad/Spline.hpp"
#include "mbmadmsgs/msg/car_inputs.hpp"
#include "mbmadmsgs/msg/car_obs_list.hpp"
#include "mbmadmsgs/msg/drive_maneuver.hpp"
#include "mbmadmsgs/msg/ctrl_reference.hpp"
#include "mbmadmsgs/srv/track_get_waypoints.hpp"
#include "mbmad/CheckpointGraph.hpp"
#include "mbmad/CarParameters.hpp"
extern "C" {
#include "Motion.h"
}


namespace mbmad
{

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
    : Node { "carctrlnode", "/mad/car0" }
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
    CarParameters::p()->setCarid(this->get_namespace());
        rclcpp::QoS qos { rclcpp::KeepLast(1) };
    qos.best_effort().durability_volatile();
    rclcpp::QoS qosReliable { rclcpp::KeepLast(1) };
    qosReliable.reliable();
    subManeuver = create_subscription<mbmadmsgs::msg::DriveManeuver>(
      "maneuver", qosReliable, std::bind(&CarCtrlNode::maneuverCallback, this, std::placeholders::_1));
    subObsList = create_subscription<mbmadmsgs::msg::CarObsList>(
      "/mad/locate/carobs", qos, std::bind(&CarCtrlNode::obsListCallback, this, std::placeholders::_1));
    pubInputs = create_publisher<mbmadmsgs::msg::CarInputs>("carinputs", qos);
    pubCtrlReference = create_publisher<mbmadmsgs::msg::CtrlReference>("ctrlreference", qos);
    ::Motion_initialize();
    task.start();
    return true;
  }

  bool exit()
  {
    ::Motion_terminate();
    return true;
  }

private:
  mbsafe::CheckpointSequence cpSeq { *this, CheckpointGraph::lastCheckpoint };
  mbsafe::CheckpointJitterMonitor cpJitterMonReceive { *this, cpSeq,
                                    CheckpointGraph::Checkpoint::CtrlStart,
                                    0.5 * static_cast<double>(mbmad::CarParameters::p()->Tva),
                                    2.5 * static_cast<double>(mbmad::CarParameters::p()->Tva) };
  mbsafe::CheckpointDeadlineMonitor cpDeadlineMonPublish { *this, cpSeq,
                                    CheckpointGraph::Checkpoint::CameraStart,
                                    CheckpointGraph::Checkpoint::CtrlPublish,
                                    -CheckpointGraph::maxDeadline, CheckpointGraph::maxDeadline*2.0F };
  mbsafe::CheckpointDeadlineMonitor cpDeadlineMonReceivePublish { *this, cpSeq,
                                    CheckpointGraph::Checkpoint::CtrlStart,
                                    CheckpointGraph::Checkpoint::CtrlPublish,
                                    0.0, CheckpointGraph::maxDeadline*2.0F };
  mbsafe::CheckpointGraphMonitor cpGraphMon { *this, CheckpointGraph::graph };
  diagnostic_updater::Updater diagUpdater { this, 1.0 };
  rclcpp::Subscription<mbmadmsgs::msg::CarObsList>::SharedPtr subObsList;
  rclcpp::Subscription<mbmadmsgs::msg::DriveManeuver>::SharedPtr subManeuver;
  rclcpp::Publisher<mbmadmsgs::msg::CarInputs>::SharedPtr pubInputs;
  rclcpp::Publisher<mbmadmsgs::msg::CtrlReference>::SharedPtr pubCtrlReference;

  static constexpr double dt = static_cast<double>(CarParameters::Tva);
  static constexpr int64_t dtMicro = static_cast<int64_t>(dt * 1e6);
  mbsafe::Task<100000UL, 91> task { std::bind(&CarCtrlNode::step, this), dtMicro };
  std::mutex mutexStep;

  bool carLocated { false };
  bool driveCmdRelease { false };


  /**
  * @brief callback for /mad/caroutputsext topic
  * @param[in] msg The ROS message
  */
  void obsListCallback(const mbmadmsgs::msg::CarObsList::SharedPtr msgList)
  {
    cpSeq.init(msgList->cpseq);
    cpSeq.update(CheckpointGraph::Checkpoint::CtrlStart);
    Motion_U.carid = static_cast<real_T>(CarParameters::p()->carid);
    int i = 0;
    for (auto& msg : msgList->list) {
      Motion_U.carobslist[i].s[0] = msg.s.at(0);
      Motion_U.carobslist[i].s[1] = msg.s.at(1);
      Motion_U.carobslist[i].psi = msg.psi;
      Motion_U.carobslist[i].beta = msg.beta;
      Motion_U.carobslist[i].v = msg.v;        
      Motion_U.carobslist[i].cxe = msg.cxe;
      Motion_U.carobslist[i].cx = msg.cx;
      Motion_U.carobslist[i].cxd = msg.cxd;
      Motion_U.carobslist[i].cs[0] = msg.cs.at(0);
      Motion_U.carobslist[i].cs[1] = msg.cs.at(1);
      Motion_U.carobslist[i].cey = msg.cey;
      Motion_U.carobslist[i].cepsi = msg.cepsi;
      Motion_U.carobslist[i].ckappa = msg.ckappa;
      Motion_U.carobslist[i].rey = msg.rey;
      Motion_U.carobslist[i].ley = msg.ley;
      Motion_U.carobslist[i].prob = msg.prob;
      i++;
    }
    if (Motion_U.carobslist[CarParameters::p()->carid].prob >= 1.0f) {
      carLocated = true;
    }
    task.triggerStep(static_cast<int64_t>(dt * 1.5 * 1.0e6));
  }

  /**
   * @brief callback for /mad/ctrlinputs topic
   * @param[in] msg The ROS message
   */
  void maneuverCallback(const mbmadmsgs::msg::DriveManeuver::SharedPtr msg)
  {
    driveCmdRelease = true;
  }

  /**
   * @brief step function to execute one sampling step
   */
  void step()
  {
    std::unique_lock<std::mutex> lock(mutexStep);

    if (carLocated && driveCmdRelease){
      ::Motion_step();
      mbmadmsgs::msg::CarInputs carInputsMsg;
      carInputsMsg.carid = static_cast<uint8_t>(CarParameters::p()->carid);
      carInputsMsg.opmode = carInputsMsg.OPMODE_NORMAL;
      carInputsMsg.cmd = static_cast<uint8_t>(Motion_Y.inputs.cmd);
      //carInputsMsg.cmd = carInputsMsg.CMD_FORWARD; // TODO: take from d1
      carInputsMsg.pedals = static_cast<float>(Motion_Y.inputs.pedals);
      carInputsMsg.steering = static_cast<float>(Motion_Y.inputs.steering);
      if (cpSeq.empty() == false) {
        // avoid invalid deadline measurements when there is no camera data (i.e. during startup)
        cpSeq.update(CheckpointGraph::Checkpoint::CtrlPublish);
        carInputsMsg.cpseq = cpSeq.message();
      }
      if (cpSeq.health().health() == false) {
        // emergency halt
        carInputsMsg.cmd = carInputsMsg.CMD_HALT;
        carInputsMsg.pedals = 0.0F;
        // heal immediately
        cpSeq.health().recover();
        BOOST_LOG_TRIVIAL(error) << "[ERROR] no health -> emergency halt";
      }
      if (!(Motion_U.carobslist[CarParameters::p()->carid].prob > 0.0F)) {
	      carInputsMsg.pedals = 0.0F;
      }
      pubInputs->publish(carInputsMsg);

      mbmadmsgs::msg::CtrlReference ctrlReferenceMsg;
      ctrlReferenceMsg.carid = static_cast<uint8_t>(CarParameters::p()->carid);
      ctrlReferenceMsg.s.at(0) = static_cast<float>(Motion_Y.ctrlref.s[0]);
      ctrlReferenceMsg.s.at(1) = static_cast<float>(Motion_Y.ctrlref.s[1]);
      ctrlReferenceMsg.psi = static_cast<float>(Motion_Y.ctrlref.psi);
      ctrlReferenceMsg.v = static_cast<float>(Motion_Y.ctrlref.v);
      pubCtrlReference->publish(ctrlReferenceMsg);
    }    
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
    }
    node->exit();
    mbsafe::Platform::exit();
  }
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}

