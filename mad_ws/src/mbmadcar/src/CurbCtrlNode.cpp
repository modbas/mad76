/**
  * MODBAS
  * madcar
  * 
  * CurbCtrlNode
  * from Robin Stumberger
  * 
  * Copyright (C) 2019, Frank Traenkle, http://www.modbas.de
  * 
  */

// all includes
#include <memory>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include "rclcpp/rclcpp.hpp"
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
#ifdef MAD24
#include "mbmad/CarParameters24.hpp"
#else
#include "mbmad/CarParameters.hpp"
#endif
#include "mbmad/CheckpointGraph.hpp"
#include "SpeedController.hpp"
#include "PositionController.hpp"
#include "PathController.hpp"
#include "PathController2.hpp"
#include "Acc.hpp"
#include "OperationModeFsm.hpp"


namespace mbmad
{

using namespace std::chrono_literals;

/**
* @brief The CurbCtrlNode C++ class
*/
class CurbCtrlNode : public rclcpp::Node
{
public:
  /**
   * @brief The only constructor
   */
  CurbCtrlNode() : Node { "CurbCtrlNode", "/mad/car0" }, pathController { *this }, pathController2{ *this}
  {
    cpSeq.registerMonitor(cpJitterMonReceive);
    cpSeq.registerMonitor(cpDeadlineMonPublish);
    cpSeq.registerMonitor(cpDeadlineMonReceivePublish);
    cpSeq.registerMonitor(cpGraphMon);
    diagUpdater.setHardwareID(get_name());
    diagUpdater.add("CheckpointSequence", this, &CurbCtrlNode::diagCheckpointSequence);
  }

  bool init()
  {
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::error);
    CarParameters::p()->setCarid(this->get_namespace());
    rclcpp::QoS qos { rclcpp::KeepLast(1) };
    qos.best_effort().durability_volatile();
    rclcpp::QoS qosReliable { rclcpp::KeepLast(1) };
    qosReliable.reliable();

    // subscriber
    subOutputsExtList = create_subscription<mbmadmsgs::msg::CarOutputsExtList>(
      "/mad/locate/caroutputsext", qos, std::bind(&CurbCtrlNode::outputsExtListCallback, this, std::placeholders::_1));

    // hier ensteht ein fehler wenn "/mad/car0/joystickinputs" --> /mad/car0/mad/car0/joystickinputs ist dann das topic 
    subJoystickInputs = create_subscription<mbmadmsgs::msg::CarInputs>(
      "joystickinputs", qos, std::bind(&CurbCtrlNode::joystickReadCallback, this, std::placeholders::_1));

    subinner_curb = create_subscription<mbmadmsgs::msg::DriveManeuver>(
      "inner_curb", qosReliable, std::bind(&CurbCtrlNode::innerCurbCallback, this, std::placeholders::_1));

    subinner_line = create_subscription<mbmadmsgs::msg::DriveManeuver>(
      "inner_line", qosReliable, std::bind(&CurbCtrlNode::innerSplineCallback, this, std::placeholders::_1));

    subouter_curb = create_subscription<mbmadmsgs::msg::DriveManeuver>(
      "outer_curb", qosReliable, std::bind(&CurbCtrlNode::outerCurbCallback, this, std::placeholders::_1));

    subouter_line = create_subscription<mbmadmsgs::msg::DriveManeuver>(
      "outer_line", qosReliable, std::bind(&CurbCtrlNode::outerSplineCallback, this, std::placeholders::_1));

    // publisher
    pubInputs = create_publisher<mbmadmsgs::msg::CarInputs>("carinputs", qos);
    pubCtrlReference = create_publisher<mbmadmsgs::msg::CtrlReference>("ctrlreference", qos);
    pubOutputsExt = this->create_publisher<mbmadmsgs::msg::CarOutputsExt>("caroutputsext", qos);
    task.start();
    return true;
  }

  // save state after exiting
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
  rclcpp::Subscription<mbmadmsgs::msg::CarInputs>::SharedPtr subJoystickInputs;

  rclcpp::Subscription<mbmadmsgs::msg::DriveManeuver>::SharedPtr subinner_curb;
  rclcpp::Subscription<mbmadmsgs::msg::DriveManeuver>::SharedPtr subinner_line;
  rclcpp::Subscription<mbmadmsgs::msg::DriveManeuver>::SharedPtr subouter_curb;
  rclcpp::Subscription<mbmadmsgs::msg::DriveManeuver>::SharedPtr subouter_line;

  rclcpp::Publisher<mbmadmsgs::msg::CarInputs>::SharedPtr pubInputs;
  rclcpp::Publisher<mbmadmsgs::msg::CtrlReference>::SharedPtr pubCtrlReference;
  rclcpp::Publisher<mbmadmsgs::msg::CarOutputsExt>::SharedPtr pubOutputsExt;

  PathController pathController; /**< The path controller */
  PathController2 pathController2; /**< The path controller */
  OperationModeFsm opModeFsm; /**< Operation Mode Management */
  std::array<float, 2> s { { } }; /**< The current car position */
  std::array<Spline, 4> splines; /**< The current path splines */
  float prob { 0.0F }; /**< The current car location probability */
  float psi { 0.0F }; /**< The current car yaw angle */
  float v { 0.0F }; /**< The current car speed */
  float x { 0.0F }; /**< The current longitudinal position */

  float joystick_pedals { 0.0F }; /**< The joystick pedals data*/
  float joystick_steering { 0.0F}; /**< The joystick steering data*/
  float joystick_carid {0}; /**< The joystick car id*/

  float pedals {0.0F}; 
  float steering {0.0F};

  float dist2RefSpline {0.0F}; /**< The distance to the reference Spline (inner or outer curb) */
  float safty_margin { 0.03F }; /**< The witdh of the catchment area */

  mbmadmsgs::msg::CarOutputsExtList carOutputsExtList; /**< The car outputs list message */
  mbmadmsgs::msg::DriveManeuver::_type_type maneuverType { mbmadmsgs::msg::DriveManeuver::TYPE_HALT };

  static constexpr double dt = static_cast<double>(CarParameters::Tva);
  static constexpr int64_t dtMicro = static_cast<int64_t>(dt * 1e6);
  mbsafe::Task<100000UL, 91> task { std::bind(&CurbCtrlNode::step, this), dtMicro };

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
        pubOutputsExt->publish(msg); // publish for measurement
        break;
      }
    }
    task.triggerStep(static_cast<int64_t>(dt * 1.5F * 1.0e6));
  }

  /**
   * @brief callback for /mad/car?/inner_curb topic
   * @param[in] msg The ROS message
   */
  void innerCurbCallback(const mbmadmsgs::msg::DriveManeuver::SharedPtr msg)
  {
    std::unique_lock<std::mutex> lock(mutexStep);
    splines.at(0) = Spline(msg->breaks, msg->s1, msg->s2, msg->segments, false);
    maneuverType = msg->type;
    // xref = msg->xref;
    // vmax = msg->vmax;
    pathController.init();
    pathController2.init();
  }

  /**
   * @brief callback for /mad/car?/inner_line topic
   * @param[in] msg The ROS message
   */
  void innerSplineCallback(const mbmadmsgs::msg::DriveManeuver::SharedPtr msg)
  {
    std::unique_lock<std::mutex> lock(mutexStep);
    splines.at(1) = Spline(msg->breaks, msg->s1, msg->s2, msg->segments, false);
    
  }
  
  /**
   * @brief callback for /mad/car?/outer_curb topic
   * @param[in] msg The ROS message
   */
  void outerCurbCallback(const mbmadmsgs::msg::DriveManeuver::SharedPtr msg)
  {
    std::unique_lock<std::mutex> lock(mutexStep);
    splines.at(2) = Spline(msg->breaks, msg->s1, msg->s2, msg->segments, false);

  }

  /**
   * @brief callback for /mad/car?/outer_line topic
   * @param[in] msg The ROS message
   */
  void outerSplineCallback(const mbmadmsgs::msg::DriveManeuver::SharedPtr msg)
  {
    std::unique_lock<std::mutex> lock(mutexStep);
    splines.at(3) = Spline(msg->breaks, msg->s1, msg->s2, msg->segments, false);

  }
 
  /**
   * @brief callback for mad/car0/joystickinputs topic
   * @param[in] msg The ROS message
   */
  void joystickReadCallback(const mbmadmsgs::msg::CarInputs::SharedPtr msg)
  {
    std::unique_lock<std::mutex> lock(mutexStep);
    joystick_pedals = msg->pedals;
    joystick_steering = msg->steering;
    joystick_carid = msg->carid;
  }

  /**
   * @brief step function to execute one sampling step
   */
  void step()
  {
    std::unique_lock<std::mutex> lock(mutexStep);


    if (prob >= 1.0F) {
      opModeFsm.dispatch<EventLocationGained>(EventLocationGained{});
    } 
    else if (prob <= 0.0F) {
      opModeFsm.dispatch<EventLocationLost>(EventLocationLost{});
    } 
    else {
      opModeFsm.dispatch<EventLocationDegraded>(EventLocationDegraded{});
    }

    // -------------------------------------------------------------------------------------------------------------------------------------------------------------
    // Logik für die CurbCtrlNode
    pedals = joystick_pedals;
    
    float ey { 0.0F };
    float wkappa { 0.0F };

    // dist to inner spline
    dist2RefSpline = pathController2.getey(splines.at(0), opModeFsm, s, v, wkappa);

    // inner curb area
    if (dist2RefSpline < safty_margin && dist2RefSpline > -1.0F){
      steering = pathController.step(splines.at(1), opModeFsm, s, psi, v, ey, wkappa);
      // RCLCPP_INFO(this->get_logger(), "inner curb dist2RefSpline: %f", dist2RefSpline);
    }

    // if dist >= middelline --> outer spline
    // 0.07m is the middelline at the smalest point
    else if(dist2RefSpline > 0.07F){
      dist2RefSpline = pathController2.getey(splines.at(2), opModeFsm, s, v, wkappa);

      // outer curb area
      if (dist2RefSpline > -1.0*safty_margin && dist2RefSpline > -1.0F){
       steering = pathController.step(splines.at(3), opModeFsm, s, psi, v, ey, wkappa);
       // RCLCPP_INFO(this->get_logger(), "outer curb dist2RefSpline: %f", dist2RefSpline);
      }

      // mittel area
      else{
        steering = joystick_steering;
        // RCLCPP_INFO(this->get_logger(), "outer curb SelfSteering  : %f", dist2RefSpline);
      }

    }
    
    // mittel area
    else{
      steering = joystick_steering;
      // RCLCPP_INFO(this->get_logger(), "inner curb Self seering: %f", dist2RefSpline);
    }
    
    // -------------------------------------------------------------------------------------------------------------------------------------------------------------
  
  
    // sending the CarInputs
    mbmadmsgs::msg::CarInputs carInputsMsg;
    carInputsMsg.carid = static_cast<uint8_t>(CarParameters::p()->carid);
    carInputsMsg.opmode = opModeFsm.getStateId();
    carInputsMsg.cmd = carInputsMsg.CMD_FORWARD;
    carInputsMsg.pedals = pedals;
    carInputsMsg.steering = steering;

    if (cpSeq.empty() == false) {
      // avoid invalid deadline measurements when there is no camera data (i.e. during startup)
      cpSeq.update(CheckpointGraph::Checkpoint::CtrlPublish);
      carInputsMsg.cpseq = cpSeq.message();
    }

    pubInputs->publish(carInputsMsg);

    mbmadmsgs::msg::CtrlReference ctrlReferenceMsg;
    ctrlReferenceMsg.carid = static_cast<uint8_t>(CarParameters::p()->carid);
    ctrlReferenceMsg.s.at(0) = pathController.ws.at(0);
    ctrlReferenceMsg.s.at(1) = pathController.ws.at(1);
    ctrlReferenceMsg.psi = pathController.wpsi;
    ctrlReferenceMsg.v = 0; //vref
    ctrlReferenceMsg.x = pathController.wx;
    pubCtrlReference->publish(ctrlReferenceMsg);
  }

  void diagCheckpointSequence(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    cpSeq.health().diag(stat);
  }

};

}


// main
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  if (mbsafe::Platform::init(1000ULL, false, false) && mbsafe::Platform::setPrio(SCHED_FIFO, 90) && mbsafe::Platform::prefaultStack<1000ULL>()) {
    std::shared_ptr<mbmad::CurbCtrlNode> node = std::make_shared<mbmad::CurbCtrlNode>();
    
    if (node->init()) {
      rclcpp::spin(node);
      node->exit();
    }
    mbsafe::Platform::exit();
  }
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}