/**
  * @brief ROS2 node for animating cars in RViz
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


#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cstdint>
#include <cstdio>
#include <vector>
#include <string>
#include "mbmadmsgs/msg/car_outputs_list.hpp"
#include "mbmadmsgs/msg/car_outputs_ext_list.hpp"
#include "mbmadmsgs/msg/ctrl_reference.hpp"
#include "mbmadmsgs/msg/drive_maneuver.hpp"
//#include "mbmadmsgs/msg/drive_maneuver_state.hpp"
//#include "mbmadmsgs/msg/mission_state.hpp"
#ifdef MAD24
#include "mbmad/CarParameters24.hpp"
#else
#include "mbmad/CarParameters.hpp"
#endif

namespace mbmad {

using namespace std::chrono_literals;
const std::string namespaceDefault { "/mad/car0" };

/**
 * @brief The CarDisplayNode class
 */
class CarDisplayNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   */
  CarDisplayNode()
    : Node { "cardisplaynode", "/mad/car0" }
  {
    readParameters();
    //maneuverStateMsg.state = maneuverStateMsg.STATE_LOCATING;
    rclcpp::QoS qosRt { rclcpp::KeepLast(1) };
    qosRt.best_effort().durability_volatile();
    std::string namespaceTopicVision { "/mad/vision" };
    std::string namespaceTopicLocate { "/mad/locate" };
    std::string name = this->get_namespace();
    if (name != namespaceDefault && name.length() > namespaceDefault.length()) {
        namespaceTopicVision = namespaceTopicVision + name.substr(name.length()-1);
        namespaceTopicLocate = namespaceTopicLocate + name.substr(name.length()-1);
    }
    subOutputs = create_subscription<mbmadmsgs::msg::CarOutputsList>(
      namespaceTopicVision + "/caroutputs", qosRt, std::bind(&CarDisplayNode::carOutputsCallback, this, std::placeholders::_1));
    subOutputsExt = create_subscription<mbmadmsgs::msg::CarOutputsExtList>(
      namespaceTopicLocate + "/caroutputsext", qosRt, std::bind(&CarDisplayNode::carOutputsExtCallback, this, std::placeholders::_1));
    subCtrlReference = create_subscription<mbmadmsgs::msg::CtrlReference>(
      "ctrlreference", qosRt, std::bind(&CarDisplayNode::ctrlReferenceCallback, this, std::placeholders::_1));
    subManeuver = create_subscription<mbmadmsgs::msg::DriveManeuver>(
      "maneuver", qosRt, std::bind(&CarDisplayNode::maneuverCallback, this, std::placeholders::_1));
//    ctrlreferenceSub = node.subscribe("/mad/ctrlreference", 10, &CarDisplayNode::ctrlreferenceCallback, this);
//    for (uint32_t idx = 0U; idx < CarParameters::carCnt; ++idx) {
//      maneuverSubs.push_back(node.subscribe("/mad/car" + std::to_string(idx) + "/navi/maneuver", 10, &CarDisplayNode::maneuverCallback, this));
//      maneuverStateSubs.push_back(node.subscribe("/mad/car" + std::to_string(idx) + "/navi/maneuverstate", 10, &CarDisplayNode::maneuverStateCallback, this));
//    }
//    missionStateSub = node.subscribe("/mad/missionstate", 10, &CarDisplayNode::missionStateCallback, this);

    rclcpp::QoS qosRviz { rclcpp::KeepLast(1) };
    qosRviz.reliable();
    pubMarker = create_publisher<visualization_msgs::msg::MarkerArray>("markers", qosRviz);
    createMarkers();
    timerStep = create_wall_timer(50ms, std::bind(&CarDisplayNode::step, this));
  }

private:
  using ColorType = std::vector<double>;
  ColorType colorRGB { { 1.0, 0.4, 0.0 } }; /** marker color */
  rclcpp::Subscription<mbmadmsgs::msg::CarOutputsList>::SharedPtr subOutputs;
  rclcpp::Subscription<mbmadmsgs::msg::CarOutputsExtList>::SharedPtr subOutputsExt;
  rclcpp::Subscription<mbmadmsgs::msg::CtrlReference>::SharedPtr subCtrlReference;
  rclcpp::Subscription<mbmadmsgs::msg::DriveManeuver>::SharedPtr subManeuver;
//  std::vector<ros::Subscriber> maneuverStateSubs;
//  ros::Subscriber missionStateSub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubMarker;
  rclcpp::TimerBase::SharedPtr timerStep;
  visualization_msgs::msg::MarkerArray markerArray;
  mbmadmsgs::msg::CarOutputs outputsMsg;
  mbmadmsgs::msg::CarOutputsExt outputsExtMsg;
  mbmadmsgs::msg::CtrlReference ctrlreferenceMsg;
  mbmadmsgs::msg::DriveManeuver maneuverMsg;
//  mbmsgs::msg::DriveManeuverState maneuverStateMsg;
//  mbmsgs::msg::MissionState missionStateMsg;

  /**
   * @brief Cyclic step function
   */
  void step()
  {
    rclcpp::Time time { now() };

    // TF broadcaster
    visualization_msgs::msg::Marker::_ns_type ns = get_namespace();
    static tf2_ros::TransformBroadcaster broadcaster { *this };
    geometry_msgs::msg::TransformStamped transformMsg;
//      if (maneuverStateMsg.state == maneuverStateMsg.STATE_LOCATING) {
//      // hide car
//      outputsMsg.s.at(0) = -1.0F;
//      outputsMsg.s.at(1) = -1.0F;
//    }
    transformMsg.header.stamp = time;
    transformMsg.header.frame_id = "map";
    transformMsg.child_frame_id = ns;
    transformMsg.transform.translation.x = static_cast<double>(outputsMsg.s.at(0));
    transformMsg.transform.translation.y = static_cast<double>(outputsMsg.s.at(1));
    transformMsg.transform.translation.z = 0.0;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, static_cast<double>(outputsMsg.psi));
    transformMsg.transform.rotation.x = quat.x();
    transformMsg.transform.rotation.y = quat.y();
    transformMsg.transform.rotation.z = quat.z();
    transformMsg.transform.rotation.w = quat.w();
    broadcaster.sendTransform(transformMsg);

    // car marker
    markerArray.markers.at(0).scale.x = static_cast<double>(outputsExtMsg.v) * 0.5 * CarParameters::displayScale;
    markerArray.markers.at(2).text = carName();

    // car reference marker
    markerArray.markers.at(1).scale.x = static_cast<double>(ctrlreferenceMsg.v) * 0.5 * CarParameters::displayScale;
    markerArray.markers.at(1).pose.position.x = static_cast<double>(ctrlreferenceMsg.s.at(0));
    markerArray.markers.at(1).pose.position.y = static_cast<double>(ctrlreferenceMsg.s.at(1));
    markerArray.markers.at(1).pose.position.z = 0.0;
    quat.setRPY(0.0, 0.0, static_cast<double>(ctrlreferenceMsg.psi));
    markerArray.markers.at(1).pose.orientation.x = quat.x();
    markerArray.markers.at(1).pose.orientation.y = quat.y();
    markerArray.markers.at(1).pose.orientation.z = quat.z();
    markerArray.markers.at(1).pose.orientation.w = quat.w();

    // target segment marker
//    markerArray.markers.at(5).pose.position.x = missionStateMsg.targetSegmentS.at(0);
//    markerArray.markers.at(5).pose.position.y = missionStateMsg.targetSegmentS.at(1);

    // publish markers
    for (auto& marker : markerArray.markers) {
        marker.header.stamp = time;
    }
    pubMarker->publish(markerArray);
  }

  /**
   * @brief readParameters reads params from launch file
   */
  void readParameters()
  {
    std::string name = this->get_namespace();
    std::string nameNodeId = name.substr(0, 9);
    CarParameters::p()->setCarid(nameNodeId);
    colorRGB = declare_parameter<ColorType>("colorRGB", colorRGB);
  }

  /**
   * @brief createCarNameMarker creates name marker
   * @param marker
   * @param ns
   * @param id
   */
  void createCarNameMarker(visualization_msgs::msg::Marker& marker,
                           visualization_msgs::msg::Marker::_ns_type ns,
                           visualization_msgs::msg::Marker::_id_type id,
                           const ColorType& colorRGB)
  {
      marker.header.frame_id = ns;
      marker.header.stamp = now();
      marker.ns = ns;
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.text = carName();
      marker.pose.position.x = 0.2;
      marker.pose.position.y = 0.1;
      marker.pose.position.z = 0.1;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 0.0;
      marker.scale.x = 0.0;
      marker.scale.y = 0.0;
      marker.scale.z = 100e-3 * CarParameters::displayScale;
      marker.color.r = static_cast<float>(colorRGB[0]);
      marker.color.g = static_cast<float>(colorRGB[1]);
      marker.color.b = static_cast<float>(colorRGB[2]);
      marker.color.a = 1.0F;
      //marker.lifetime = ros::Duration();
  }

  /**
   * @brief createCarBoxMarker creates box marker
   * @param marker
   * @param ns
   * @param id
   * @param colorRGB
   */
  void createCarBoxMarker(visualization_msgs::msg::Marker& marker,
                                 visualization_msgs::msg::Marker::_ns_type ns,
                                 visualization_msgs::msg::Marker::_id_type id,
                                 const ColorType& colorRGB)
  {
//    marker.header.frame_id = ns;
//    marker.header.stamp = ros::Time::now();
//    marker.ns = ns;
//    marker.id = id;
//    marker.type = visualization_msgs::Marker::CUBE;
//    marker.action = visualization_msgs::Marker::ADD;
//    marker.text = ns;
//    marker.pose.position.x = 0.0;
//    marker.pose.position.y = 0.0;
//    marker.pose.position.z = 0.0;
//    marker.pose.orientation.x = 0.0;
//    marker.pose.orientation.y = 0.0;
//    marker.pose.orientation.z = 0.0;
//    marker.pose.orientation.w = 0.0;
//    marker.scale.x = CarParameters::p()->size[0];
//    marker.scale.y = CarParameters::p()->size[1];
//    marker.scale.z = 50e-3F;
//    marker.color.r = colorRGB[0];
//    marker.color.g = colorRGB[1];
//    marker.color.b = colorRGB[2];
//    marker.color.a = 0.5F;
//    marker.lifetime = ros::Duration();
    marker.header.frame_id = ns;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://mbmadcar/data/car_skeletal.dae";
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.text = ns;
    marker.pose.position.x = static_cast<double>(CarParameters::p()->center * CarParameters::displayScale); // rear axle position
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.707;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.707;
    const double scale = 0.0004 * CarParameters::displayScale;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.r = static_cast<float>(colorRGB[0]);
    marker.color.g = static_cast<float>(colorRGB[1]);
    marker.color.b = static_cast<float>(colorRGB[2]);
    marker.color.a = 1.0F;
    //marker.lifetime = ros::Duration();
  }

  /**
   * @brief createCarArrowMarker creates arrow marker for velocity
   * @param marker
   * @param ns
   * @param id
   * @param colorRGB
   */
   void createCarArrowMarker(visualization_msgs::msg::Marker& marker,
                                   visualization_msgs::msg::Marker::_ns_type ns,
                                   visualization_msgs::msg::Marker::_id_type id,
                                   const ColorType& colorRGB)
  {
      marker.header.frame_id = ns;
      marker.header.stamp = now();
      marker.ns = ns;
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.text = ns;
      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.05;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 1e-3 * CarParameters::displayScale;
      marker.scale.y = 25e-3 * CarParameters::displayScale;
      marker.scale.z = 25e-3 * CarParameters::displayScale;
      marker.color.r = static_cast<float>(colorRGB[0]);
      marker.color.g = static_cast<float>(colorRGB[1]);
      marker.color.b = static_cast<float>(colorRGB[2]);
      marker.color.a = 1.0F;
      //marker.lifetime = ros::Duration();
  }


  void createManeuverMarker(visualization_msgs::msg::Marker& marker,
                            visualization_msgs::msg::Marker::_ns_type ns,
                            visualization_msgs::msg::Marker::_id_type id,
                            const ColorType& colorRGB)
  {
      marker.header.frame_id = "map";
      marker.header.stamp = now();
      marker.ns = ns;
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.01 * CarParameters::displayScale;
      marker.scale.y = 0.01 * CarParameters::displayScale;
      marker.scale.z = 0.01 * CarParameters::displayScale;
      marker.color.r = static_cast<float>(colorRGB[0]);
      marker.color.g = static_cast<float>(colorRGB[1]);
      marker.color.b = static_cast<float>(colorRGB[2]);
      marker.color.a = 1.0F;
  }

//  void createTargetSegmentMarker(visualization_msgs::msg::Marker& marker,
//                            visualization_msgs::msg::Marker::_ns_type ns,
//                            visualization_msgs::msg::Marker::_id_type id,
//                            const ColorType& colorRGB)
//  {
//      missionStateMsg.targetSegmentS.at(0) = -1.0F;
//      missionStateMsg.targetSegmentS.at(1) = -1.0F;
//      marker.header.frame_id = "map";
//      marker.header.stamp = now();
//      marker.ns = ns;
//      marker.id = id;
//      marker.type = visualization_msgs::msg::Marker::CYLINDER;
//      marker.action = visualization_msgs::msg::Marker::ADD;
//      marker.pose.position.x = missionStateMsg.targetSegmentS.at(0);
//      marker.pose.position.y = missionStateMsg.targetSegmentS.at(1);
//      marker.pose.position.z = 0;
//      marker.pose.orientation.x = 0.0;
//      marker.pose.orientation.y = 0.0;
//      marker.pose.orientation.z = 0.0;
//      marker.pose.orientation.w = 1.0;
//      marker.scale.x = 0.1;
//      marker.scale.y = 0.1;
//      marker.scale.z = 0.02;
//      marker.color.r = colorRGB[0];
//      marker.color.g = colorRGB[1];
//      marker.color.b = colorRGB[2];
//      marker.color.a = 1.0F;
// marker.lifetime = ros::Duration();
//  }

  void updateManeuverMarker(visualization_msgs::msg::Marker& marker)
  {
    marker.points.clear();
    for (std::size_t i = 0; i < maneuverMsg.breaks.size(); ++i) {
      geometry_msgs::msg::Point p;
      p.x = static_cast<double>(maneuverMsg.s1.at(i));
      p.y = static_cast<double>(maneuverMsg.s2.at(i));
      p.z = 0.01;
      marker.points.push_back(p);
    }
  }

  /**
   * @brief createMarkers creates marker array
   */
  void createMarkers()
  {
    visualization_msgs::msg::Marker markerArrow;
    // TODO
    createCarArrowMarker(markerArrow, get_namespace(), 0, colorRGB);
    markerArrow.color.r = 1.0;
    markerArrow.color.g = 1.0;
    markerArrow.color.b = 1.0;
    markerArray.markers.push_back(markerArrow);

    visualization_msgs::msg::Marker markerRefArrow;
    createCarArrowMarker(markerRefArrow, "map", 1, colorRGB);
    markerArray.markers.push_back(markerRefArrow);

    visualization_msgs::msg::Marker markerName;
    createCarNameMarker(markerName, get_namespace(), 2, colorRGB);
    markerArray.markers.push_back(markerName);

    visualization_msgs::msg::Marker markerBox;
    createCarBoxMarker(markerBox, get_namespace(), 3, colorRGB);
    markerArray.markers.push_back(markerBox);

    visualization_msgs::msg::Marker markerManeuver;
    createManeuverMarker(markerManeuver, get_namespace(), 4, colorRGB);
    markerArray.markers.push_back(markerManeuver);

//    visualization_msgs::msg::Marker markerTargetSegment;
//    createTargetSegmentMarker(markerTargetSegment, node.getNamespace(), 5, colorRGB);
//    markerArray.markers.push_back(markerTargetSegment);

    pubMarker->publish(markerArray);
    for (auto& marker : markerArray.markers) {
        marker.action = visualization_msgs::msg::Marker::MODIFY;
    }
  }

  /**
   * @brief outputsCallback triggers display
   * @param msg CarOutputs message
   */
  void carOutputsCallback(const mbmadmsgs::msg::CarOutputsList::SharedPtr msgList)
  {
    for (auto& msg : msgList->list) {
      if (static_cast<int>(msg.carid) == CarParameters::p()->carid) {
        outputsMsg = msg;
      }
    }
  }

  void carOutputsExtCallback(const mbmadmsgs::msg::CarOutputsExtList::SharedPtr msgList)
  {
    for (auto& msg : msgList->list) {
      if (static_cast<int>(msg.carid) == CarParameters::p()->carid) {
        outputsExtMsg = msg;
      }
    }
  }

  void ctrlReferenceCallback(const mbmadmsgs::msg::CtrlReference::SharedPtr msg)
  {
    if (static_cast<int>(msg->carid) == CarParameters::p()->carid) {
      ctrlreferenceMsg = *msg;
    }
  }

  void maneuverCallback(const mbmadmsgs::msg::DriveManeuver::SharedPtr msg)
  {
    if (static_cast<int>(msg->carid) == CarParameters::p()->carid) {
      maneuverMsg = *msg;
      updateManeuverMarker(markerArray.markers.at(4));
    }
  }


//  void maneuverStateCallback(const madmsgs::DriveManeuverStateConstPtr& msg)
//  {
//    if (static_cast<int>(msg->carid) == CarParameters::p()->carid) {
//      maneuverStateMsg = *msg;
//    }
//  }

//  void missionStateCallback(const madmsgs::MissionStateConstPtr& msg)
//  {
//    if (static_cast<int>(msg->carid) == CarParameters::p()->carid) {
//      missionStateMsg = *msg;
//    }
//  }


  std::string floatToString(const float x)
  {
    float y = 1000.0F * x;
    if (std::fabs(y) < 10.0F) {
      char buf[4];
      std::snprintf(buf, sizeof(buf), "%+2.0f", static_cast<double>(y));
      return std::string(buf);
    } else {
      return "";
    }
  }

  std::string carName()
  {
    std::string label { "" };
    // TODO
//      std::string state { CarParameters::p()->maneuverState.at(maneuverStateMsg.state) };
//      if (maneuverStateMsg.batteryLow) {
//        if (maneuverStateMsg.state == maneuverStateMsg.STATE_CHARGING) {
//          state = "batterylow";
//        } else {
//          state += "low";
//        }
//      }
//      std::string label { std::to_string(CarParameters::p()->carid) + ":"
//              + state
//              + floatToString(maneuverStateMsg.ex)
//              + "," + floatToString(maneuverStateMsg.ey) };
      return label;
  }

};

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mbmad::CarDisplayNode>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
