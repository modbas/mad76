/**
  * @brief ROS2 node DelayTestNode for measuring camera delay
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

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <wiringPi.h>
#include "mbmad/CarParameters76.hpp"


#undef _MB_DEBUG
//#define _MB_DEBUG

namespace mbmad {

const std::string namespaceDefault { "/mad/vision" };

class DelayTestNode : public rclcpp::Node
{
public:
  DelayTestNode() : Node("delaytestnode", namespaceDefault)
  {
  }

  bool init()
  {
    RCLCPP_INFO(get_logger(), "Initializing DelayTestNode");

    if (wiringPiSetup() == -1)
    {
      RCLCPP_INFO(get_logger(), "wiringPiSetup failed");
      return false;
    }

    if (wiringPiSetupGpio() == -1)
    {
      RCLCPP_INFO(get_logger(), "wiringPiSetupGpio failed");
      return false;
    }

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    
    rclcpp::QoS qos { rclcpp::KeepLast(1) };
    //qos.reliable();
    qos.best_effort().durability_volatile();
    
    subImg = this->create_subscription<sensor_msgs::msg::Image>("/mad/camera/image_raw", qos, std::bind(&DelayTestNode::imageCallback, this, std::placeholders::_1));
    subCaminfo = this->create_subscription<sensor_msgs::msg::CameraInfo>("/mad/camera/camera_info", qos, std::bind(&DelayTestNode::camerainfoCallback, this, std::placeholders::_1));
    
    return true;
  }

  
  bool exit()
  {
    RCLCPP_INFO(get_logger(), "Exiting DelayTestNode");

    return true;
  }

private:
  const int LED_PIN = 17; // GPIO17
  const uint32_t LED_PERIOD = 8; // 8 * 25ms = 200ms
  const uint32_t LED_DUTY = 2; // 4 * 25ms = 50ms
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImg;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subCaminfo;
  bool camCalibDone = false;
  cv_bridge::CvImage cvImg;  
  uint64_t seqctr = 0ULL;
  cv::VideoWriter video { "/tmp/mad76.avi", cv::VideoWriter::fourcc('M','J','P','G'), 40, cv::Size(800, 600) };      
  double t0 = 0.0;
      

  
  void camerainfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (!camCalibDone) {
      camCalibDone = true;
      t0 = rclcpp::Time(msg->header.stamp).seconds();
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (camCalibDone) {      
      ++seqctr;
      double camTime = rclcpp::Time(msg->header.stamp).seconds() - t0;

      // blink LED
      uint32_t onctr = seqctr % LED_PERIOD;
      if (seqctr % LED_PERIOD == 0U) {
        t0 = rclcpp::Time(msg->header.stamp).seconds();
      }
      if (seqctr % LED_PERIOD < LED_DUTY) {
        digitalWrite(LED_PIN, LOW);
      } else {
        digitalWrite(LED_PIN, HIGH);
      }
      
      
      cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
      
      // display timestamp
      if (img.empty()) {
        RCLCPP_ERROR(get_logger(), "Image is empty");
      } else {
        try {
          std::string str("seqctr=" + std::to_string(seqctr) + "onctr=" + std::to_string(onctr) + " ontime=" + std::to_string(camTime) + "s");      
          cv::putText(img, str, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        } catch (cv::Exception& e) {
          RCLCPP_ERROR(get_logger(), "Exception: %s", e.what());
        }
      }

      // display CV image
      // cv::imshow("Image", img);
      // cv::waitKey(1);

      // write video stream
      video.write(img);
    }
  }

};

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<mbmad::DelayTestNode> node = std::make_shared<mbmad::DelayTestNode>();
  if (node->init()) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
