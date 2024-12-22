/**
  * @brief ROS2 node VisionNode for MAD76 ArUco marker tracking
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
#include <opencv2/aruco.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <vector>
#include "mbmadmsgs/msg/car_outputs_list.hpp"
#include "mbmadmsgs/srv/vision_transform_points.hpp"
#include "mbmad/CheckpointGraph.hpp"
#include "mbmad/CarParameters.hpp"
#include "mbsafe/Platform.hpp"
#include "mbsafe/CheckpointSequence.hpp"
#include "mbsafe/CheckpointDeadlineMonitor.hpp"
#include "mbsafe/CheckpointCounterMonitor.hpp"
#include "mbsafe/CheckpointJitterMonitor.hpp"
#include "mbsafe/Task.hpp"
#include "mbsafe/SingleHealth.hpp"
#include "Frame.hpp"

#undef _MB_DEBUG
//#define _MB_DEBUG

namespace mbmad {

const std::string namespaceDefault { "/mad/vision" };

class VisionNode : public rclcpp::Node
{
public:
  VisionNode() : Node("visionnode", namespaceDefault)
  {
    // create aruco dictionary and detector parameters
    dictionary = cv::aruco::generateCustomDictionary(8, 3);
    detectorParams = cv::aruco::DetectorParameters::create();
    detectorParams->markerBorderBits = 1;

    cpSeq.registerMonitor(cpJitterMonCameraAcquisition);
    cpSeq.registerMonitor(cpJitterMonReceive);
    cpSeq.registerMonitor(cpDeadlineMonReceive);
    cpSeq.registerMonitor(cpDeadlineMonPublish);

    diagUpdater.setHardwareID(get_name());
    diagUpdater.add("CheckpointSequence", this, &VisionNode::diagCheckpointSequence);
  }

  bool init()
  {
    boost::log::core::get()->set_filter(
        boost::log::trivial::severity >= boost::log::trivial::error
        );
    RCLCPP_INFO(get_logger(), "Initializing VisionNode");

    // ROS2 param HeightCompensation
    heightCompensation = declare_parameter<std::vector<double>>("HeightCompensation", heightCompensation);
    if (heightCompensation.size() != 4) {
      RCLCPP_ERROR(get_logger(), "Invalid height compensation configuration: number of height compensations must be 4: left, right, bottom, top.");
      return false;
    }

    // ROS2 param UndistoredImageSize
    imageSize = cv::Size(declare_parameter<int>("UndistoredImageSizeWidth", imageSize.width), declare_parameter<int>("UndistoredImageSizeHeight", imageSize.height));

    // ROS2 param DisplayImage
    displayImage = declare_parameter<bool>("DisplayImage", displayImage);
    
    // frame markers parameters
    std::vector<int64> frameMarkersId = {4, 7, 6, 5};
    frameMarkersId = declare_parameter<std::vector<int64>>("FrameMarkersId", frameMarkersId);
    carMaxId = (*std::min_element(frameMarkersId.begin(), frameMarkersId.end())) - 1;

    std::vector<double> frameMarkersS1 = {0.0, 0.80, 0.0, 0.80};
    
    frameMarkersS1 = declare_parameter<std::vector<double>>("FrameMarkersS1", frameMarkersS1);
    std::vector<double> frameMarkersS2 = {0.0, 0.0, 0.45, 0.45};
    frameMarkersS2 = declare_parameter<std::vector<double>>("FrameMarkersS2", frameMarkersS2);

    if (frameMarkersId.size() != 3 && frameMarkersId.size() != 4) {
      RCLCPP_ERROR(get_logger(), "Invalid frame markers configuration: number of frame markers must be 3 or 4.");
      return false;
    }
    if (frameMarkersS1.size() != frameMarkersId.size() || frameMarkersS2.size() != frameMarkersId.size()) {
      RCLCPP_ERROR(get_logger(), "Invalid frame markers configuration: number of s1 and s2 coordinates must be equal to number of ids.");
      return false;
    }

    cv::Vec2f heightCompensation1(heightCompensation[0], heightCompensation[1]);
    cv::Vec2f heightCompensation2(heightCompensation[2], heightCompensation[3]);

    frames.push_back(std::make_shared<mbmad::Frame>(
      frameMarkersId[0], cv::Point2f(frameMarkersS1[0], frameMarkersS2[0]), 
      frameMarkersId[1], cv::Point2f(frameMarkersS1[1], frameMarkersS2[1]), 
      frameMarkersId[2], cv::Point2f(frameMarkersS1[2], frameMarkersS2[2]),
      heightCompensation1, heightCompensation2));
    if (frameMarkersId.size() == 4) {
      frames.push_back(std::make_shared<mbmad::Frame>(
        frameMarkersId[3], cv::Point2f(frameMarkersS1[3], frameMarkersS2[3]), 
        frameMarkersId[2], cv::Point2f(frameMarkersS1[2], frameMarkersS2[2]), 
        frameMarkersId[1], cv::Point2f(frameMarkersS1[1], frameMarkersS2[1]),
        heightCompensation1, heightCompensation2));
    }
    
    rclcpp::QoS qos { rclcpp::KeepLast(1) };
    //qos.reliable();
    qos.best_effort().durability_volatile();
    
    subImg = this->create_subscription<sensor_msgs::msg::Image>("/mad/camera/image_raw", qos, std::bind(&VisionNode::imageCallback, this, std::placeholders::_1));
    subCaminfo = this->create_subscription<sensor_msgs::msg::CameraInfo>("/mad/camera/camera_info", qos, std::bind(&VisionNode::camerainfoCallback, this, std::placeholders::_1));
    pubImg = this->create_publisher<sensor_msgs::msg::Image>("image", qos);
    pubOutputs = this->create_publisher<mbmadmsgs::msg::CarOutputsList>("caroutputs", qos);
    timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&VisionNode::timerCallback, this));

    return true;
  }

  
  bool exit()
  {
    RCLCPP_INFO(get_logger(), "Exiting VisionNode");

    return true;
  }

private:
  const float markerLength = 0.021f;
  const int detectBoardCtrMax = 100;
  const uint32_t displayImageDownsample = 4;
  int carMaxId = 3;
  //std::vector<double> heightCompensation = {-0.015, 0.008, -0.014, 0.022};
  
  // ROS2 params
  std::vector<double> heightCompensation = {-0.004, 0.0, 0.0, 0.0};
  cv::Size imageSize = cv::Size(860, 645);
  bool displayImage = false;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImg;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subCaminfo;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubImg;
  rclcpp::Publisher<mbmadmsgs::msg::CarOutputsList>::SharedPtr pubOutputs;
  rclcpp::Service<mbmadmsgs::srv::VisionTransformPoints>::SharedPtr srvTransform;
  rclcpp::TimerBase::SharedPtr timer;
  
  cv::Mat undistortMap1;
  cv::Mat undistortMap2;
  
  bool camCalibDone = false;
  bool boardCalibDone = false;
  bool imageProcDone = false;
  
  cv_bridge::CvImage cvImg;
  
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  cv::Ptr<cv::aruco::DetectorParameters> detectorParams;    
  std::vector<std::shared_ptr<mbmad::Frame>> frames;

  mbsafe::CheckpointSequence cpSeq { *this, CheckpointGraph::lastCheckpoint };
  mbsafe::CheckpointJitterMonitor cpJitterMonCameraAcquisition { *this, cpSeq,
                                    CheckpointGraph::Checkpoint::CameraAcquisition,
                                    0.5 * static_cast<double>(mbmad::CarParameters::p()->Tva),
                                    1.5 * static_cast<double>(mbmad::CarParameters::p()->Tva), 0, 1 };
  mbsafe::CheckpointJitterMonitor cpJitterMonReceive { *this, cpSeq,
                                    CheckpointGraph::Checkpoint::VisionStart,
                                    0.5 * static_cast<double>(mbmad::CarParameters::p()->Tva),
                                    1.5 * static_cast<double>(mbmad::CarParameters::p()->Tva), 0, 1 };
  // mbsafe::CheckpointCounterMonitor cpCounterMonReceive { *this, cpSeq,
  //                                   CheckpointGraph::Checkpoint::VisionStart };
  mbsafe::CheckpointDeadlineMonitor cpDeadlineMonReceive { *this, cpSeq,
                                    CheckpointGraph::Checkpoint::CameraPublish,
                                    CheckpointGraph::Checkpoint::VisionStart,
                                    0.0, 2.0 * static_cast<double>(mbmad::CarParameters::p()->Tva), 0, 1 };
  mbsafe::CheckpointDeadlineMonitor cpDeadlineMonPublish { *this, cpSeq,
                                    CheckpointGraph::Checkpoint::VisionStart,
                                    CheckpointGraph::Checkpoint::VisionPublish,
                                    0.0, static_cast<double>(mbmad::CarParameters::p()->Tva), 0, 1 };
  //mbsafe::CheckpointGraphMonitor cpGraphMon { *this, CheckpointGraph::graph };
  diagnostic_updater::Updater diagUpdater { this, 1.0 };
  uint64_t seqctr = 0ULL;

  void camerainfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (!camCalibDone) {
      cv::Mat camMatrix = cv::Mat(3, 3, CV_64F, (void *)msg->k.data()).clone();
      cv::Mat distCoeffs = cv::Mat(1, 5, CV_64F, (void *)msg->d.data()).clone();
      cv::initUndistortRectifyMap(camMatrix, distCoeffs, cv::Mat(),
                                  camMatrix, imageSize,
                                  CV_32FC1, undistortMap1, undistortMap2);
  #ifdef _MB_DEBUG
      std::cout << "camMatrix=" << camMatrix << std::endl;
      std::cout << "distCoeffs=" << distCoeffs << std::endl;
  #endif
      camCalibDone = true;
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (camCalibDone) {      
      cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

      if (boardCalibDone) {        
        ++seqctr;
        rclcpp::Time camTime(msg->header.stamp);
        cpSeq.update(CheckpointGraph::Checkpoint::CameraAcquisition, seqctr, 
                     camTime);
        cpSeq.update(CheckpointGraph::Checkpoint::CameraStart, seqctr,
                     camTime);
        cpSeq.update(CheckpointGraph::Checkpoint::CameraPublish, seqctr,
                     camTime);
        cpSeq.update(CheckpointGraph::Checkpoint::VisionStart);
      }

      // rectify image
      cv::remap(img, cvImg.image, undistortMap1, undistortMap2, cv::INTER_LINEAR);
      
      if (boardCalibDone) {        
        detectMarkers(cvImg.image);
      } else {
        detectBoard(cvImg.image);    
      }

      if (displayImage && seqctr % displayImageDownsample == 0U) {
        cv::imshow("VisionNode", cvImg.image);
        cv::waitKey(1);
      }
    }
  }

  /**
   * @brief ROS2 service for coordinate transformation
   * @param request_header
   * @param request 
   * @param response
   */
  void transformCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const mbmadmsgs::srv::VisionTransformPoints::Request::SharedPtr request,
    mbmadmsgs::srv::VisionTransformPoints::Response::SharedPtr response)
  {
    response->s1.clear();
    response->s2.clear();
    if (request->s1.size() != request->s2.size()) {
      RCLCPP_ERROR(get_logger(), "Invalid request: s1 and s2 vectors must have the same size.");
      return;
    }
    if (camCalibDone && boardCalibDone) {      
      for (std::size_t i=0; i<request->s1.size(); ++i) {
        const cv::Point2f point(request->s1[i], request->s2[i]);
        cv::Point2f pointW;
        Frame::avgPoint2Point(frames, point, pointW);
        response->s1.push_back(pointW.x);
        response->s2.push_back(pointW.y);
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Calibration not ready.");
    }      
  }

  void timerCallback()
  {
    if (camCalibDone) {
      cvImg.header.stamp = rclcpp::Clock().now();
      cvImg.header.frame_id = "camera";
      cvImg.encoding = "bgr8";
      pubImg->publish(*cvImg.toImageMsg());
    }
  }

  
  void detectBoard(const cv::Mat& img)
  {
    
    std::vector< int > ids;
    std::vector< std::vector< cv::Point2f > > corners, rejected;
         
    cv::aruco::detectMarkers(img, dictionary, corners, ids, detectorParams, rejected);
    boardCalibDone = true;
    for (auto frame: frames) {
      boardCalibDone &= frame->computeFrame(ids, corners);
    }
    if (boardCalibDone) {
      if (srvTransform == nullptr) {
          srvTransform = this->create_service<mbmadmsgs::srv::VisionTransformPoints>("transform_points",
            std::bind(&VisionNode::transformCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
      }
    }
  }

  void detectMarkers(const cv::Mat& img)
  {
    std::vector< int > ids;
    std::vector< std::vector< cv::Point2f > > corners, rejected;
    std::vector<cv::Point2f> points;
    std::vector<Frame::dtype> psis;

    
    // detect markers and estimate pose
    cv::aruco::detectMarkers(img, dictionary, corners, ids, detectorParams, rejected);
        
    for (std::size_t i = 0; i < ids.size(); ++i) {
      cv::Point2f point;
      Frame::avgCorners2Point(frames, corners[i], point);
      points.push_back(point);
      psis.push_back(Frame::avgCorners2Yawangle(frames, corners[i], point));
    }

    mbmadmsgs::msg::CarOutputsList msg;      
    if (ids.size() > 0) {
      cv::aruco::drawDetectedMarkers(cvImg.image, corners, ids);
      for (std::size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] <= carMaxId) {
          mbmadmsgs::msg::CarOutputs carOutputs;
          cv::Point2f pointCompensated;
          frames[0]->compensateHeight(points[i], pointCompensated);
          carOutputs.carid = ids[i];
          carOutputs.s.at(0) = pointCompensated.x;
          carOutputs.s.at(1) = pointCompensated.y;
          carOutputs.psi = psis[i];
          carOutputs.prob = 1.0f;
          msg.list.push_back(carOutputs);
        }
#ifdef _MB_DEBUG
        std::cout << "i=" << i << " id=" << ids[i] << " s=" << points[i] << " psi=" << psis[i]*180.0f/M_PIf << " corners=" << corners[i] << std::endl;
#endif
      }      
    }
    cpSeq.update(CheckpointGraph::Checkpoint::VisionPublish);
    msg.cpseq = cpSeq.message();
    pubOutputs->publish(msg);
    imageProcDone = true;
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
  if (mbsafe::Platform::init(10000ULL, false, false)
        && mbsafe::Platform::setPrio(SCHED_FIFO, 90)
        && mbsafe::Platform::prefaultStack<10000ULL>()) {
    std::shared_ptr<mbmad::VisionNode> node = std::make_shared<mbmad::VisionNode>();
    if (node->init()) {
      rclcpp::spin(node);
    }
    mbsafe::Platform::exit();
  }
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
