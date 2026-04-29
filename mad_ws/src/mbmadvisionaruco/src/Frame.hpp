/**
  * @brief C++ class Frame representing MAD76 coordinate frames
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
#include <array>
#include <map>
#include <cmath>
#include <vector>
#include <memory>


namespace mbmad {

class Frame
{
public:
    using dtype = float;

    Frame(const int id0, const cv::Point2f& point0, const int id1, const cv::Point2f& point1, const int id2, const cv::Point2f& point2, const int id3, const cv::Point2f& point3,
          const cv::Vec2f& heightCompensationBottomTop, const cv::Vec2f& heightCompensationLeftRight, const int samplesCtrMax = 100)
        : idMap({{id0, 0}, {id1, 1}, {id2, 2}, {id3, 3}}),
          points({point0, point1, point2, point3}),
          heightCompensationBottomTop(heightCompensationBottomTop),
          heightCompensationLeftRight(heightCompensationLeftRight),
          samplesCtrMax(samplesCtrMax)
    {
    }
    
    ~Frame() = default;

    void reset()
    {
        pointsC = {cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f)};
        samplesCtr = {0, 0, 0, 0};
    }

    bool computeFrame(const std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& corners)
    {
        for (std::size_t i = 0; i < ids.size(); ++i) {
            if (idMap.find(ids[i]) != idMap.end()) {
                cv::Point2f pointC = (corners[i][0] + corners[i][1] + corners[i][2] + corners[i][3]) * 0.25f;
                int id = idMap[ids[i]];
                ++samplesCtr[id];
                pointsC[id] = (static_cast<dtype>(samplesCtr[id]-1) * pointsC[id] + pointC) / static_cast<dtype>(samplesCtr[id]);
            }           
        }
        bool allMarkersDetected = true;
        for (std::size_t i = 0; i < samplesCtr.size(); ++i) {
            if (samplesCtr[i] < samplesCtrMax) {
                allMarkersDetected = false;
            }
        }
        if (allMarkersDetected) {
            // transformation matrix from camera to board frame
            Rwc = cv::getPerspectiveTransform(pointsC, points);            
        }
        return allMarkersDetected;
    }

    void corners2corners(const std::vector<cv::Point2f>& corners, std::array<cv::Point2f, 4>& cornersW)
    {
        std::array<cv::Point2f, 4> cornersVecW = {corners[0], corners[1], corners[2], corners[3]};
        cv::perspectiveTransform(cornersVecW, cornersW, Rwc);
    }

    void corners2point(const std::array<cv::Point2f, 4>& cornersW, cv::Point2f& pointW)
    {
        pointW = (cornersW[0] + cornersW[1] + cornersW[2] + cornersW[3]) * 0.25f;
    }

    void point2point(const cv::Point2f& point, cv::Point2f& pointW)
    {
        std::array<cv::Point2f, 1> pointVec {point};
        std::array<cv::Point2f, 1> pointWVec;
        cv::perspectiveTransform(pointVec, pointWVec, Rwc);
        pointW = pointWVec[0];
    }

    dtype corners2yawangle(const std::array<cv::Point2f, 4>& cornersW)
    {
        cv::Vec2f yawvecW;
        corners2yawvec(cornersW, yawvecW);
        return std::atan2(yawvecW(1), yawvecW(0));;
    }

    void corners2yawvec(const std::array<cv::Point2f, 4>& cornersW, cv::Vec2f& yawvecW)
    {
        cv::Vec2f diagW0 = cornersW[1] - cornersW[3];
        cv::Vec2f diagW1 = cornersW[2] - cornersW[0];
        yawvecW = (diagW0 + diagW1) * 0.5f;
    }

    void compensateHeight(const cv::Point2f& point, cv::Point2f& pointComp)
    {
        // bottom top compensation
        const dtype delta1 = ((points[1].x - point.x) * heightCompensationLeftRight(0) 
           + (point.x - points[0].x) * heightCompensationLeftRight(1)) / (points[1].x - points[0].x);
        const dtype delta2 = ((points[2].y - point.y) * heightCompensationBottomTop(0) 
           + (point.y - points[0].y) * heightCompensationBottomTop(1)) / (points[2].y - points[0].y);
        pointComp.x = point.x - delta1;
        pointComp.y = point.y - delta2;
    }

private:
    static constexpr dtype distWeightFactor = 100.0f;
    cv::Mat Rwc; // perspective transformation matrix from camera to board frame
    std::map<int, int> idMap;
    std::array<cv::Point2f, 4> pointsC = {cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f)}; // points in camera frame
    const std::array<cv::Point2f, 4> points; // points in board frame
    const cv::Vec2f heightCompensationBottomTop;
    const cv::Vec2f heightCompensationLeftRight;
    const int samplesCtrMax = 100;
    std::array<int, 4> samplesCtr = {0, 0, 0, 0}; // number of samples for each point
};

}
