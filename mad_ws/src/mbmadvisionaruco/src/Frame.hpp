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

    Frame(const int id0, const cv::Point2f& point0, const int idX, const cv::Point2f& pointX, const int idY, const cv::Point2f& pointY,
          const cv::Vec2f& heightCompensation1, const cv::Vec2f& heightCompensation2, const int samplesCtrMax = 100)
        : idMap({{id0, 0}, {idX, 1}, {idY, 2}}),
          points({point0, pointX, pointY}),
          heightCompensation1(heightCompensation1),
          heightCompensation2(heightCompensation2),
          samplesCtrMax(samplesCtrMax)
    {
    }
    
    ~Frame() = default;

    void reset()
    {
        pointsC = {cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f)};
        samplesCtr = {0, 0, 0};
    }

    bool computeFrame(const std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& corners)
    {
        for (std::size_t i = 0; i < ids.size(); ++i) {
            if (idMap.find(ids[i]) != idMap.end()) {
                cv::Point2f pointC {0.0f, 0.0f};
                corners2pointC(corners[i], pointC);
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
            // rotation matrix from camera to board frame
            dtype Rbcm[4] = {pointsC[1].x-pointsC[0].x, pointsC[2].x-pointsC[0].x,
                                      pointsC[1].y-pointsC[0].y, pointsC[2].y-pointsC[0].y};
            cv::Mat Rbc = cv::Mat(2, 2, CV_32F, Rbcm);                        
            // rotation matrix from board to world frame
            dtype Rwbm[4] = {points[1].x - points[0].x, points[2].x - points[0].x,
                                         points[1].y - points[0].y, points[2].y - points[0].y };
            cv::Mat Rwb = cv::Mat(2, 2, CV_32F, Rwbm);
            // rotation matrix from camera to world frame
            Rwc = Rwb * Rbc.inv();
            // std::cout << "Rbc: " << Rbc << std::endl;
            // std::cout << "Rwb: " << Rwb << std::endl;
            // std::cout << "Rwc: " << Rwc << std::endl;
        }
        return allMarkersDetected;
    }

    static void avgCorners2Point(std::vector<std::shared_ptr<Frame>>& frames,
        const std::vector<cv::Point2f>& corners, cv::Point2f& pointW)
    {
        pointW = cv::Point2f(0.0f, 0.0f);
        cv::Vec2f weightTot(0.0f, 0.0f);
        for (const auto& frame : frames) {
            cv::Point2f point;
            frame->corners2point(corners, point);
            cv::Vec2f dist;
            frame->dist2origin(point, dist);
            cv::Vec2f weight(1.0f / (1.0f + distWeightFactor * dist(0)), 1.0f / (1.0f + distWeightFactor * dist(1)));
            pointW.x += point.x * weight(0);
            pointW.y += point.y * weight(1);
            weightTot += weight;
        }
        pointW.x /= weightTot(0);
        pointW.y /= weightTot(1);
    }

    void corners2point(const std::vector<cv::Point2f>& corners, cv::Point2f& pointW)
    {
        cv::Point2f pointC {0.0f, 0.0f};
        corners2pointC(corners, pointC);
        c2w(pointC, pointW);
    }

    static void avgPoint2Point(std::vector<std::shared_ptr<Frame>>& frames,
        const cv::Point2f& point, cv::Point2f& pointW)
    {
        pointW = cv::Point2f(0.0f, 0.0f);
        cv::Vec2f weightTot(0.0f, 0.0f);
        for (const auto& frame : frames) {
            cv::Point2f pointW1;
            frame->point2point(point, pointW1);
            cv::Vec2f dist;
            frame->dist2origin(pointW1, dist);
            cv::Vec2f weight(1.0f / (1.0f + distWeightFactor * dist(0)), 1.0f / (1.0f + distWeightFactor * dist(1)));
            pointW.x += pointW1.x * weight(0);
            pointW.y += pointW1.y * weight(1);
            weightTot += weight;
        }
        pointW.x /= weightTot(0);
        pointW.y /= weightTot(1);
    }

    void point2point(const cv::Point2f& point, cv::Point2f& pointW)
    {
        c2w(point, pointW);
    }

    static dtype avgCorners2Yawangle(std::vector<std::shared_ptr<Frame>>& frames,
        const std::vector<cv::Point2f>& corners, const cv::Point2f& pointW)
    {
        cv::Vec2f yawvec { 0.0f, 0.0f };
        cv::Vec2f distFramesVec(frames[0]->points[0].x - frames[1]->points[0].x,
                                frames[0]->points[0].y - frames[1]->points[0].y);
        dtype weightTot = 0.0f;
        for (const auto& frame : frames) {
            cv::Vec2f yawvecW;
            frame->corners2yawvec(corners, yawvecW);
            cv::Vec2f distVec;
            frame->dist2origin(pointW, distVec);
            dtype weight = 1.0f / (1.0f + distWeightFactor * std::sqrt(distVec(0) * distVec(0) + distVec(1) * distVec(1)));
            yawvec += yawvecW * weight;
            weightTot += weight;
        }
        yawvec /= weightTot;
        return std::atan2(yawvec(1), yawvec(0));;
    }

    dtype corners2yawangle(const std::vector<cv::Point2f>& corners)
    {
        cv::Vec2f yawvecW;
        corners2yawvec(corners, yawvecW);
        return std::atan2(yawvecW(1), yawvecW(0));;
    }

    void corners2yawvec(const std::vector<cv::Point2f>& corners, cv::Vec2f& yawvecW)
    {
        cv::Vec2f diagC0 = corners[1] - corners[3];
        cv::Vec2f diagC1 = corners[2] - corners[0];
        cv::Vec2f yawvecC = (diagC0 + diagC1) * 0.5f;
        cv::Mat yawvecWmat = Rwc * cv::Mat(yawvecC);
        yawvecW(0) = yawvecWmat.at<dtype>(0, 0);
        yawvecW(1) = yawvecWmat.at<dtype>(1, 0);
    }

    void compensateHeight(const cv::Point2f& point, cv::Point2f& pointComp)
    {
        pointComp.x = (point.x - points[0].x) * (points[1].x - heightCompensation1(1) - points[0].x + heightCompensation1(0)) / (points[1].x - points[0].x) + points[0].x - heightCompensation1(0);
        pointComp.y = (point.y - points[0].y) * (points[2].y - heightCompensation2(1) - points[0].y + heightCompensation2(0)) / (points[2].y - points[0].y) + points[0].y - heightCompensation2(0);
    }

private:
    static constexpr dtype distWeightFactor = 10.0f;
    cv::Mat2f Rwc; // rotation matrix from camera to world frame    
    std::map<int, int> idMap;
    std::array<cv::Point2f, 3> pointsC = {cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f)}; // points in camera frame
    const std::array<cv::Point2f, 3> points; // points in board frame
    const cv::Vec2f heightCompensation1;
    const cv::Vec2f heightCompensation2;
    const int samplesCtrMax = 100;
    std::array<int, 3> samplesCtr = {0, 0, 0}; // number of samples for each point
    
    void corners2pointC(const std::vector<cv::Point2f>& corners, cv::Point2f& point)
    {
        point = cv::Point2f(0.0f, 0.0f);
        for (const auto& corner : corners) {
            point += corner;
        }
        point /= static_cast<dtype>(corners.size());
    }

    void c2w(const cv::Point2f& pointC, cv::Point2f& pointW)
    {
        cv::Mat pointWmat = Rwc * cv::Mat(pointC - pointsC[0]) + cv::Mat(points[0]);
        //std::cout << "pointWmat: " << pointWmat << std::endl;
        pointW.x = pointWmat.at<dtype>(0, 0);
        pointW.y = pointWmat.at<dtype>(1, 0);
    }

    void dist2origin(const cv::Point2f& point, cv::Vec2f& dist)
    {
        dist(0) = std::abs(point.x - points[0].x);
        dist(1) = std::abs(point.y - points[0].y);
    }
};

}