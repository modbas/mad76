/**
  * @brief main function of ROS2 node RcNode for GPIO / SPI MAD76 IO control
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
#include "mbsafe/Platform.hpp"
#include "RcNode.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  if (mbsafe::Platform::init(1000ULL, false, false)
        && mbsafe::Platform::setPrio(SCHED_FIFO, 90)
        && mbsafe::Platform::prefaultStack<1000ULL>()) {
    std::shared_ptr<mbmad::RcNode> node = std::make_shared<mbmad::RcNode>();
    if (node->init()) {
      rclcpp::spin(node);
    }
    node->exit();
    mbsafe::Platform::exit();
  }
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
