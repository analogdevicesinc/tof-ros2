/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "xyzImage_msg.h"

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <string>
using namespace std::chrono_literals;
using namespace aditof;

XYZImageMsg::XYZImageMsg() {}

XYZImageMsg::XYZImageMsg(
  const std::shared_ptr<aditof::Camera> & camera, aditof::Frame ** frame, std::string encoding)
{
  imgEncoding = encoding;
}

void XYZImageMsg::FrameDataToMsg(const std::shared_ptr<Camera> & camera, aditof::Frame * frame)
{
  FrameDetails fDetails;
  frame->getDetails(fDetails);

  setMetadataMembers(fDetails.width, fDetails.height);

  uint16_t * frameData = getFrameData(frame, "xyz");

  if (!frameData) {
    LOG(ERROR) << "getFrameData call failed";
    return;
  }

  setDataMembers(frameData);
}

void XYZImageMsg::setMetadataMembers(int width, int height)
{
  message.header.frame_id = "aditof_xyz_img";
  message.width = width;
  message.height = height;
  message.is_bigendian = false;
}

void XYZImageMsg::setDataMembers(uint16_t * frameData)
{
  int16_t * msgDataPtr = (int16_t *)frameData;

  //Modifier to describe what the fields are.
  sensor_msgs::PointCloud2Modifier modifier(message);

  modifier.setPointCloud2Fields(
    4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);

  //Msg header
  //  message.header = std_msgs::msg::Header();
  //  message.header.stamp =rclcpp::Clock{RCL_ROS_TIME}.now();
  //  message.header.frame_id = "aditof_xyz_img";

  //message.height = message.height;
  //message.width = message.width;
  //message.is_dense = true;

  //Total number of bytes per point
  message.point_step = 16;
  message.row_step = message.point_step * message.width * message.height;
  message.data.resize(message.row_step);

  //Iterators for PointCloud msg
  sensor_msgs::PointCloud2Iterator<float> iterX(message, "x");
  sensor_msgs::PointCloud2Iterator<float> iterY(message, "y");
  sensor_msgs::PointCloud2Iterator<float> iterZ(message, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(message, "r");

  int i = 0;

  for (; iterX != iterX.end(); ++iterX, ++iterY, ++iterZ, ++iter_r) {
    *iterX = static_cast<float>(msgDataPtr[i]);
    *iterY = static_cast<float>(msgDataPtr[i + 1]);
    *iterZ = static_cast<float>(msgDataPtr[i + 2]);

    *iter_r = 255;
    i += 3;
  }
}

sensor_msgs::msg::PointCloud2 XYZImageMsg::getMessagePointCloud() { return message; }

void XYZImageMsg::publishMsg(rclcpp::Publisher<sensor_msgs::msg::PointCloud2> & pub)
{
  pub.publish(message);
}
