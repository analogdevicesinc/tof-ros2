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
#ifndef XYZIMAGE_MSG_H
#define XYZIMAGE_MSG_H

#include <aditof/frame.h>
#include <aditof_sensor_msg.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "aditof_utils.h"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"

class XYZImageMsg : public AditofSensorPointCloudMsg
{
public:
  XYZImageMsg(std::string encoding);
  /**
   * @brief Each message corresponds to one frame
   */
  sensor_msgs::msg::PointCloud2 message;

  /**
   * @brief Variale to color the points in rviz2
   */
  sensor_msgs::msg::ChannelFloat32 m_intensity, m_range;

  /**
   * @brief Points that will be publish on message
   */
  std::vector<geometry_msgs::msg::Point32> m_points;

  /**
   * @brief Will be assigned a value from the list of strings in include/sensor_msgs/image_encodings.h
   */
  std::string imgEncoding;

  /**
   * @brief Converts the frame data to a message
   */
  void FrameDataToMsg(
    const std::shared_ptr<aditof::Camera> & camera, aditof::Frame ** frame) override;
  /**
   * @brief Assigns values to the message fields concerning metadata
   */
  void setMetadataMembers(int width, int height);

  /**
   * @brief Assigns values to the message fields concerning the point data
   */
  void setDataMembers(uint16_t * frameData);

  /**
   * @brief Publishes a message
   */
  void publishMsg(rclcpp::Publisher<sensor_msgs::msg::PointCloud2> & pub) override;

  /**
   * @brief Get the point cloud message
   */
  sensor_msgs::msg::PointCloud2 getMessagePointCloud() override;

private:
  XYZImageMsg();
};

#endif  // XYZIMAGE_MSG_H
