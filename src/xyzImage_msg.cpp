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
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
using namespace std::chrono_literals;
using namespace aditof;

XYZImageMsg::XYZImageMsg() {}

XYZImageMsg::XYZImageMsg(const std::shared_ptr<aditof::Camera> &camera,
                       aditof::Frame **frame, std::string encoding)
{
    imgEncoding = encoding;
    FrameDataToMsg(camera, frame);
}

void XYZImageMsg::FrameDataToMsg(const std::shared_ptr<Camera> &camera,
                                aditof::Frame **frame)
{
    FrameDetails fDetails;
    (*frame)->getDetails(fDetails);

    setMetadataMembers(fDetails.width, fDetails.height);

    uint16_t *frameData = getFrameData(frame, "xyz");

    if (!frameData)
    {
        LOG(ERROR) << "getFrameData call failed";
        return;
    }

    setDataMembers(camera, frameData);
}

void XYZImageMsg::setMetadataMembers(int width, int height)
{
    message.header.frame_id = "aditof_xyz_img";
    message.width = width;
    message.height = height;
    message.is_bigendian = false;
}

void XYZImageMsg::setDataMembers(const std::shared_ptr<Camera> &camera,
                                uint16_t *frameData)
{
    m_points.clear();
    m_intensity.values.clear();
    m_range.values.clear();

    int16_t *msgDataPtr = (int16_t*)frameData;

    for(int i=0 ; i < message.width * message.height * 3 ; i+=3)
    {
        auto pt = geometry_msgs::msg::Point32();
        pt.x = static_cast<float>(msgDataPtr[i]);
        pt.y = static_cast<float>(msgDataPtr[i+1]);
        pt.z = static_cast<float>(msgDataPtr[i+2]);
        m_points.push_back(pt);

        m_intensity.values.push_back(static_cast<float>(pt.z));
        m_range.values.push_back(static_cast<float>(pt.z));
    }

    sensor_msgs::msg::PointCloud cloud;

    cloud.header.stamp.nanosec = rclcpp::Clock{RCL_ROS_TIME}.now().nanoseconds();
    cloud.header.stamp.sec = rclcpp::Clock{RCL_ROS_TIME}.now().seconds();
    cloud.header.frame_id = "map";

    cloud.points = m_points;
    cloud.channels.push_back(m_intensity);
    cloud.channels.push_back(m_range);

    sensor_msgs::convertPointCloudToPointCloud2(cloud, message);
}

sensor_msgs::msg::PointCloud2 XYZImageMsg::getMessagePointCloud()
{
    return message;
}

void XYZImageMsg::publishMsg(rclcpp::Publisher<sensor_msgs::msg::PointCloud2> &pub)
{
     pub.publish(message);
}


