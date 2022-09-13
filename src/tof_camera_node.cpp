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
#include <aditof_utils.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "aditof/camera.h"
#include <aditof_sensor_msg.h>
#include <irImage_msg.h>
#include <depthImage_msg.h>

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"


using namespace aditof;

std::mutex m_mtxDynamicRec;
std::mutex m_mtxDynamicRec2;

using namespace std::chrono_literals;

int main(int argc, char **argv)
{

    std::string *arguments = parseArgs(argc, argv);
    bool m_enableDepthCompute = true;
    /*
    pos 0 - ip
    pos 1 - config_path
    pos 2 - use_depthCompute
    pos 3 - mode
    */
    arguments[2] = "1"; // always use depthCompute

    // Initializing camera and establishing connection
    std::shared_ptr<Camera> camera = initCamera(arguments);

    // Creating camera_node
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("tof_camera_publisher", options);
    image_transport::ImageTransport it(node);
    
    image_transport::Publisher pubIR = it.advertise("tof_camera/ir", 1);
    image_transport::Publisher pubDEPTH = it.advertise("tof_camera/depth", 2);

    if (arguments[2] == "true")
        m_enableDepthCompute = true;
    else
        m_enableDepthCompute = false;
    enableCameraDepthCompute(camera, m_enableDepthCompute);

    if (arguments[3] == "1")
        setFrameType(camera, "qmp");
    else if (arguments[3] == "2")
        setFrameType(camera, "mp");


    auto tmp = new Frame;
    aditof::Frame **frame = &tmp;
    IRImageMsg *messageIr = new IRImageMsg(camera, frame, sensor_msgs::image_encodings::MONO16);
    DepthImageMsg *messageDepth = new DepthImageMsg(camera, frame, sensor_msgs::image_encodings::RGBA8);
    startCamera(camera);


    rclcpp::WallRate loop_rate(5);
    while (rclcpp::ok())
    {

        getNewFrame(camera, frame);
        messageIr->FrameDataToMsg(camera, frame);
        messageDepth->FrameDataToMsg(camera, frame);

        pubIR.publish(messageIr->getMessage());
        pubDEPTH.publish(messageDepth->getMessage());
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    return 0;
}
