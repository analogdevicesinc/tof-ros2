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
#include "cameraInfo_message.h"

using namespace aditof;

CameraInfoMsg::CameraInfoMsg() {}

CameraInfoMsg::CameraInfoMsg(const std::shared_ptr<aditof::Camera> &camera,
                             aditof::Frame **frame, rclcpp::Time tStamp) {
    FrameDataToMsg(camera, frame, tStamp);
}

void CameraInfoMsg::FrameDataToMsg(const std::shared_ptr<Camera> &camera,
                                   aditof::Frame **frame, rclcpp::Time tStamp) {
    FrameDetails fDetails;
    (*frame)->getDetails(fDetails);

    setMembers(camera, fDetails.width, fDetails.height, tStamp);
}

void CameraInfoMsg::setMembers(const std::shared_ptr<Camera> &camera, int width,
                               int height, rclcpp::Time tStamp) {
    message.header.stamp = tStamp;
    message.header.frame_id = "aditof_camera_info";

    message.width = width;
    message.height = height;
    message.distortion_model = "plumb_bob";

    IntrinsicParameters intr = getIntrinsics(camera);

    message.d = {intr.k1, intr.k2, intr.p1, intr.p2, intr.k3};
    message.k = {intr.fx, 0.0, intr.cx, 0.0, intr.fy, intr.cy, 0.0, 0.0, 1.0};
    message.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    message.p = {message.k[0], message.k[1], message.k[2], 0.0f,     message.k[3], message.k[4],
             message.k[5], 0.0f,     message.k[6], message.k[7], message.k[8], 0.0f};
}

void CameraInfoMsg::publishMsg(const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) { pub->publish(message); }
