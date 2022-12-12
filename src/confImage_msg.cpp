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
#include "confImage_msg.h"
using namespace aditof;

ConfImageMsg::ConfImageMsg() {}

ConfImageMsg::ConfImageMsg(const std::shared_ptr<aditof::Camera> &camera,
                       aditof::Frame **frame, std::string encoding)
{
    imgEncoding = encoding;
    FrameDataToMsg(camera, frame);
}

void ConfImageMsg::FrameDataToMsg(const std::shared_ptr<Camera> &camera,
                                aditof::Frame **frame)
{
    FrameDetails fDetails;
    (*frame)->getDetails(fDetails);

    setMetadataMembers(fDetails.width, fDetails.height);

    uint16_t *frameData = getFrameData(frame, "raw");

    if (!frameData)
    {
        LOG(ERROR) << "getFrameData call failed";
        return;
    }

    setDataMembers(camera, frameData);
}

void ConfImageMsg::setMetadataMembers(int width, int height)
{
    // message.header.stamp = tStamp;
    message.header.frame_id = "aditof_conf_img";

    message.width = width;
    message.height = height;

    message.encoding = imgEncoding;
    message.is_bigendian = false;

    int pixelByteCnt = sensor_msgs::image_encodings::bitDepth(imgEncoding) / 8 *
                       sensor_msgs::image_encodings::numChannels(imgEncoding);
    message.step = width * pixelByteCnt;

    message.data.resize(message.step * height);
}

void ConfImageMsg::setDataMembers(const std::shared_ptr<Camera> &camera,
                                uint16_t *frameData)
{
    if (message.encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
    {
        confTake5byte(frameData, message.width, message.height);
        irTo16bitGrayscale(frameData, message.width, message.height);
        uint8_t *msgDataPtr = message.data.data();
        memcpy(msgDataPtr, frameData, message.step * message.height);
    }
    else
        LOG(ERROR) << "Image encoding invalid or not available";
}

void ConfImageMsg::confTake5byte(uint16_t *frameData, int width, int height)
{
    uint8_t *data = (uint8_t *)frameData;

    int j=0;
    for (int i = 4; i < width * height *5 ; i+=5)
    {
        frameData[j] = 255 * data[i] / 8;
        j++;
    }
}

sensor_msgs::msg::Image ConfImageMsg::getMessage()
{
    return message;
}

 void ConfImageMsg::publishMsg(rclcpp::Publisher<sensor_msgs::msg::Image> &pub) {
     pub.publish(message);
 }
