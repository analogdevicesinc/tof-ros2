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
#include "message_factory.h"
#include "publisher_factory.h"
#include <aditof_utils.h>

using namespace aditof;

std::mutex m_mtxDynamicRec;
std::mutex m_mtxDynamicRec2;


int main(int argc, char **argv) {


    auto tmp = new Frame;
    std::string *arguments = parseArgs(argc, argv);
    /*
    pos 0 - ip
    pos 1 - config_path
    pos 2 - use_depthCompute
    pos 3 - mode
    pos 4 - rqt 
    */

    std::shared_ptr<Camera> camera = initCamera(arguments);
    // ROS_ASSERT_MSG(camera, "initCamera call failed");
    rclcpp::init(argc, argv);
    //Create camera node
    auto camera_node = rclcpp::Node::make_shared("tof_camera_node");
    // ROS_ASSERT_MSG(camera, "ros init failed");
    
    //Creating generic publisher for the camera 
    // PublisherFactory publishers();
        
    //Creating frame
    // aditof::Frame **frame = &tmp;

    // publishers.m_enableDepthCompute =
    //     (strcmp(arguments[2].c_str(), "true") ? false : true);
    // if (strcmp(arguments[4].c_str(), "true") != 0) {
    //     publishers.createNew(intToMode(std::stoi(arguments[3])), nHandle,
    //                          camera, frame);
    // }
    // dynamic_reconfigure::Server<aditof_roscpp::Aditof_roscppConfig> server;
    
    // if (strcmp(arguments[4].c_str(), "true") == 0) {
    //     dynamic_reconfigure::Server<
    //         aditof_roscpp::Aditof_roscppConfig>::CallbackType f;
    //     f = boost::bind(&callback, _1, &publishers, &nHandle, camera, frame);
    //     server.setCallback(f);
    // }

    // while (rclcpp::ok()) {
    //     while (m_mtxDynamicRec.try_lock())
    //         ;
    //     while (m_mtxDynamicRec2.try_lock())
    //         ;

    //     m_mtxDynamicRec.unlock();
    //     getNewFrame(camera, frame);
    //     publishers.updatePublishers(camera_node, camera, frame);
    //     rclcpp::spin_some(camera_node);
    //     m_mtxDynamicRec2.unlock();
    // }
    // publishers.deletePublishers(camera);

    return 0;
}
