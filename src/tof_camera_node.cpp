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
#include "aditof/system.h"
#include <publisher_factory.h>
#include <string>

#include "image_transport/image_transport.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
// #include "rclcpp/rclcpp.hpp"

using namespace aditof;

// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono>     // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory>     // Dynamic memory management
#include <string>     // String functions
#include <map>
// ROS Client Library for C++
// Allows use of the most common elements of ROS 2
#include "rclcpp/rclcpp.hpp"

// Built-in message type that will be used to publish data
#include "std_msgs/msg/string.hpp"

// chrono_literals handles user-defined time durations (e.g. 500ms)
using namespace std::chrono_literals;
 
// Create the node class named MinimalPublisher which inherits the attributes
// and methods of the rclcpp::Node class.
class TofNode : public rclcpp::Node
{
private:
  // Initializing camera and establishing connection
  std::shared_ptr<Camera> camera;
  aditof::Frame **frame;

  PublisherFactory publishers;
  Frame *tmp;

public:
  TofNode(std::string *arguments, std::shared_ptr<Camera> camera)
      : Node("tof_camera_node")
  {

    this->camera = camera;
    tmp = new Frame;
    frame = &tmp;
    startCamera(camera);
    publishers.createNew(this, camera, frame, (arguments[2] == "true") ? true : false);


    timer_ = this->create_wall_timer(
        100ms, std::bind(&TofNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    getNewFrame(camera, frame);
    publishers.updatePublishers(camera, frame);
  }
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{

  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // pos 0 - ip
  // pos 1 - config_path
  // pos 2 - use_depthCompute
  // pos 3 - mode
  std::string *arguments = parseArgs(argc, argv);

  std::shared_ptr<Camera> camera = initCamera(arguments);

  if(!camera)
  {
      LOG(WARNING) << "No cameras found";
      return 0;
  }

  // Setting camera parameters
  int m_mode = atoi(arguments[3].c_str());
  switch (m_mode)
  {
  case 1:
    // LR - QMP mode of the camera
    (arguments[2] == "true") ? enableCameraDepthCompute(camera, true) : enableCameraDepthCompute(camera, false);
    setFrameType(camera, "lrqmp");
    break;
  case 2:
    // LR - MP mode of the camera
    (arguments[2] == "true") ? enableCameraDepthCompute(camera, true) : enableCameraDepthCompute(camera, false);
    setFrameType(camera, "lrmp");
    break;
  case 3:
    // VGA mode of the camera (Tenbin)
    setFrameType(camera, "vga");
    break;
  }
  // Start processing data from the node as well as the callbacks and the timer
  rclcpp::spin(std::make_shared<TofNode>(arguments, camera));

  // Shutdown the node when finished
  rclcpp::shutdown();
  return 0;
}
