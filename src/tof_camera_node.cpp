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
#include <publisher_factory.h>

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

// ROS Client Library for C++
// Allows use of the most common elements of ROS 2
#include "rclcpp/rclcpp.hpp"

// Built-in message type that will be used to publish data
#include "std_msgs/msg/string.hpp"

// chrono_literals handles user-defined time durations (e.g. 500ms)
using namespace std::chrono_literals;

// int main(int argc, char **argv)
// {
//     /*
//     pos 0 - ip
//     pos 1 - config_path
//     pos 2 - use_depthCompute
//     pos 3 - mode
//     */
//     // std::string *arguments = parseArgs(argc, argv);

//     // // Initializing camera and establishing connection
//     // std::shared_ptr<Camera> camera = initCamera(arguments);

//     // // Setting camera parameters
//     // int m_mode = atoi(arguments[3].c_str());
//     // switch(m_mode)
//     // {
//     //     case 1:
//     //         //LR - QMP mode of the camera
//     //         (arguments[2] == "true") ? enableCameraDepthCompute(camera, true) : enableCameraDepthCompute(camera, false);
//     //         setFrameType(camera, "lrqmp");
//     //         break;
//     //     case 2:
//     //         //LR - MP mode of the camera
//     //         (arguments[2] == "true") ? enableCameraDepthCompute(camera, true) : enableCameraDepthCompute(camera, false);
//     //         setFrameType(camera, "lrmp");
//     //         break;
//     //     case 3:
//     //         //VGA mode of the camera
//     //         setFrameType(camera, "vga");
//     //         break;
//     //     default:
//     //     //wrong statement
//     //     return 0;
//     // }

//     // Creating camera frame for the API
//     // auto tmp = new Frame;
//     // aditof::Frame **frame = &tmp;
//     // // startCamera(camera);

//     // // Creating camera node
//     rclcpp::init(argc, argv);
//     // rclcpp::NodeOptions options;

//     // rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("tof_camera_publisher", options);

//     // Creating Image transporter
//     // image_transport::ImageTransport it(node);

//         // getNewFrame(camera, frame);
//         // publishers.updatePublishers(camera, frame);
//     rclcpp::spin(std::make_shared<TofNode>());
//     rclcpp::shutdown();
//     return 0;
// }

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
  image_transport::ImageTransport *it;

  int m_adsd3500ABinvalidationThreshold_;
  int m_adsd3500ConfidenceThreshold_;
  bool m_adsd3500JBLFfilterEnableState_;
  int m_adsd3500JBLFfilterSize_;
  int m_adsd3500RadialThresholdMin_;
  int m_adsd3500RadialThresholdMax_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

public:
  // Constructor creates a node named minimal_publisher.
  // The published message count is initialized to 0.
  TofNode()
      : Node("tof_camera_node"), count_(0)
  {
    this->declare_parameter("adsd3500ABinvalidationThreshold", 0);
    this->declare_parameter("adsd3500ConfidenceThreshold", 0);
    this->declare_parameter("adsd3500JBLFfilterEnableState", false);
    this->declare_parameter("adsd3500JBLFfilterSize", 0);
    this->declare_parameter("adsd3500RadialThresholdMin", 0);
    this->declare_parameter("adsd3500RadialThresholdMax", 0);

    m_adsd3500ABinvalidationThreshold_ = this->get_parameter("adsd3500ABinvalidationThreshold").as_int();
    m_adsd3500ConfidenceThreshold_ = this->get_parameter("adsd3500ConfidenceThreshold").as_int();
    m_adsd3500JBLFfilterEnableState_ = this->get_parameter("adsd3500JBLFfilterEnableState").as_bool();
    m_adsd3500JBLFfilterSize_ = this->get_parameter("adsd3500JBLFfilterSize").as_int();
    m_adsd3500RadialThresholdMin_ = this->get_parameter("adsd3500RadialThresholdMin").as_int();
    m_adsd3500RadialThresholdMax_ = this->get_parameter("adsd3500RadialThresholdMax").as_int();
    // camera = initCamera(arguments);
    // tmp = new Frame;
    // frame = &tmp;
    // it = new image_transport::Image_transport(selfe);
    // startCamera(camera);
    // publishers.createNew(it, camera, frame, (arguments[2] == "true") ? true : false);

    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&TofNode::parametersCallback, this, std::placeholders::_1));

    // Publisher publishes String messages to a topic named "addison".
    // The size of the queue is 10 messages.
    publisher_ = this->create_publisher<std_msgs::msg::String>("addison", 10);
    // Initialize the timer. The timer_callback function will execute every
    // 500 milliseconds.
    timer_ = this->create_wall_timer(
        500ms, std::bind(&TofNode::timer_callback, this));
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // Here update class attributes, do some actions, etc.
    return result;
  }

private:
  // This method executes every 500 milliseconds
  void timer_callback()
  {
    // getNewFrame(camera, frame);
    // publishers.updatePublishers(camera, frame);

    // Create a new message of type String
    auto message = std_msgs::msg::String();
    // Set our message's data attribute and increment the message count by 1
    message.data = "Hi Automatic Addison! " + std::to_string(count_++);
    // Print every message to the terminal window
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // Publish the message to the topic named "addison"
    publisher_->publish(message);
  }

  // Declaration of the timer_ attribute
  rclcpp::TimerBase::SharedPtr timer_;

  // Declaration of the publisher_ attribute
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // Declaration of the count_ attribute
  size_t count_;
};

// Node execution starts here
int main(int argc, char *argv[])
{

  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // pos 0 - ip
  // pos 1 - config_path
  // pos 2 - use_depthCompute
  // pos 3 - mode

  // std::string *arguments = parseArgs(argc, argv);

  // Setting camera parameters
  // int m_mode = atoi(arguments[3].c_str());
  // switch(m_mode)
  // {
  //     case 1:
  //         //LR - QMP mode of the camera
  //         (arguments[2] == "true") ? enableCameraDepthCompute(camera, true) : enableCameraDepthCompute(camera, false);
  //         setFrameType(camera, "lrqmp");
  //         break;
  //     case 2:
  //         //LR - MP mode of the camera
  //         (arguments[2] == "true") ? enableCameraDepthCompute(camera, true) : enableCameraDepthCompute(camera, false);
  //         setFrameType(camera, "lrmp");
  //         break;
  //     case 3:
  //         //VGA mode of the camera
  //         setFrameType(camera, "vga");
  //         break;
  //     default:
  //     //wrong statement
  //     return 0;
  // }

  // Start processing data from the node as well as the callbacks and the timer
  rclcpp::spin(std::make_shared<TofNode>());
  // Shutdown the node when finished
  rclcpp::shutdown();
  return 0;
}