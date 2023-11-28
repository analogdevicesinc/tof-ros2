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
#include <aditof_sensor_msg.h>
#include <aditof_utils.h>
#include <publisher_factory.h>

#include <chrono>
#include <functional>  // Arithmetic, comparisons, and logical operations
#include <map>
#include <memory>  // Dynamic memory management
#include <rclcpp/rclcpp.hpp>
#include <string>  // String functions
#include <thread>

#include "aditof/camera.h"
#include "aditof/system.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "safedataaccess.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace aditof;

// Create the node class named MinimalPublisher which inherits the attributes
// and methods of the rclcpp::Node class.

class TofNode : public rclcpp::Node
{
private:
  // Initializing camera and establishing connection
  std::shared_ptr<Camera> camera;
  aditof::Frame ** frame;
  PublisherFactory publishers;

public:
  SafeDataAccess<aditof::Frame *> m_safeDataAccess;

private:
  // camera parameters
  int m_adsd3500ABinvalidationThreshold_;
  int m_adsd3500ConfidenceThreshold_;
  bool m_adsd3500JBLFfilterEnableState_;
  int m_adsd3500JBLFfilterSize_;
  int m_adsd3500RadialThresholdMin_;
  int m_adsd3500RadialThresholdMax_;

  bool m_ir_thread;
  bool m_depth_thread;
  bool m_raw_thread;
  bool m_conf_thread;
  bool m_xyz_thread;

private:
  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    if (
      m_ir_thread != get_parameter("ir").as_bool() ||
      m_depth_thread != get_parameter("depth").as_bool() ||
      m_raw_thread != get_parameter("raw").as_bool() ||
      m_conf_thread != get_parameter("conf").as_bool() ||
      m_xyz_thread != get_parameter("xyz").as_bool()) {
      publishers.setThreadMode("ir", get_parameter("ir").as_bool());
      publishers.setThreadMode("depth", get_parameter("depth").as_bool());
      publishers.setThreadMode("raw", get_parameter("raw").as_bool());
      publishers.setThreadMode("conf", get_parameter("conf").as_bool());
      publishers.setThreadMode("xyz", get_parameter("xyz").as_bool());

      m_ir_thread = get_parameter("ir").as_bool();
      m_depth_thread = get_parameter("depth").as_bool();
      m_raw_thread = get_parameter("raw").as_bool();
      m_conf_thread = get_parameter("conf").as_bool();
      m_xyz_thread = get_parameter("xyz").as_bool();
    } else if (
      m_adsd3500ABinvalidationThreshold_ !=
        get_parameter("adsd3500ABinvalidationThreshold").as_int() ||
      m_adsd3500ConfidenceThreshold_ != get_parameter("adsd3500ConfidenceThreshold").as_int() ||
      m_adsd3500JBLFfilterEnableState_ != get_parameter("adsd3500JBLFfilterEnableStat").as_bool() ||
      m_adsd3500JBLFfilterSize_ != get_parameter("adsd3500JBLFfilterSize").as_int() ||
      m_adsd3500RadialThresholdMin_ != get_parameter("adsd3500RadialThresholdMin").as_int() ||
      m_adsd3500RadialThresholdMax_ != get_parameter("adsd3500RadialThresholdMax").as_int()) {
      // Stream off //temporary solution, replace if we can modify during runtime of the camera
      stopCamera(camera);
      streamOnFlag = false;

      control_adsd3500SetABinvalidationThreshold(
        camera, get_parameter("adsd3500ABinvalidationThreshold").as_int());
      control_adsd3500SetConfidenceThreshold(
        camera, get_parameter("adsd3500ConfidenceThreshold").as_int());
      control_adsd3500SetJBLFfilterEnableState(
        camera, get_parameter("adsd3500JBLFfilterEnableStat").as_bool());
      control_adsd3500SetJBLFfilterSize(camera, get_parameter("adsd3500JBLFfilterSize").as_int());
      control_adsd3500SetRadialThresholdMin(
        camera, get_parameter("adsd3500RadialThresholdMin").as_int());
      control_adsd3500SetRadialThresholdMax(
        camera, get_parameter("adsd3500RadialThresholdMax").as_int());

      startCamera(camera);
      streamOnFlag = true;

      m_adsd3500ABinvalidationThreshold_ =
        get_parameter("adsd3500ABinvalidationThreshold").as_int();
      m_adsd3500ConfidenceThreshold_ = get_parameter("adsd3500ConfidenceThreshold").as_int();
      m_adsd3500JBLFfilterEnableState_ = get_parameter("adsd3500JBLFfilterEnableStat").as_bool();
      m_adsd3500JBLFfilterSize_ = get_parameter("adsd3500JBLFfilterSize").as_int();
      m_adsd3500RadialThresholdMin_ = get_parameter("adsd3500RadialThresholdMin").as_int();
      m_adsd3500RadialThresholdMax_ = get_parameter("adsd3500RadialThresholdMax").as_int();
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    return result;
  }

  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

public:
  TofNode(std::string * arguments, std::shared_ptr<Camera> camera, aditof::Frame ** frame)
  : Node("tof_camera_node")
  {
    m_ir_thread = true;
    m_depth_thread = true;
    m_raw_thread = true;
    m_conf_thread = true;
    m_xyz_thread = true;

    this->declare_parameter("ir", true);
    this->declare_parameter("depth", true);
    this->declare_parameter("raw", true);
    this->declare_parameter("conf", true);
    this->declare_parameter("xyz", true);

    this->get_parameter("ir", m_ir_thread);
    this->get_parameter("depth", m_depth_thread);
    this->get_parameter("raw", m_raw_thread);
    this->get_parameter("conf", m_conf_thread);
    this->get_parameter("xyz", m_xyz_thread);

    this->declare_parameter("adsd3500ABinvalidationThreshold", 0);
    this->declare_parameter("adsd3500ConfidenceThreshold", 0);
    this->declare_parameter("adsd3500JBLFfilterEnableStat", false);
    this->declare_parameter("adsd3500JBLFfilterSize", 0);
    this->declare_parameter("adsd3500RadialThresholdMin", 0);
    this->declare_parameter("adsd3500RadialThresholdMax", 0);

    this->get_parameter("adsd3500ABinvalidationThreshold", m_adsd3500ABinvalidationThreshold_);
    this->get_parameter("adsd3500ConfidenceThreshold", m_adsd3500ConfidenceThreshold_);
    this->get_parameter("adsd3500JBLFfilterEnableStat", m_adsd3500JBLFfilterEnableState_);
    this->get_parameter("adsd3500JBLFfilterSize", m_adsd3500JBLFfilterSize_);
    this->get_parameter("adsd3500RadialThresholdMin", m_adsd3500RadialThresholdMin_);
    this->get_parameter("adsd3500RadialThresholdMax", m_adsd3500RadialThresholdMax_);

    this->camera = camera;
    this->frame = frame;

    // add 12 items in the list needed for circular vector
    for (int i = 0; i <= 12; i++) {
      m_safeDataAccess.populateData(new aditof::Frame);
    }

    if (!streamOnFlag) {
      startCamera(camera);
      streamOnFlag = true;
    }

    publishers.createNew(this, camera, frame, &m_safeDataAccess);

    callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&TofNode::parameterCallback, this, std::placeholders::_1));

    if (
      (std::strcmp(arguments[3].c_str(), "True") == 0) ||
      (std::strcmp(arguments[3].c_str(), "true") == 0))
      publishers.createMultiThreadPublisherWorkers(camera, frame);
    else
      publishers.createSingleThreadPublisherWorker(camera, frame);
  }

  void service_callback()
  {
    if (streamOnFlag) {
      globalTimeStamp = rclcpp::Clock{RCL_ROS_TIME}.now();

      aditof::Frame * tmp1 = m_safeDataAccess.getNextElement();

      m_safeDataAccess.setReadytoStart();
      getNewFrame(camera, tmp1);
      m_safeDataAccess.addElement(tmp1);
    }
  }

  void stopNode()
  {
    publishers.removePublisherWorkers();
    publishers.deletePublishers(camera);
  }
};

int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  std::string * arguments = parseArgs(argc, argv);
  // find camera (local/usb/network), set config file and initialize the camera
  std::shared_ptr<Camera> camera = initCamera(arguments);
  // versioning print
  versioningAuxiliaryFunction(camera);

  if (!camera) {
    LOG(WARNING) << "No cameras found";
    return 0;
  }

  // getting available frame types, backward compatibility
  std::vector<std::string> availableFrameTypes;
  getAvailableFrameTypes(camera, availableFrameTypes);

  // Setting camera parameters
  int currentMode = atoi(arguments[2].c_str());
  int availableFrameTypeSize = availableFrameTypes.size();

  if (0 <= currentMode && currentMode < availableFrameTypeSize) {
    setFrameType(camera, availableFrameTypes.at(currentMode));
  } else {
    LOG(ERROR) << "Incompatible or unavalable mode type chosen";
    return 0;
  }

  auto tmp = new Frame;
  aditof::Frame ** frame = &tmp;

  // Create ToF Node
  std::shared_ptr<TofNode> tof_node = std::make_shared<TofNode>(arguments, camera, frame);

  while (rclcpp::ok()) {
    tof_node->service_callback();
    rclcpp::spin_some(tof_node);
  }

  tof_node->stopNode();

  // Shutdown the node when finished
  rclcpp::shutdown();
  return 0;
}
