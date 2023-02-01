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
#include "aditof/camera.h"
#include <aditof_sensor_msg.h>
#include "aditof/system.h"

#include <rclcpp/rclcpp.hpp>
#include <publisher_factory.h>
#include "image_transport/image_transport.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory>     // Dynamic memory management
#include <string>     // String functions
#include <map>

using namespace std::chrono_literals;
using namespace aditof;

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
    bool m_streamOnFlag = 0;

    // camera parameters
    int m_adsd3500ABinvalidationThreshold_;
    int m_adsd3500ConfidenceThreshold_;
    bool m_adsd3500JBLFfilterEnableState_;
    int m_adsd3500JBLFfilterSize_;
    int m_adsd3500RadialThresholdMin_;
    int m_adsd3500RadialThresholdMax_;

private:
    rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        //Stream off //temporary solution, replace if we can modify during runtime of the camera

        stopCamera(camera);
        m_streamOnFlag = 0;

        control_adsd3500SetABinvalidationThreshold(camera, get_parameter("adsd3500ABinvalidationThreshold").as_int());
        control_adsd3500SetConfidenceThreshold(camera, get_parameter("adsd3500ConfidenceThreshold").as_int());
        control_adsd3500SetJBLFfilterEnableState(camera, get_parameter("adsd3500JBLFfilterEnableStat").as_bool());
        control_adsd3500SetJBLFfilterSize(camera, get_parameter("adsd3500JBLFfilterSize").as_int());
        control_adsd3500SetRadialThresholdMin(camera, get_parameter("adsd3500RadialThresholdMin").as_int());
        control_adsd3500SetRadialThresholdMax(camera, get_parameter("adsd3500RadialThresholdMax").as_int());

        startCamera(camera);
        m_streamOnFlag = 1;

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        return result;
    }

    void timer_callback()
    {
        getNewFrame(camera, frame);
        publishers.updatePublishers(camera, frame);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;


public:
    TofNode(std::string *arguments, std::shared_ptr<Camera> camera)
        : Node("tof_camera_node")
    {
        this->declare_parameter("adsd3500ABinvalidationThreshold", 0);
        this->declare_parameter("adsd3500ConfidenceThreshold",0);
        this->declare_parameter("adsd3500JBLFfilterEnableStat",false);
        this->declare_parameter("adsd3500JBLFfilterSize",0);
        this->declare_parameter("adsd3500RadialThresholdMin",0);
        this->declare_parameter("adsd3500RadialThresholdMax",0);

        this->get_parameter("adsd3500ABinvalidationThreshold", m_adsd3500ABinvalidationThreshold_);
        this->get_parameter("adsd3500ConfidenceThreshold", m_adsd3500ConfidenceThreshold_);
        this->get_parameter("adsd3500JBLFfilterEnableStat", m_adsd3500JBLFfilterEnableState_);
        this->get_parameter("adsd3500JBLFfilterSize", m_adsd3500JBLFfilterSize_);
        this->get_parameter("adsd3500RadialThresholdMin", m_adsd3500RadialThresholdMin_);
        this->get_parameter("adsd3500RadialThresholdMax", m_adsd3500RadialThresholdMax_);

        this->camera = camera;
        tmp = new Frame;
        frame = &tmp;

        if (!m_streamOnFlag)
        {
            startCamera(camera);
            m_streamOnFlag = true;
        }

        publishers.createNew(this, camera, frame, (arguments[2] == "true") ? true : false);

        timer_ = this->create_wall_timer(100ms, std::bind(&TofNode::timer_callback, this));
        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&TofNode::parameterCallback, this,std::placeholders::_1));
    }

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

    if (!camera)
    {
        LOG(WARNING) << "No cameras found";
        return 0;
    }

    //getting available frame types, backward compatibility
    std::vector<std::string> availableFrameTypes;
    getAvailableFrameTypes(camera, availableFrameTypes);

    //In case old modes are available
    if (availableFrameTypes.size() > 1)
    {
        // Setting camera parameters
        int m_mode = atoi(arguments[3].c_str());
        switch (m_mode)
        {
        case 0:
            // LR - QMP mode of the camera
            (arguments[2] == "true") ? enableCameraDepthCompute(camera, true) : enableCameraDepthCompute(camera, false);
            setFrameType(camera, availableFrameTypes.at(0));
            m_currentMode = ModeTypes::mode1; 
            break;
        case 1:
            // LR - MP mode of the camera
            (arguments[2] == "true") ? enableCameraDepthCompute(camera, true) : enableCameraDepthCompute(camera, false);
            setFrameType(camera, availableFrameTypes.at(1));
            m_currentMode = ModeTypes::mode1; 
            break;
        case 2:
            (arguments[2] == "true") ? enableCameraDepthCompute(camera, true) : enableCameraDepthCompute(camera, false);
            setFrameType(camera, availableFrameTypes.at(2));
            m_currentMode = ModeTypes::mode1; 
            break;
        case 3:
            (arguments[2] == "true") ? enableCameraDepthCompute(camera, true) : enableCameraDepthCompute(camera, false);
            setFrameType(camera, availableFrameTypes.at(3));
            m_currentMode = ModeTypes::mode1; 
            break;
        }
    }
    else
    {
        enableCameraDepthCompute(camera, m_enableDepthCompute);
        setFrameType(camera, availableFrameTypes.at(0));
        m_currentMode = ModeTypes::mode1; 
    }
    // Start processing data from the node as well as the callbacks and the timer
    rclcpp::spin(std::make_shared<TofNode>(arguments, camera));

    // Shutdown the node when finished
    rclcpp::shutdown();
    return 0;
}
