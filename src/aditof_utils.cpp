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
#include "aditof_utils.h"

#include <aditof/system.h>
#include <string.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <regex>

std::mutex mtx_dynamic_rec;
using namespace aditof;

std::string * parseArgs(int argc, char ** argv)
{
  // pos 0 - ip
  // pos 1 - config_path
  // pos 2 - mode

  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = 1;
  std::string ip = "";
  std::string config_path = "config/config_adsd3500_adsd3100.json";
  std::string mode = "0";
  std::string enable_multithread = "False";

  for (int i = 1; i < argc; i++) {
    std::string left;
    std::string right;
    std::string argnew(argv[i]);

    left = argnew.substr(0, argnew.find("=", 0));
    right = argnew.substr(argnew.find("=", 0) + 1, argnew.size());

    if (std::strcmp(left.c_str(), "ip") == 0)
      ip = right;
    else if (std::strcmp(left.c_str(), "config_file") == 0)
      config_path = right;
    else if (std::strcmp(left.c_str(), "mode") == 0)
      mode = right;
    else if (std::strcmp(left.c_str(), "enable_multithread") == 0)
      enable_multithread = right;
  }

  if (ip.empty()) {
    LOG(INFO) << "No ip provided, attempting to connect to the camera "
                 "through USB";
  }
  if (config_path.empty()) {
    LOG(INFO) << "Config file not provided!";
  }
  if (mode.empty()) {
    LOG(INFO) << "Camera mode not provided!";
  }

  std::string * result = new std::string[4];
  if (!ip.empty()) {
    result[0] = std::string("ip:") + ip;
  }
  result[1] = config_path;
  result[2] = mode;
  result[3] = enable_multithread;

  return result;
}

std::shared_ptr<Camera> initCamera(std::string * arguments)
{
  Status status = Status::OK;
  LOG(INFO) << "Started camera intialization";

  System system;

  std::vector<std::shared_ptr<Camera>> cameras;
  if (arguments[0].empty()) {
    system.getCameraList(cameras);
  } else {
    system.getCameraList(cameras, arguments[0]);
  }

  if (cameras.empty()) {
    LOG(WARNING) << "No cameras found";
    return nullptr;
  }

  std::shared_ptr<Camera> camera = cameras.front();

  status = camera->initialize(arguments[1]);
  if (status != Status::OK) {
    LOG(ERROR) << "Could not initialize camera!";
    return 0;
  }

  return camera;
}

void getAvailableFrameTypes(
  const std::shared_ptr<aditof::Camera> & camera, std::vector<std::string> & availableFrameTypes)
{
  //get available frae types of camera
  aditof::Status status = aditof::Status::OK;

  status = camera->getAvailableFrameTypes(availableFrameTypes);
  if (status != Status::OK) {
    LOG(ERROR) << "Couldn't get available frame types";
    return;
  }
}

void enableCameraDepthCompute(const std::shared_ptr<aditof::Camera> & camera, const bool value)
{
  //set depthCompute to on or off
  aditof::Status status = aditof::Status::OK;

  status = camera->setControl("enableDepthCompute", (value) ? "on" : "off");
  if (status != Status::OK) {
    LOG(ERROR) << "Couldn't set depth compute option";
    return;
  }
}

void startCamera(const std::shared_ptr<aditof::Camera> & camera)
{
  Status status = Status::OK;

  status = camera->start();
  if (status != Status::OK) {
    LOG(ERROR) << "Could not start camera!";
    return;
  } else {
    LOG(INFO) << "Camera started";
  }
  return;
}

void stopCamera(const std::shared_ptr<aditof::Camera> & camera)
{
  Status status = Status::OK;

  status = camera->stop();
  if (status != Status::OK) {
    LOG(ERROR) << "Could not stop camera!";
    return;
  } else {
    LOG(INFO) << "Camera stopped";
  }
  return;
}

void setFrameType(const std::shared_ptr<aditof::Camera> & camera, const std::string & type)
{
  std::vector<std::string> frameTypes;
  camera->getAvailableFrameTypes(frameTypes);
  if (frameTypes.empty()) {
    LOG(ERROR) << "No frame type available!";
    return;
  }

  std::vector<std::string>::iterator it = std::find(frameTypes.begin(), frameTypes.end(), type);
  if (it == frameTypes.end()) {
    LOG(ERROR) << "Requested frame type is not available";
    return;
  }

  Status status = Status::OK;
  status = camera->setFrameType(type);
  if (status != Status::OK) {
    LOG(ERROR) << "Could not set camera frame type!";
    return;
  } else {
    LOG(INFO) << "Frame type set: " << type;
  }
}

void getAvailableFrameType(
  const std::shared_ptr<aditof::Camera> & camera, std::vector<std::string> & availableFrameTypes)
{
  camera->getAvailableFrameTypes(availableFrameTypes);
  if (availableFrameTypes.empty()) {
    LOG(ERROR) << "No frame type available!";
    return;
  }
}

void getCameraDataDetails(
  const std::shared_ptr<aditof::Camera> & camera, aditof::CameraDetails & details)
{
  Status status = camera->getDetails(details);
  if (status != Status::OK) {
    LOG(ERROR) << "Couldn't get camera details!";
    return;
  }
}

void setMode(const std::shared_ptr<aditof::Camera> & camera, const std::string & mode)
{
  std::vector<std::string> modes;
  camera->getAvailableModes(modes);
  if (modes.empty()) {
    LOG(ERROR) << "No camera modes available!";
    return;
  }

  std::vector<std::string>::iterator it = std::find(modes.begin(), modes.end(), mode);
  if (it == modes.end()) {
    LOG(ERROR) << "Requested mode is not available";
    return;
  }

  Status status = Status::OK;
  status = camera->setMode(mode);
  if (status != Status::OK) {
    LOG(ERROR) << "Could not set camera mode!";
    return;
  }
}

void setCameraRevision(const std::shared_ptr<aditof::Camera> & camera, const std::string rev)
{
  Status status = Status::OK;
  status = camera->setControl("revision", rev);
  if (status != Status::OK) {
    LOG(ERROR) << "Could not set camera revision!";
    return;
  }
}

void setIrGammaCorrection(const std::shared_ptr<aditof::Camera> & camera, float gamma)
{
  Status status = Status::OK;
  status = camera->setControl("ir_gamma_correction", std::to_string(gamma));
  if (status != Status::OK) {
    LOG(ERROR) << "Could not set ir gamma correction!";
    return;
  }
}

void applyNoiseReduction(const std::shared_ptr<Camera> & camera, int threshold)
{
  Status status = Status::OK;
  status = camera->setControl("noise_reduction_threshold", std::to_string(threshold));
  if (status != Status::OK) {
    LOG(ERROR) << "Could not set noise reduction!";
    return;
  }
}

void disableNoiseReduction(const std::shared_ptr<Camera> & camera)
{
  Status status = Status::OK;
  status = camera->setControl("noise_reduction_threshold", std::to_string(0));
  if (status != Status::OK) {
    LOG(ERROR) << "Could not set noise reduction!";
    return;
  }
}

void getNewFrame(const std::shared_ptr<Camera> & camera, aditof::Frame * frame)
{
  Status status = Status::OK;

  try {
    status = camera->requestFrame(frame);
    if (status != Status::OK) {
      LOG(ERROR) << "Could not request frame!";
    }
  } catch (std::exception & e) {
  }
}

uint16_t * getFrameData(aditof::Frame * frame, const std::string & dataType)
{
  uint16_t * frameData;
  Status status = Status::OK;

  status = frame->getData(dataType, &frameData);

  if (status != Status::OK) {
    LOG(ERROR) << "Could not get frame data!";
    return nullptr;
  }

  if (!frameData) {
    LOG(ERROR) << "no memory allocated in frame";
    return nullptr;
  }
  return frameData;
}

IntrinsicParameters getIntrinsics(const std::shared_ptr<Camera> & camera)
{
  aditof::CameraDetails cameraDetails;
  camera->getDetails(cameraDetails);
  return cameraDetails.intrinsics;
}

int getRangeMax(const std::shared_ptr<Camera> & camera)
{
  aditof::CameraDetails cameraDetails;
  camera->getDetails(cameraDetails);
  return cameraDetails.maxDepth;
}

int getRangeMin(const std::shared_ptr<Camera> & camera)
{
  aditof::CameraDetails cameraDetails;
  camera->getDetails(cameraDetails);
  return cameraDetails.minDepth;
}

void irTo16bitGrayscale(uint16_t * frameData, int width, int height, bool shifteight)
{
  std::vector<uint16_t> data(frameData, frameData + width * height);

  auto min_val = 0;       //std::min_element(data.begin(), data.end());
  auto max_val = 0x0fff;  //std::max_element(data.begin(), data.end());
  uint16_t delta = max_val - min_val;
  int minColorValue = 0;

  if (delta == 0) {
    return;
  }

  for (int i = 0; i < width * height; i++) {
    float norm_val = static_cast<float>(data[i] - min_val) / delta;
    float grayscale_val =
      norm_val * std::numeric_limits<unsigned short int>::max() + (1.0f - norm_val) * minColorValue;
    frameData[i] = static_cast<uint16_t>(grayscale_val);
    if (shifteight == true) {
      frameData[i] = frameData[i] << 8;
    }
  }
}

enum ModeTypes intToMode(int var)
{
  ModeTypes newMode;
  switch (var) {
    case 1:
      newMode = ModeTypes::mode0;
      break;
    case 2:
      newMode = ModeTypes::mode1;
      break;
    case 3:
      newMode = ModeTypes::mode2;
      break;
    case 4:
      newMode = ModeTypes::mode3;
      break;
  }
  return (newMode);
}

void control_adsd3500SetABinvalidationThreshold(
  const std::shared_ptr<aditof::Camera> & camera, int threshold)
{
  Status status = Status::OK;
  status = camera->adsd3500SetABinvalidationThreshold(threshold);
  if (status != Status::OK) {
    LOG(ERROR) << "Could not set adsd3500 set AB invalidation Threshold!";
    return;
  }
}
void control_adsd3500GetABinvalidationThreshold(
  const std::shared_ptr<aditof::Camera> & camera, int & threshold)
{
  Status status = Status::OK;
  status = camera->adsd3500GetABinvalidationThreshold(threshold);
  if (status != Status::OK) {
    LOG(ERROR) << "Could not set adsd3500 set AB invalidation Threshold!";
    return;
  }
}
void control_adsd3500SetConfidenceThreshold(
  const std::shared_ptr<aditof::Camera> & camera, int threshold)
{
  Status status = Status::OK;
  status = camera->adsd3500SetConfidenceThreshold(threshold);
  if (status != Status::OK) {
    LOG(ERROR) << "Could not set adsd3500 set AB invalidation Threshold!";
    return;
  }
}
void control_adsd3500GetConfidenceThreshold(
  const std::shared_ptr<aditof::Camera> & camera, int & threshold)
{
  Status status = Status::OK;
  status = camera->adsd3500GetConfidenceThreshold(threshold);
  if (status != Status::OK) {
    LOG(ERROR) << "Could not set adsd3500 set AB invalidation Threshold!";
    return;
  }
}
void control_adsd3500SetJBLFfilterEnableState(
  const std::shared_ptr<aditof::Camera> & camera, bool enable)
{
  Status status = Status::OK;
  status = camera->adsd3500SetJBLFfilterEnableState(enable);
  if (status != Status::OK) {
    LOG(ERROR) << "Could not set adsd3500 set AB invalidation Threshold!";
    return;
  }
}
void control_adsd3500GetJBLFfilterEnableState(
  const std::shared_ptr<aditof::Camera> & camera, bool & enable)
{
  Status status = Status::OK;
  status = camera->adsd3500GetJBLFfilterEnableState(enable);
  if (status != Status::OK) {
    LOG(ERROR) << "Could not set adsd3500 set AB invalidation Threshold!";
    return;
  }
}
void control_adsd3500SetJBLFfilterSize(
  const std::shared_ptr<aditof::Camera> & camera, int threshold)
{
  Status status = Status::OK;
  status = camera->adsd3500SetJBLFfilterSize(threshold);
  if (status != Status::OK) {
    LOG(ERROR) << "Could not set adsd3500 set AB invalidation Threshold!";
    return;
  }
}
void control_adsd3500GetJBLFfilterSize(
  const std::shared_ptr<aditof::Camera> & camera, int & threshold)
{
  Status status = Status::OK;
  status = camera->adsd3500GetJBLFfilterSize(threshold);
  if (status != Status::OK) {
    LOG(ERROR) << "Could not set adsd3500 set AB invalidation Threshold!";
    return;
  }
}
void control_adsd3500SetRadialThresholdMin(
  const std::shared_ptr<aditof::Camera> & camera, int threshold)
{
  Status status = Status::OK;
  status = camera->adsd3500SetRadialThresholdMin(threshold);
  if (status != Status::OK) {
    LOG(ERROR) << "Could not set adsd3500 set AB invalidation Threshold!";
    return;
  }
}
void control_adsd3500GetRadialThresholdMin(
  const std::shared_ptr<aditof::Camera> & camera, int & threshold)
{
  Status status = Status::OK;
  status = camera->adsd3500GetRadialThresholdMin(threshold);
  if (status != Status::OK) {
    LOG(ERROR) << "Could not set adsd3500 set AB invalidation Threshold!";
    return;
  }
}
void control_adsd3500SetRadialThresholdMax(
  const std::shared_ptr<aditof::Camera> & camera, int threshold)
{
  Status status = Status::OK;
  status = camera->adsd3500SetRadialThresholdMax(threshold);
  if (status != Status::OK) {
    LOG(ERROR) << "Could not set adsd3500 set AB invalidation Threshold!";
    return;
  }
}
void control_adsd3500GetRadialThresholdMax(
  const std::shared_ptr<aditof::Camera> & camera, int & threshold)
{
  Status status = Status::OK;
  status = camera->adsd3500GetRadialThresholdMax(threshold);
  if (status != Status::OK) {
    LOG(ERROR) << "Could not set adsd3500 set AB invalidation Threshold!";
    return;
  }
}

void versioningAuxiliaryFunction(const std::shared_ptr<aditof::Camera> & camera)
{
  aditof::CameraDetails cameraDetails;
  camera->getDetails(cameraDetails);

  LOG(INFO) << "SD card image version: " << cameraDetails.sdCardImageVersion;
  LOG(INFO) << "Kernel version: " << cameraDetails.kernelVersion;
  LOG(INFO) << "U-Boot version: " << cameraDetails.uBootVersion;
}
