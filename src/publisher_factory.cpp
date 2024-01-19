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

#include "publisher_factory.h"

bool streamOnFlag;
rclcpp::Time globalTimeStamp;
int pub_factory_adsd3500_status = -1;

PublisherFactory::PublisherFactory(){};

void PublisherFactory::createNew(
  rclcpp::Node * node, const std::shared_ptr<aditof::Camera> & camera, aditof::Frame ** frame,
  SafeDataAccess<aditof::Frame *> * safeDataAccess)
{
  m_safeDataAccess = safeDataAccess;
  // Get frame types
  aditof::CameraDetails * details_tmp = new aditof::CameraDetails;
  getCameraDataDetails(camera, *details_tmp);

  for (auto iter : (*details_tmp).frameType.dataDetails) {
    if (!strcmp(iter.type.c_str(), "ab")) {
      imgPublishers.emplace_back(
        node->create_publisher<sensor_msgs::msg::Image>("tof_camera/ab", 2));
      imgMsgs.emplace_back(new IRImageMsg(camera, frame, sensor_msgs::image_encodings::MONO16));
      LOG(INFO) << "Added ab publisher";
    } else if (!strcmp(iter.type.c_str(), "depth")) {
      imgPublishers.emplace_back(
        node->create_publisher<sensor_msgs::msg::Image>("tof_camera/depth", 2));
      imgMsgs.emplace_back(new DepthImageMsg(camera, frame, sensor_msgs::image_encodings::RGBA8));
      LOG(INFO) << "Added depth publisher";
    } else if (!strcmp(iter.type.c_str(), "raw")) {
      imgPublishers.emplace_back(
        node->create_publisher<sensor_msgs::msg::Image>("tof_camera/raw", 2));
      imgMsgs.emplace_back(new RAWImageMsg(camera, frame, sensor_msgs::image_encodings::MONO16));
      LOG(INFO) << "Added raw data publisher";
    } else if (!strcmp(iter.type.c_str(), "xyz")) {
      pointCloudPublishers.emplace_back(
        node->create_publisher<sensor_msgs::msg::PointCloud2>("tof_camera/xyz", 2));
      pointCloudMsgs.emplace_back(
        new XYZImageMsg(camera, frame, sensor_msgs::image_encodings::MONO16));
      LOG(INFO) << "Added xyz data publisher";
    } else if (!strcmp(iter.type.c_str(), "conf")) {
      imgPublishers.emplace_back(
        node->create_publisher<sensor_msgs::msg::Image>("tof_camera/conf", 2));
      imgMsgs.emplace_back(new ConfImageMsg(camera, frame, sensor_msgs::image_encodings::MONO16));
      LOG(INFO) << "Added conf data publisher";
    }
  }

  cameraStatusPublisher.emplace_back(
    node->create_publisher<std_msgs::msg::Int32>("tof_camera_status", 10));
}

void PublisherFactory::setThreadMode(std::string topicIndex, bool mode)
{
  int size = imgPublishers.size();
  for (int i = 0; i < size; i++) {
    const std::string topicname = std::string(imgPublishers.at(i)->get_topic_name());
    if (strstr(topicname.c_str(), topicIndex.c_str())) {
      imgMsgs.at(i)->publisherEnabled = mode;
    }
  }
  size = pointCloudPublishers.size();
  for (int i = 0; i < size; i++) {
    const std::string topicname = std::string(pointCloudPublishers.at(i)->get_topic_name());
    if (strstr(topicname.c_str(), topicIndex.c_str())) {
      pointCloudMsgs.at(i)->publisherEnabled = mode;
    }
  }
}

void PublisherFactory::createMultiThreadPublisherWorkers(
  const std::shared_ptr<aditof::Camera> & camera, aditof::Frame ** frame)
{
  deletePublisherWorkers = false;
  for (unsigned int i = 0; i < imgMsgs.size(); ++i) {
    tofThreads.emplace_back(new std::thread(
      publisherImgMsgsWorker, imgMsgs.at(i), imgPublishers.at(i), camera, frame, m_safeDataAccess));
  }
  for (unsigned int i = 0; i < pointCloudMsgs.size(); ++i) {
    tofThreads.emplace_back(new std::thread(
      publisherPointCloudMsgsWorker, pointCloudMsgs.at(i), pointCloudPublishers.at(i), camera,
      frame, m_safeDataAccess));
  }
  tofThreads.emplace_back(new std::thread(publisherCameraStatusMsgsWorker, cameraStatusPublisher));
}

void PublisherFactory::createSingleThreadPublisherWorker(
  const std::shared_ptr<aditof::Camera> & camera, aditof::Frame ** frame)
{
  deletePublisherWorkers = false;
  tofThreads.emplace_back(new std::thread(
    publisherSingleThreadWorker, imgPublishers, imgMsgs, pointCloudPublishers, pointCloudMsgs,
    cameraStatusPublisher, camera, frame, m_safeDataAccess));
}

void PublisherFactory::removePublisherWorkers()
{
  deletePublisherWorkers = true;
  for (int i = 0; i < tofThreads.size(); i++) tofThreads[i]->join();
}

void PublisherFactory::deletePublishers(const std::shared_ptr<aditof::Camera> & camera)
{
  stopCamera(camera);
  imgPublishers.clear();
  imgMsgs.clear();
}

void PublisherFactory::setDepthFormat(const int val)
{
  for (unsigned int i = 0; i < imgMsgs.size(); ++i) {
    if (std::dynamic_pointer_cast<DepthImageMsg>(imgMsgs[i])) {
      std::dynamic_pointer_cast<DepthImageMsg>(imgMsgs[i]).get()->setDepthDataFormat(val);
    }
  }
}

void publisherCameraStatusMsgsWorker(
  std::vector<rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> cameraStatus_publisher)
{
  rclcpp::Rate loop_rate(1.0 / 10);

  while (!deletePublisherWorkers) {
    if (rclcpp::ok()) {
      std_msgs::msg::Int32 msgi;
      msgi.data = pub_factory_adsd3500_status;

      if (cameraStatus_publisher.size()) cameraStatus_publisher.at(0)->publish(msgi);
      loop_rate.sleep();
    }
  }
}

void publisherImgMsgsWorker(
  std::shared_ptr<AditofSensorMsg> imgMsgs,
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher,
  const std::shared_ptr<aditof::Camera> & camera, aditof::Frame ** frame,
  SafeDataAccess<aditof::Frame *> * safeDataAccess)
{
  rclcpp::Time localTimeStamp = rclcpp::Clock{RCL_ROS_TIME}.now();
  rclcpp::Rate loop_rate(1.0 / 10);

  while (!deletePublisherWorkers) {
    if (rclcpp::ok() && streamOnFlag && imgMsgs->publisherEnabled) {
      localTimeStamp = rclcpp::Clock{RCL_ROS_TIME}.now();
      aditof::Frame * fr = safeDataAccess->getCurrentElement();

      if (fr != nullptr) {
        imgMsgs->FrameDataToMsg(camera, fr, localTimeStamp);
        imgMsgs->publishMsg(*img_publisher);
      }

    } else {
      std::stringstream ss;
      ss << std::this_thread::get_id();
      uint64_t id = std::stoull(ss.str());

      LOG(INFO) << "Thread sleep id=" << id;
      loop_rate.sleep();
    }
  }
}

void publisherPointCloudMsgsWorker(
  std::shared_ptr<AditofSensorPointCloudMsg> pointCloudMsgs,
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPublishers,
  const std::shared_ptr<aditof::Camera> & camera, aditof::Frame ** frame,
  SafeDataAccess<aditof::Frame *> * safeDataAccess)
{
  rclcpp::Time localTimeStamp = rclcpp::Clock{RCL_ROS_TIME}.now();
  rclcpp::Rate loop_rate(1.0 / 10);

  while (!deletePublisherWorkers) {
    if (rclcpp::ok() && streamOnFlag && pointCloudMsgs->publisherEnabled) {
      localTimeStamp = rclcpp::Clock{RCL_ROS_TIME}.now();
      aditof::Frame * fr = safeDataAccess->getCurrentElement();

      if (fr != nullptr) {
        pointCloudMsgs->FrameDataToMsg(camera, fr);
        pointCloudMsgs->publishMsg(*pointCloudPublishers);
      }

    } else {
      std::stringstream ss;
      ss << std::this_thread::get_id();
      uint64_t id = std::stoull(ss.str());

      LOG(INFO) << "Thread sleep id=" << id;
      loop_rate.sleep();
    }
  }
}

void publisherSingleThreadWorker(
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> imgPublishers,
  std::vector<std::shared_ptr<AditofSensorMsg>> imgMsgs,
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pointCloudPublishers,
  std::vector<std::shared_ptr<AditofSensorPointCloudMsg>> pointCloudMsgs,
  std::vector<rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> cameraStatus_publisher,
  const std::shared_ptr<aditof::Camera> & camera, aditof::Frame ** frame,
  SafeDataAccess<aditof::Frame *> * safeDataAccess)
{
  rclcpp::Time localTimeStamp = rclcpp::Clock{RCL_ROS_TIME}.now();
  rclcpp::Rate loop_rate(1.0 / 10);

  while (!deletePublisherWorkers) {
    if (streamOnFlag) {
      localTimeStamp = rclcpp::Clock{RCL_ROS_TIME}.now();
      for (unsigned int i = 0; i < imgMsgs.size(); ++i) {
        if (rclcpp::ok() && imgMsgs.at(i)->publisherEnabled) {
          aditof::Frame * fr = safeDataAccess->getCurrentElement();

          if (fr != nullptr) {
            imgMsgs.at(i)->FrameDataToMsg(camera, fr, localTimeStamp);
            imgMsgs.at(i)->publishMsg(*imgPublishers.at(i));
          }
        }
      }
      for (unsigned int i = 0; i < pointCloudMsgs.size(); ++i) {
        if (rclcpp::ok() && pointCloudMsgs.at(i)->publisherEnabled) {
          aditof::Frame * fr = safeDataAccess->getCurrentElement();

          if (fr != nullptr) {
            pointCloudMsgs.at(i)->FrameDataToMsg(camera, fr);
            pointCloudMsgs.at(i)->publishMsg(*pointCloudPublishers.at(i));
          }
        }
      }
    }
    std_msgs::msg::Int32 msg;
    msg.data = pub_factory_adsd3500_status;
    if (cameraStatus_publisher.size()) {
      cameraStatus_publisher.at(0)->publish(msg);
      if (!streamOnFlag) {
        loop_rate.sleep();
      }
    }
  }
}
