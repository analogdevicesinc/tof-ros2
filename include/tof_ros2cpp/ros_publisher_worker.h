#ifndef ROS_PUBLISHER_WORKER_H
#define ROS_PUBLISHER_WORKER_H

#include "ros_task.h"
#include "publisher_factory.h"
#include "aditof/camera.h"

class RosPublisherWorker : public RosTask
{
public:
  RosPublisherWorker(PublisherFactory* publisherFactory,
                     int publisherIndex);
  ~RosPublisherWorker();

  bool isRunCompleted();
  bool setIsRunCompleted(bool val);

  void setCameraData(
     std::shared_ptr<aditof::Camera> * camera, aditof::Frame ** frame,
    rclcpp ::Time timestamp);

  void run() override;

private:
  PublisherFactory* m_publisherFactory;
  int m_publisherIndex;
  bool m_isRunCompleted;

  std::shared_ptr<aditof::Camera> * m_camera;
  aditof::Frame ** m_frame;
  rclcpp ::Time m_timestamp;
};


#endif // ROS_PUBLISHER_WORKER_H
