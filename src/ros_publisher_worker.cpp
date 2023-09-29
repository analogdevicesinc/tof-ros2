#include "ros_publisher_worker.h"

RosPublisherWorker::RosPublisherWorker(PublisherFactory* publisherFactory, int publisherIndex) :
  m_publisherFactory(publisherFactory),
  m_publisherIndex(publisherIndex),
  m_isRunCompleted(true)
{

}

RosPublisherWorker::~RosPublisherWorker()
{

}

bool RosPublisherWorker::isRunCompleted()
{
  return m_isRunCompleted;
}

bool RosPublisherWorker::setIsRunCompleted(bool val)
{
  if(val != m_isRunCompleted)
  {
    m_isRunCompleted = val;
  }
}

void RosPublisherWorker::setCameraData(
   std::shared_ptr<aditof::Camera> * camera, aditof::Frame ** frame,
  rclcpp ::Time timestamp)
{
  m_camera = camera;
  m_frame = frame;
  m_timestamp = timestamp;
  m_isRunCompleted = false;
}

void RosPublisherWorker::run()
{
  while(rclcpp::ok())
  {
    if(m_isRunCompleted == false)
    {
      m_publisherFactory->updateOnePublisher(
            *m_camera, m_frame, m_timestamp, m_publisherIndex);
      m_isRunCompleted = true;
    }
  }
}
