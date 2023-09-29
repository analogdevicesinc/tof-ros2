#ifndef WORKER_THREAD_H
#define WORKER_THREAD_H

#include <thread>

#include "ros_task.h"

/**
 * \brief Class for running a task in a thread.
 *
 * This class declares a method that is run o a thread
 */
class WorkerThread : public std::thread
{
public:
  /**
   * \brief Constructor for WorkerThread.
   *
   * This is the default constructor for interface
   *  WorkerThread.
   *
   * @param rosTask A class that implement RosTask interface
   */
  WorkerThread(RosTask * rosTask);

  /**
   * \brief Destructor for WorkerThread.
   *
   * This is a virtual destructor for WorkerThread.
   *
   */
  ~WorkerThread();

  /**
   * @brief Method that run on a thread.
   *
   * This function will run on a thread.
   *
   */
  void runTask();

private:
  /*! This data member will run its method run on a thread */
  RosTask * m_rosTask;
};

#endif  // WORKER_THREAD_H
