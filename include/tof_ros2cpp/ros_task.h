#ifndef ROS_TASK_H
#define ROS_TASK_H

/**
 * \brief Interface for running a task in a thread.
 *
 * This interface declares a method that is run o a thread
 */
class RosTask
{
public:
  /**
   * \brief Constructor for RosTask.
   *
   * This is the default constructor for interface
   *  RosTask.
   *
   */
  RosTask() {}

  /**
   * \brief Destructor for RosTask.
   *
   * This is a virtual destructor for RosTask.
   *
   */
  virtual ~RosTask() {}

  /**
   * @brief Method that run on a thread.
   *
   * This function will run on a thread and a class
   * that implement this method will have concurrent support.
   *
   */
  virtual void run() = 0;
};

#endif  // ROS_TASK_H
