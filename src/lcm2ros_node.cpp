#include <sstream>
#include <signal.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

#include <lcm/lcm-cpp.hpp>
#include "lcm_std_msgs/Float64.hpp"
#include "lcm_std_msgs/Float64MultiArray.hpp"

static std::vector<std::string> f64_channel_name = {
    "state/phase",
    "desire/phase",
};
static std::vector<std::string> f64_array_channel_name = {
    "joint/q",
    "joint/v",
    "joint/vdot",
    "joint/current",
    "raw/joint/q",
    "raw/joint/v",
    "raw/joint/vdot",
    "raw/joint/current",

    "raw/state/q",
    "raw/state/v",
    "raw/state/vdot",
    "state/q",
    "state/v",
    "state/vdot",
    "state/lfoot",
    "state/lfootv",
    "state/rfoot",
    "state/rfootv",
    "state/com/x",
    "state/com/u",
    "state/tau",
    "state/L",

    "desire/q",
    "desire/v",
    "desire/vdot",
    "desire/lfoot",
    "desire/lfootv",
    "desire/rfoot",
    "desire/rfootv",
    "desire/com/x",
    "desire/com/u",
    "desire/tau",
    "desire/L",

    "command/q",
    "command/v",
    "command/vdot",
    "command/lfoot",
    "command/lfootv",
    "command/rfoot",
    "command/rfootv",
    "command/com/x",
    "command/com/u",
    "command/tau",

    "wbc/cost",
    "wbc/tau",
    "wbc/lambda",

    "control/data"
};

static std::vector<ros::Publisher> f64_pub_vec;
static std::vector<ros::Publisher> f64_array_pub_vec;

static pthread_t thread_lcm;

class MsgHandler
{
private:
  struct timespec tv;

public:
  void f64Callback(const lcm::ReceiveBuffer *rbuf,
                   const std::string &chan,
                   const lcm_std_msgs::Float64 *msg)
  {
    std_msgs::Float64 f64_msg;
    f64_msg.data = msg->data;
    for (uint32_t i = 0; i < f64_channel_name.size(); i++)
    {
      if (chan == f64_channel_name[i])
      {
        f64_pub_vec[i].publish(f64_msg);
        break;
      }
    }
  }

  void f64arrayCallback(const lcm::ReceiveBuffer *rbuf,
                        const std::string &chan,
                        const lcm_std_msgs::Float64MultiArray *msg)
  {
    std_msgs::Float64MultiArray f64array_msg;
    f64array_msg.data.resize(msg->size);
    f64array_msg.data.assign(msg->data.begin(), msg->data.end());

    for (uint32_t i = 0; i < f64_array_channel_name.size(); i++)
    {
      if (chan == f64_array_channel_name[i])
      {
        f64_array_pub_vec[i].publish(f64array_msg);
        break;
      }
    }
  }
};

void *lcm_thread(void *arg)
{
  lcm::LCM *lcmPtr = (lcm::LCM *)arg;
  while (ros::ok() && (0 == lcmPtr->handle()))
  {
    usleep(1);
  }
  return 0;
}

bool lcmInitial(lcm::LCM &lcm)
{
  if (!lcm.good())
  {
    std::cout << "Faild to lcm initial.\n";
    return false;
  }
  int ret = 0;
  ret = pthread_create(&thread_lcm, NULL, lcm_thread, (void *)&lcm);
  if (ret != 0)
  {
    printf("Create lcm_thread failed! ret %s\n", strerror(ret));
    return false;
  }

  MsgHandler msgHandler;

  for (uint32_t i = 0; i < f64_channel_name.size(); i++)
  {
    lcm.subscribe(f64_channel_name[i], &MsgHandler::f64Callback, &msgHandler);
  }
  for (uint32_t i = 0; i < f64_array_channel_name.size(); i++)
  {
    lcm.subscribe(f64_array_channel_name[i], &MsgHandler::f64arrayCallback, &msgHandler);
  }

  return true;
}

void topicInitial(ros::NodeHandle &nh)
{
  for (uint32_t i = 0; i < f64_channel_name.size(); i++)
  {
    f64_pub_vec.push_back(nh.advertise<std_msgs::Float64>(f64_channel_name[i], 1000));
  }
  for (uint32_t i = 0; i < f64_array_channel_name.size(); i++)
  {
    f64_array_pub_vec.push_back(nh.advertise<std_msgs::Float64MultiArray>(f64_array_channel_name[i], 1000));
  }
}

void sigintHandler(int sig)
{
  printf("\n");
  ros::shutdown();
  printf("signal exit.\n");
  exit(EXIT_SUCCESS);
}

int32_t process_rt()
{
  int32_t ret;
  pid_t pid = getpid();
  struct sched_param param;
  param.sched_priority = sched_get_priority_max(SCHED_FIFO); // SCHED_RR
  ret = sched_setscheduler(pid, SCHED_FIFO, &param);
  if (ret != 0)
  {
    printf("Failed to set rt of process %d. %s\n", pid, strerror(ret));
  }
  return ret;
}

int main(int argc, char **argv)
{
  process_rt();
  ros::init(argc, argv, "lcm2ros");
  ros::NodeHandle nh;
  signal(SIGINT, sigintHandler);

  topicInitial(nh);
  ros::AsyncSpinner spinner(8);
  spinner.start();

  lcm::LCM lcm;
  if (!lcmInitial(lcm))
  {
    return -1;
  }

  std::cout << "Running...\n";
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    loop_rate.sleep();
  }
  pthread_join(thread_lcm, NULL);

  return 0;
}