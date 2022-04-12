#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

#include <lcm/lcm-cpp.hpp>
#include "lcm_std_msgs/Float64.hpp"
#include "lcm_std_msgs/Float64MultiArray.hpp"

#include <sstream>
#include <signal.h>

ros::Publisher state_q_pub;
ros::Publisher state_v_pub;
ros::Publisher state_vdot_pub;
ros::Publisher desire_q_pub;
ros::Publisher desire_v_pub;
ros::Publisher desire_vdot_pub;

ros::Publisher desire_tau_pub;

ros::Publisher state_com_x_pub;
ros::Publisher state_com_u_pub;
ros::Publisher desire_com_x_pub;
ros::Publisher desire_com_u_pub;

pthread_t thread_lcm;

class MsgHandler
{
private:
  struct timespec tv;

public:
  void f64arrayCallback(const lcm::ReceiveBuffer *rbuf,
                        const std::string &chan,
                        const lcm_std_msgs::Float64MultiArray *msg)
  {
    std_msgs::Float64MultiArray f64array_msg;
    f64array_msg.data.resize(msg->size);
    f64array_msg.data.assign(msg->data.begin(), msg->data.end());

    if (chan == "state/q")
    {
      state_q_pub.publish(f64array_msg);
    }
    else if (chan == "state/v")
    {
      state_v_pub.publish(f64array_msg);
    }
    else if (chan == "state/vdot")
    {
      state_vdot_pub.publish(f64array_msg);
    }
    else if (chan == "desire/q")
    {
      desire_q_pub.publish(f64array_msg);
    }
    else if (chan == "desire/v")
    {
      desire_v_pub.publish(f64array_msg);
    }
    else if (chan == "desire/vdot")
    {
      desire_vdot_pub.publish(f64array_msg);
    }
    else if (chan == "desire/tau")
    {
      desire_tau_pub.publish(f64array_msg);
    }
    else if (chan == "state/com/x")
    {
      state_com_x_pub.publish(f64array_msg);
    }
    else if (chan == "state/com/u")
    {
      state_com_u_pub.publish(f64array_msg);
    }
    else if (chan == "desire/com/x")
    {
      desire_com_x_pub.publish(f64array_msg);
    }
    else if (chan == "desire/com/u")
    {
      desire_com_u_pub.publish(f64array_msg);
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
  lcm.subscribe("state/q", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("state/v", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("state/vdot", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("desire/q", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("desire/v", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("desire/vdot", &MsgHandler::f64arrayCallback, &msgHandler);

  lcm.subscribe("desire/tau", &MsgHandler::f64arrayCallback, &msgHandler);

  lcm.subscribe("state/com/x", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("state/com/u", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("desire/com/x", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("desire/com/u", &MsgHandler::f64arrayCallback, &msgHandler);
  return true;
}

void topicInitial(ros::NodeHandle &nh)
{
  state_q_pub = nh.advertise<std_msgs::Float64MultiArray>("state/q", 1000);
  state_v_pub = nh.advertise<std_msgs::Float64MultiArray>("state/v", 1000);
  state_vdot_pub = nh.advertise<std_msgs::Float64MultiArray>("state/vdot", 1000);
  desire_q_pub = nh.advertise<std_msgs::Float64MultiArray>("desire/q", 1000);
  desire_v_pub = nh.advertise<std_msgs::Float64MultiArray>("desire/v", 1000);
  desire_vdot_pub = nh.advertise<std_msgs::Float64MultiArray>("desire/vdot", 1000);

  desire_tau_pub = nh.advertise<std_msgs::Float64MultiArray>("desire/tau", 1000);

  state_com_x_pub = nh.advertise<std_msgs::Float64MultiArray>("state/com/x", 1000);
  state_com_u_pub = nh.advertise<std_msgs::Float64MultiArray>("state/com/u", 1000);
  desire_com_x_pub = nh.advertise<std_msgs::Float64MultiArray>("desire/com/x", 1000);
  desire_com_u_pub = nh.advertise<std_msgs::Float64MultiArray>("desire/com/u", 1000);
}

void sigintHandler(int sig)
{
  printf("\n");
  ros::shutdown();
  printf("signal exit.\n");
  exit(EXIT_SUCCESS);
}

int main(int argc, char **argv)
{
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