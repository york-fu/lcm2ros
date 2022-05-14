#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

#include <lcm/lcm-cpp.hpp>
#include "lcm_std_msgs/Float64.hpp"
#include "lcm_std_msgs/Float64MultiArray.hpp"

#include <sstream>
#include <signal.h>

ros::Publisher joint_q_pub;
ros::Publisher joint_v_pub;
ros::Publisher joint_vdot_pub;
ros::Publisher joint_current_pub;

ros::Publisher raw_joint_q_pub;
ros::Publisher raw_joint_v_pub;
ros::Publisher raw_joint_vdot_pub;

ros::Publisher state_q_pub;
ros::Publisher state_v_pub;
ros::Publisher state_vdot_pub;
ros::Publisher raw_state_q_pub;
ros::Publisher raw_state_v_pub;
ros::Publisher raw_state_vdot_pub;
ros::Publisher desire_q_pub;
ros::Publisher desire_v_pub;
ros::Publisher desire_vdot_pub;

ros::Publisher state_com_x_pub;
ros::Publisher state_com_u_pub;
ros::Publisher desire_com_x_pub;
ros::Publisher desire_com_u_pub;

ros::Publisher state_phase_pub;
ros::Publisher desire_phase_pub;

ros::Publisher state_L_pub;
ros::Publisher desire_L_pub;

ros::Publisher state_cost_pub;
ros::Publisher desire_tau_pub;

pthread_t thread_lcm;

class MsgHandler
{
private:
  struct timespec tv;

public:
  void f64Callback(const lcm::ReceiveBuffer *rbuf,
                   const std::string &chan,
                   const lcm_std_msgs::Float64 *msg)
  {
    std_msgs::Float64 f64Msg;
    f64Msg.data = msg->data;
    if (chan == "state/phase")
    {
      state_phase_pub.publish(f64Msg);
    }
    else if (chan == "desire/phase")
    {
      desire_phase_pub.publish(f64Msg);
    }
  }

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
    else if (chan == "raw/state/q")
    {
      raw_state_q_pub.publish(f64array_msg);
    }
    else if (chan == "raw/state/v")
    {
      raw_state_v_pub.publish(f64array_msg);
    }
    else if (chan == "raw/state/vdot")
    {
      raw_state_vdot_pub.publish(f64array_msg);
    }
    else if (chan == "joint/q")
    {
      joint_q_pub.publish(f64array_msg);
    }
    else if (chan == "joint/v")
    {
      joint_v_pub.publish(f64array_msg);
    }
    else if (chan == "joint/vdot")
    {
      joint_vdot_pub.publish(f64array_msg);
    }
    else if (chan == "joint/current")
    {
      joint_current_pub.publish(f64array_msg);
    }
    else if (chan == "raw/joint/q")
    {
      raw_joint_q_pub.publish(f64array_msg);
    }
    else if (chan == "raw/joint/v")
    {
      raw_joint_v_pub.publish(f64array_msg);
    }
    else if (chan == "raw/joint/vdot")
    {
      raw_joint_vdot_pub.publish(f64array_msg);
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
    else if (chan == "state/L")
    {
      state_L_pub.publish(f64array_msg);
    }
    else if (chan == "desire/L")
    {
      desire_L_pub.publish(f64array_msg);
    }
    else if (chan == "state/cost")
    {
      state_cost_pub.publish(f64array_msg);
    }
    else if (chan == "desire/tau")
    {
      desire_tau_pub.publish(f64array_msg);
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

  lcm.subscribe("joint/q", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("joint/v", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("joint/vdot", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("joint/current", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("raw/joint/q", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("raw/joint/v", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("raw/joint/vdot", &MsgHandler::f64arrayCallback, &msgHandler);

  lcm.subscribe("state/q", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("state/v", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("state/vdot", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("raw/state/q", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("raw/state/v", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("raw/state/vdot", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("desire/q", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("desire/v", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("desire/vdot", &MsgHandler::f64arrayCallback, &msgHandler);

  lcm.subscribe("state/com/x", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("state/com/u", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("desire/com/x", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("desire/com/u", &MsgHandler::f64arrayCallback, &msgHandler);

  lcm.subscribe("state/phase", &MsgHandler::f64Callback, &msgHandler);
  lcm.subscribe("desire/phase", &MsgHandler::f64Callback, &msgHandler);

  lcm.subscribe("state/L", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("desire/L", &MsgHandler::f64arrayCallback, &msgHandler);

  lcm.subscribe("state/cost", &MsgHandler::f64arrayCallback, &msgHandler);
  lcm.subscribe("desire/tau", &MsgHandler::f64arrayCallback, &msgHandler);

  return true;
}

void topicInitial(ros::NodeHandle &nh)
{
  joint_q_pub = nh.advertise<std_msgs::Float64MultiArray>("joint/q", 1000);
  joint_v_pub = nh.advertise<std_msgs::Float64MultiArray>("joint/v", 1000);
  joint_vdot_pub = nh.advertise<std_msgs::Float64MultiArray>("joint/vdot", 1000);
  joint_current_pub = nh.advertise<std_msgs::Float64MultiArray>("joint/current", 1000);
  raw_joint_q_pub = nh.advertise<std_msgs::Float64MultiArray>("raw/joint/q", 1000);
  raw_joint_v_pub = nh.advertise<std_msgs::Float64MultiArray>("raw/joint/v", 1000);
  raw_joint_vdot_pub = nh.advertise<std_msgs::Float64MultiArray>("raw/joint/vdot", 1000);

  state_q_pub = nh.advertise<std_msgs::Float64MultiArray>("state/q", 1000);
  state_v_pub = nh.advertise<std_msgs::Float64MultiArray>("state/v", 1000);
  state_vdot_pub = nh.advertise<std_msgs::Float64MultiArray>("state/vdot", 1000);
  raw_state_q_pub = nh.advertise<std_msgs::Float64MultiArray>("raw/state/q", 1000);
  raw_state_v_pub = nh.advertise<std_msgs::Float64MultiArray>("raw/state/v", 1000);
  raw_state_vdot_pub = nh.advertise<std_msgs::Float64MultiArray>("raw/state/vdot", 1000);
  desire_q_pub = nh.advertise<std_msgs::Float64MultiArray>("desire/q", 1000);
  desire_v_pub = nh.advertise<std_msgs::Float64MultiArray>("desire/v", 1000);
  desire_vdot_pub = nh.advertise<std_msgs::Float64MultiArray>("desire/vdot", 1000);

  state_com_x_pub = nh.advertise<std_msgs::Float64MultiArray>("state/com/x", 1000);
  state_com_u_pub = nh.advertise<std_msgs::Float64MultiArray>("state/com/u", 1000);
  desire_com_x_pub = nh.advertise<std_msgs::Float64MultiArray>("desire/com/x", 1000);
  desire_com_u_pub = nh.advertise<std_msgs::Float64MultiArray>("desire/com/u", 1000);

  state_phase_pub = nh.advertise<std_msgs::Float64>("state/phase", 1000);
  desire_phase_pub = nh.advertise<std_msgs::Float64>("desire/phase", 1000);

  state_L_pub = nh.advertise<std_msgs::Float64MultiArray>("state/L", 1000);
  desire_L_pub = nh.advertise<std_msgs::Float64MultiArray>("desire/L", 1000);

  state_cost_pub = nh.advertise<std_msgs::Float64MultiArray>("state/cost", 1000);
  desire_tau_pub = nh.advertise<std_msgs::Float64MultiArray>("desire/tau", 1000);
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