#include <lcm/lcm-cpp.hpp>
#include "lcm_std_msgs/Float64.hpp"
#include "lcm_std_msgs/Float64MultiArray.hpp"

#include <iostream>
#include <math.h>
#include <unistd.h>

int main(int argc, char **argv)
{
  lcm::LCM lcm;
  if (!lcm.good())
    return 1;

  lcm_std_msgs::Float64MultiArray f64array_msg;
  f64array_msg.size = 2;
  f64array_msg.data.resize(f64array_msg.size);

  std::cout << "Running...\n";
  uint64_t count = 0;
  while (1)
  {
    for (uint32_t i = 0; i < f64array_msg.size; i++)
    {
      f64array_msg.data[i] = std::sin(count / 100.0);
    }
    lcm.publish("state/q", &f64array_msg);
    count++;
    usleep(1000);
  }

  return 0;
}