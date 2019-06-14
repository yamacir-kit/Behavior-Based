#include <boost/cstdlib.hpp>

#include <ros/ros.h>

#include <behavior_based/configure.hpp>

int main(int argc, char** argv)
{
  using namespace behavior_based;

  ros::init(argc, argv, configure::project_name);

  ros::NodeHandle handle {"~"};

  ros::spin();

  return boost::exit_success;
}

