#include <boost/cstdlib.hpp>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include <behavior_based/configure.hpp>
#include <behavior_based/expression/dispatch.hpp>

int main(int argc, char** argv)
{
  using namespace behavior_based;

  ros::init(argc, argv, configure::project_name);

  ros::NodeHandle handle {"~"};

  auto publish {expression::dispatch(
    [&](const geometry_msgs::TwistStamped& data)
    {
      static auto p {handle.advertise<geometry_msgs::TwistStamped>("/twist_raw", 1)};
      return p.publish(data);
    }
  )};

  // ros::spin();

  for (ros::Rate rate {10}; ros::ok(); rate.sleep())
  {
    geometry_msgs::TwistStamped twist {};
    twist.header.stamp = ros::Time::now();
    twist.twist.linear.x = 1.0;

    publish(twist);
  }

  return boost::exit_success;
}

