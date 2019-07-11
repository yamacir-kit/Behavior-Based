#include <boost/cstdlib.hpp>

#include <ros/ros.h>

// #include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <lgsvl_msgs/Detection3DArray.h>

#include <behavior_based/actuator/vehicle.hpp>

#include <behavior_based/behavior/variable_matching.hpp>

#include <behavior_based/configure.hpp>

#include <behavior_based/expression/dispatch.hpp>
#include <behavior_based/expression/fold.hpp>
#include <behavior_based/expression/list.hpp>

#include <behavior_based/geometry/angle.hpp>

#include <behavior_based/semantics/current_velocity.hpp>
#include <behavior_based/semantics/forward.hpp>
#include <behavior_based/semantics/target.hpp>

#include <behavior_based/utility/demangle.hpp>

/**
 * FAQ
 *
 * Q. Why use functor to implement behavior instead of simple function?
 * A. Because C++ not allows partial specialization of function.
 */

int main(int argc, char** argv)
{
  using namespace behavior_based;

  ros::init(argc, argv, configure::project_name);

  ros::NodeHandle handle {"~"};

  /**
   * The environment is set of sensor data.
   *
   * This describes set of latest sensor data which the robot can seen (is
   * almost current, allowed time delay is depend on your system).
   */
  using environment
    = expression::list
      <
        lgsvl_msgs::Detection3DArray::ConstPtr,
        nav_msgs::Odometry::ConstPtr,
        sensor_msgs::Joy::ConstPtr
      >;

  environment current_environment {};

  using slave
    = behavior::seek
      <
        semantics::current_velocity<nav_msgs::Odometry>,
        semantics::target<sensor_msgs::Joy>
      >;

  using forward
    = behavior::seek
      <
        semantics::current_velocity<nav_msgs::Odometry>,
        semantics::forward<Eigen::Vector2d>
      >;

  using avoidance
    = behavior::flee
      <
        semantics::current_velocity<nav_msgs::Odometry>,
        semantics::target<lgsvl_msgs::Detection3DArray>
      >;

  /**
   * The linear list of behaviors.
   *
   * Each behavior knows how to extract information which it interested in from
   * current environment. Each output (independent from each other) are folded
   * into one output by higher order function `expression::fold`, and then
   * publish (publisher dispatched by output type).
   */
  // slave behaviors {};
  constexpr expression::list<slave, avoidance> behaviors {};

  /**
   * Message publisher dispatcher.
   *
   * A folded output of each behaviors are dispatched by its type and published.
   * The dispatcher element takes a ROS message type and is responsible for
   * publishing the message. If your system does not rely on ROS, you can
   * transfer data to drivers or actuators in a framework-dependent way.
   *
   * This dispatcher is provided for cases where this library is used to
   * describe MIMO systems. For the MISO system, it is sufficient to simply give
   * the dispatcher only one element (or use simple function instead of
   * dispatcher).
   */
  auto publish {expression::dispatch(
    // [&](const geometry_msgs::Twist& data)
    // {
    //   static auto p {handle.advertise<geometry_msgs::TwistStamped>("/twist_raw", 1)};
    //   geometry_msgs::TwistStamped twist {};
    //   twist.header.stamp = ros::Time::now();
    //   twist.twist = data;
    //   return p.publish(twist);
    // },

    [&](const autoware_msgs::VehicleCmd& data)
    {
      static auto publisher {handle.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 1)};
      auto command {data};
      command.twist_cmd.header.stamp = ros::Time::now();
      return publisher.publish(command);
    }
  )};

  actuator::vehicle<
    semantics::current_velocity<nav_msgs::Odometry>
  > actuate {};

  auto prioritized_acceleration_allocation = [&]()
  {
    auto allocate = [&](const auto& a, const auto& b)
    {
      auto strategic_importance = [&](const auto& v)
      {
        return geometry::angle(Eigen::Vector2d::UnitX(), v) / boost::math::constants::pi<double>();
      };

      return a + (1.0 - strategic_importance(a)) * b(current_environment);
    };

    return expression::fold_left(behaviors, Eigen::Vector2d::Zero(), allocate);
  };

  #define CALLBACK(TYPENAME, ...)                                              \
  std::function<void (const TYPENAME::ConstPtr&)>                              \
  {                                                                            \
    [&](auto&& message) __VA_ARGS__                                            \
  }

  #define CONNECT(TYPENAME, TOPICNAME)                                         \
  handle.subscribe<TYPENAME>(TOPICNAME, 1,                                     \
    CALLBACK(TYPENAME,                                                         \
    {                                                                          \
      static_cast<TYPENAME::ConstPtr&>(current_environment) = message;         \
                                                                               \
      return publish(                                                          \
               actuate(                                                        \
                 prioritized_acceleration_allocation(),                        \
                 current_environment                                           \
               )                                                               \
             );                                                                \
    })                                                                         \
  )

  std::vector<ros::Subscriber> sensory {
    CONNECT(nav_msgs::Odometry,           "/odom")
  , CONNECT(lgsvl_msgs::Detection3DArray, "/simulator/ground_truth/3d_detections")
  , CONNECT(sensor_msgs::Joy,             "/joy")
  };

  ros::spin();

  return boost::exit_success;
}

