#include <boost/cstdlib.hpp>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <lgsvl_msgs/Detection3DArray.h>

#include <behaviors/actuator/vehicle.hpp>

#include <behaviors/behavior/variable_matching.hpp>

#include <behaviors/configure.hpp>

#include <behaviors/expression/dispatch.hpp>
#include <behaviors/expression/fold.hpp>
#include <behaviors/expression/list.hpp>

#include <behaviors/geometry/angle.hpp>

#include <behaviors/semantics/current_velocity.hpp>
#include <behaviors/semantics/forward.hpp>
#include <behaviors/semantics/target.hpp>

/**
 * FAQ
 *
 * Q. Why use functor to implement behavior instead of simple function?
 * A. Because C++ not allows partial specialization of function.
 */

int main(int argc, char** argv)
{
  using namespace behaviors;

  const std::string program_name {"sample"};
  {
    using namespace configure;

    std::cerr << "; " << project_name << " " << program_name << " - Version " << version.major << " Revision " << version.minor << " Patch " << version.patch << "\n"
              << ";\n"
              << "; configuration ; " << build_type << "\n"
              << "; source-hash\t; " << build_hash << "\n"
              << "; compiled-at\t; " << build_time << "\n"
              << std::endl;
  } // using namespace configure

  ros::init(argc, argv, program_name);

  ros::NodeHandle handle {"~"};

  /**
   * The environment is set of sensor data.
   *
   * This describes set of latest sensor data which the robot can seen (is
   * almost current, allowed time delay is depend on your system).
   */
  using environment
    = expression::list<
        lgsvl_msgs::Detection3DArray::ConstPtr,
        nav_msgs::Odometry::ConstPtr,
        sensor_msgs::Joy::ConstPtr
      >;

  environment current_environment {};

  using slave
    = behavior::seek<
        semantics::current_velocity<nav_msgs::Odometry>,
        semantics::target<sensor_msgs::Joy>
      >;

  // using forward
  //   = behavior::seek<
  //       semantics::current_velocity<nav_msgs::Odometry>,
  //       semantics::forward<Eigen::Vector2d>
  //     >;

  using avoidance
    = behavior::flee<
        semantics::current_velocity<nav_msgs::Odometry>,
        semantics::target<lgsvl_msgs::Detection3DArray>
      >;

  // using nop
  //   = behavior::seek<
  //       semantics::current_velocity<nav_msgs::Odometry>,
  //       semantics::target<expression::unit>
  //     >;

  /**
   * The linear list of behaviors.
   *
   * Each behavior knows how to extract information which it interested in from
   * current environment. Each output (independent from each other) are folded
   * into one output by higher order function `expression::fold`, and then
   * publish (publisher dispatched by output type).
   */
  expression::list<slave, avoidance> behaviors {};

  /**
   * Message publisher dispatcher.
   *
   * A folded output of each behaviors are dispatched by its type and published.
   * The dispatcher element takes a ROS message type and is responsible for
   * publishing the message. If your system does not rely on ROS, you can send
   * data to drivers or actuators in a framework-dependent way.
   *
   * This dispatcher is provided for cases where this library is used to
   * describe MIMO systems. For the MISO system, it is sufficient to simply give
   * the dispatcher only one element (or use simple publish function instead of
   * dispatcher).
   */
  auto publish {expression::dispatch(
    [&](const autoware_msgs::VehicleCmd& data)
    {
      static auto publisher {handle.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 1)};
      return publisher.publish(data);
    }
  )};

  actuator::vehicle<
    semantics::current_velocity<nav_msgs::Odometry>
  > actuate {};

  /**
   * Prioritized Acceleration Allocation is well-known behavior outputs combine
   * method.
   *
   * This library describes behaviors as list of functors. Thus, calculating
   * each behavior's output and combining can be express as fold (term in
   * functional programming).
   */
  auto prioritized_acceleration_allocation = [&]()
  {
    auto allocate = [&](const auto& a, const auto& b) -> Eigen::Vector2d
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

