#include <boost/cstdlib.hpp>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <lgsvl_msgs/Detection3DArray.h>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_extension/projection/mgrs_projector.h>
#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

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

#define DEFINE_SLOT(TYPE, NAME)                                                \
struct NAME                                                                    \
  : public TYPE::ConstPtr                                                      \
{                                                                              \
  using message_type = TYPE;                                                   \
                                                                               \
  template <typename... Ts>                                                    \
  explicit NAME(Ts&&... operands)                                              \
    : TYPE::ConstPtr {std::forward<decltype(operands)>(operands)...}           \
  {}                                                                           \
                                                                               \
  NAME& operator=(const TYPE::ConstPtr& rhs) &                                 \
  {                                                                            \
    static_cast<TYPE::ConstPtr&>(*this) = rhs;                                 \
    return *this;                                                              \
  }                                                                            \
}

DEFINE_SLOT(geometry_msgs::PoseStamped, goal_slot);
DEFINE_SLOT(geometry_msgs::PoseStamped, predict_slot);
DEFINE_SLOT(nav_msgs::Odometry, odometry_slot);
DEFINE_SLOT(sensor_msgs::Joy, joy_slot);

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

  lanelet::GPSPoint point {49.0, 8.4};
  lanelet::Origin origin {point};
  lanelet::ErrorMessages error {};
  lanelet::projection::UtmProjector projector {origin, true, false};

  auto map {lanelet::load("/home/yamasa/Downloads/lanelet2.osm", "osm_handler", projector, &error)};

  /**
   * The environment is set of sensor data.
   *
   * This describes set of latest sensor data which the robot can seen (is
   * almost current, allowed time delay is depend on your system).
   */
  using environment
    = expression::list<
        goal_slot,
        joy_slot,
        odometry_slot,
        predict_slot
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

  // using avoidance
  //   = behavior::flee<
  //       semantics::current_velocity<nav_msgs::Odometry>,
  //       semantics::target<lgsvl_msgs::Detection3DArray>
  //     >;

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
  expression::list<
    slave
  // , avoidance
  > behaviors {};

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
      std::cerr << "\n"
                << "; publisher\t; autoware_msgs::VehicleCmd" << std::endl;
      static auto publisher {handle.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 1)};
      PRINT_VALUE(data.twist_cmd.header.stamp);
      PRINT_VALUE(data.twist_cmd.twist.linear.x);
      PRINT_VALUE(data.twist_cmd.twist.angular.z);
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
    std::cerr << "\n"
              << "; " << std::string(78, '-') << "\n"
              << ";   Prioritized Acceleration Allocation" << "\n"
              << "; " << std::string(78, '-') << std::endl;

    auto allocate = [&](const auto& a, const auto& b) mutable
      -> Eigen::Vector2d
    {
      auto importance_of = [&](const auto& v)
      {
        return geometry::angle(Eigen::Vector2d::UnitX(), v) / boost::math::constants::pi<double>();
      };

      return a + (1.0 - importance_of(a)) * b(current_environment);
    };

    return expression::fold_left(behaviors, Eigen::Vector2d::Zero(), allocate);
  };

  #define CALLBACK(TYPENAME, ...)                                              \
  std::function<void (const TYPENAME::ConstPtr&)>                              \
  {                                                                            \
    [&](auto&& message) mutable __VA_ARGS__                                    \
  }

  #define CONNECT(SLOTNAME, TOPICNAME)                                         \
  handle.subscribe<SLOTNAME::message_type>(TOPICNAME, 1,                       \
    CALLBACK(SLOTNAME::message_type,                                           \
    {                                                                          \
      static_cast<SLOTNAME&>(current_environment) = message;                   \
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
    CONNECT(goal_slot, "/move_base_simple/goal"),
    CONNECT(joy_slot, "/joy"),
    CONNECT(odometry_slot, "/odom"),
    CONNECT(predict_slot, "/predict_pose"),
  };

  ros::spin();

  return boost::exit_success;
}

