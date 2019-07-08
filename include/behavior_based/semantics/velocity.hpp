#ifndef INCLUDED_BEHAVIOR_BASED_SEMANTICS_VELOCITY_HPP
#define INCLUDED_BEHAVIOR_BASED_SEMANTICS_VELOCITY_HPP

#include <cmath>

#include <boost/math/constants/constants.hpp>

#include <Eigen/Core>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <behavior_based/configure.hpp>
#include <behavior_based/expression/list.hpp>
#include <behavior_based/semantics/define.hpp>
#include <behavior_based/semantics/facade.hpp>

namespace NAMESPACE { namespace semantics
{
  DEFINE_SEMANTICS_CATEGORY(velocity, output_type::Zero());

  template <typename Velocity>
  struct velocity_traits
  {
    using output_type = typename Velocity::output_type;

    static constexpr auto linear_max = 10.0; // [m/s]
    static constexpr auto angular_max = boost::math::constants::pi<double>(); // [rad/s]
  };

  // TODO Eigen が賢いコンストラクタを持ってるのでそれを使うように修正すること

  DEFINE_SEMANTICS_CATEGORY_SPECIALIZATION(velocity, geometry_msgs::Twist,
  {
    return output_type {
      std::cos(message->angular.z),
      std::sin(message->angular.z)
    } * message->linear.x;
  });

  DEFINE_SEMANTICS_CATEGORY_SPECIALIZATION(velocity, geometry_msgs::TwistStamped,
  {
    return output_type {
      std::cos(message->twist.angular.z),
      std::sin(message->twist.angular.z)
    } * message->twist.linear.x;
  });

  DEFINE_SEMANTICS_CATEGORY_SPECIALIZATION(velocity, nav_msgs::Odometry,
  {
    return output_type {
      std::cos(message->twist.twist.angular.z),
      std::sin(message->twist.twist.angular.z)
    } * message->twist.twist.linear.x;
  });
}} // namespace NAMESPACE::semantics

#endif // INCLUDED_BEHAVIOR_BASED_SEMANTICS_VELOCITY_HPP

