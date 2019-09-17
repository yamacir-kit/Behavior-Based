#ifndef INCLUDED_BEHAVIORS_SEMANTICS_CURRENT_VELOCITY_HPP
#define INCLUDED_BEHAVIORS_SEMANTICS_CURRENT_VELOCITY_HPP

#include <cmath>

#include <boost/math/constants/constants.hpp>

#include <Eigen/Core>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <behaviors/expression/list.hpp>
#include <behaviors/semantics/define.hpp>
#include <behaviors/semantics/facade.hpp>

namespace behaviors { namespace semantics
{
  DEFINE_SEMANTICS_CATEGORY(current_velocity, vector_type::Zero());

  template <typename CurrentVelocity>
  struct current_velocity_traits
  {
    using vector_type
      = typename CurrentVelocity::vector_type;

    static constexpr auto linear_max {10.0}; // [m/s]

    static constexpr auto angular_min {-boost::math::constants::pi<double>()}; // [rad/s]
    static constexpr auto angular_max {+boost::math::constants::pi<double>()}; // [rad/s]
  };

  DEFINE_SEMANTICS_CATEGORY_SPECIALIZATION(current_velocity, geometry_msgs::Twist,
  {
    return vector_type {
      std::cos(message->angular.z),
      std::sin(message->angular.z)
    } * message->linear.x;
  })

  DEFINE_SEMANTICS_CATEGORY_SPECIALIZATION(current_velocity, geometry_msgs::TwistStamped,
  {
    return vector_type {
      std::cos(message->twist.angular.z),
      std::sin(message->twist.angular.z)
    } * message->twist.linear.x;
  })

  DEFINE_SEMANTICS_CATEGORY_SPECIALIZATION(current_velocity, nav_msgs::Odometry,
  {
    return vector_type {
      std::cos(message->twist.twist.angular.z),
      std::sin(message->twist.twist.angular.z)
    } * message->twist.twist.linear.x;
  })
}} // namespace behaviors::semantics

#endif // INCLUDED_BEHAVIORS_SEMANTICS_CURRENT_VELOCITY_HPP

