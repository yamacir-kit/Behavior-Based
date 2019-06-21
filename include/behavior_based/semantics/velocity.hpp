#ifndef INCLUDED_BEHAVIOR_BASED_SEMANTICS_VELOCITY_HPP
#define INCLUDED_BEHAVIOR_BASED_SEMANTICS_VELOCITY_HPP

#include <cmath>

#include <Eigen/Core>

#include <nav_msgs/Odometry.h>

#include <behavior_based/configure.hpp>
#include <behavior_based/expression/list.hpp>
#include <behavior_based/semantics/define.hpp>
#include <behavior_based/semantics/facade.hpp>

namespace NAMESPACE { namespace semantics
{
  template <typename Velocity>
  struct velocity_traits
  {
    using output_type = typename Velocity::output_type;

    constexpr max = 10.0; // [m/s]
  };

  DEFINE_SEMANTICS_CATEGORY(velocity, output_type::Zero());

  DEFINE_SEMANTICS_CATEGORY_SPECIALIZATION(velocity, nav_msgs::Odometry,
  {
    const auto& twist {(*message).twist.twist};

    const output_type value {
      std::cos(twist.angular.z),
      std::sin(twist.angular.z)
    };

    return value * twist.linear.x;
  })
}} // namespace NAMESPACE::semantics

#endif // INCLUDED_BEHAVIOR_BASED_SEMANTICS_VELOCITY_HPP

