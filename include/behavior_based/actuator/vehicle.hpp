#ifndef INCLUDED_BEHAVIOR_BASED_ACTUATOR_VEHICLE_HPP
#define INCLUDED_BEHAVIOR_BASED_ACTUATOR_VEHICLE_HPP

#include <cmath>

#include <boost/math/special_functions.hpp>

#include <geometry_msgs/Twist.h>

#include <behavior_based/configure.hpp>
#include <behavior_based/geometry/angle.hpp>
#include <behavior_based/semantics/velocity.hpp>

// XXX 暫定的に古いAPIをそのまま移植しているため、ここだけAPIがクソ仕様
// セマンティクス抽出の逆変換になっているべき

namespace NAMESPACE { namespace actuator
{
  template <typename...>
  struct vehicle;

  template <typename Velocity>
  struct vehicle<Velocity>
  {
    using behavior_output_type
      = typename semantics::velocity_traits<Velocity>::output_type;

    template <typename Environment>
    decltype(auto) operator()(const behavior_output_type& steering,
                              const Environment& environment) const
    {
      const auto current {static_cast<const Velocity&>(environment)()};

      const auto desired {current + steering};

      const auto angle {geometry::angle(Eigen::Vector2d::UnitX(), desired)};

      geometry_msgs::Twist twist {};

      twist.linear.x
        = desired.norm() * std::max(std::cos(angle), 0.0);

      twist.angular.z
        = boost::math::sign(desired[1])
          * angle
          * (semantics::velocity_traits<Velocity>::angular_max
             / boost::math::constants::pi<double>());

      return twist;
    }
  };
}} // namespace NAMESPACE::actuator

#endif // INCLUDED_BEHAVIOR_BASED_ACTUATOR_VEHICLE_HPP

