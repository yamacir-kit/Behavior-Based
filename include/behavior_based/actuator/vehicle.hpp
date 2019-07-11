#ifndef INCLUDED_BEHAVIOR_BASED_ACTUATOR_VEHICLE_HPP
#define INCLUDED_BEHAVIOR_BASED_ACTUATOR_VEHICLE_HPP

#include <cmath>

#include <boost/math/special_functions.hpp>

// #include <geometry_msgs/Twist.h>
#include <autoware_msgs/VehicleCmd.h>

#include <behavior_based/configure.hpp>
#include <behavior_based/geometry/angle.hpp>
#include <behavior_based/semantics/extractor.hpp>
#include <behavior_based/semantics/current_velocity.hpp>

namespace NAMESPACE { namespace actuator
{
  template <typename...>
  struct vehicle;

  template <typename CurrentVelocity>
  struct vehicle<CurrentVelocity>
  {
    using output_type
      = typename semantics::current_velocity_traits<CurrentVelocity>::output_type;

    // XXX CurrentVelocity unneeded?
    template <typename Environment>
    auto operator()(const output_type& steering,
                    const Environment& environment) const
    {
      static constexpr auto angular_velocity_max {
        semantics::current_velocity_traits<CurrentVelocity>::angular_max
      };

      const auto current_velocity {
        semantics::extract<CurrentVelocity>().from(environment)
      };

      const auto desired_velocity {current_velocity + steering};

      const auto angle {geometry::angle(Eigen::Vector2d::UnitX(), desired_velocity)};

      // geometry_msgs::Twist twist {};
      autoware_msgs::VehicleCmd command {};

      command.twist_cmd.twist.linear.x
        = desired_velocity.norm() * std::max(std::cos(angle), 0.0);

      command.twist_cmd.twist.angular.z
        = boost::math::sign(desired_velocity[1])
          * angle
          * (angular_velocity_max / boost::math::constants::pi<double>());

      return command;
    }
  };
}} // namespace NAMESPACE::actuator

#endif // INCLUDED_BEHAVIOR_BASED_ACTUATOR_VEHICLE_HPP

