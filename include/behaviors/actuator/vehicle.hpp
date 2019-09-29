#ifndef INCLUDED_BEHAVIORS_ACTUATOR_VEHICLE_HPP
#define INCLUDED_BEHAVIORS_ACTUATOR_VEHICLE_HPP

#include <cmath>

#include <boost/math/special_functions.hpp>

#include <autoware_msgs/VehicleCmd.h>

#include <behaviors/geometry/angle.hpp>
#include <behaviors/semantics/current_velocity.hpp>
#include <behaviors/semantics/extractor.hpp>
#include <behaviors/utility/demangle.hpp>
#include <behaviors/utility/print.hpp>

namespace behaviors { namespace actuator
{
  template <typename...>
  struct vehicle;

  template <typename CurrentVelocity>
  struct vehicle<CurrentVelocity>
  {
    using vector_type
      = typename semantics::current_velocity_traits<CurrentVelocity>::vector_type;

    static constexpr auto angular_velocity_max {
      semantics::current_velocity_traits<CurrentVelocity>::angular_max
    };

    // XXX CurrentVelocity unneeded?
    template <typename Environment>
    auto operator()(const vector_type& steering, const Environment& environment) const
    {
      std::cerr << "; vehicle\t; specialized for " << utility::demangle(typeid(CurrentVelocity)) << std::endl;

      const auto current_velocity {
        semantics::extract<CurrentVelocity>().from(environment)
      };
      PRINT_VECTOR2D(current_velocity);

      const auto desired_velocity {current_velocity + steering};

      const auto angle {geometry::angle(Eigen::Vector2d::UnitX(), desired_velocity)};

      autoware_msgs::VehicleCmd command {};

      // command.gear = (desired_velocity.x() < 0 ? 64 : 1);

      command.twist_cmd.twist.linear.x
        = desired_velocity.norm(); // * std::max(std::cos(angle), 0.0);

      command.twist_cmd.twist.angular.z
        = boost::math::sign(desired_velocity[1])
          * angle
          * (angular_velocity_max / boost::math::constants::pi<double>());

      command.twist_cmd.header.stamp = ros::Time::now();

      return command;
    }
  };
}} // namespace behaviors::actuator

#endif // INCLUDED_BEHAVIORS_ACTUATOR_VEHICLE_HPP

