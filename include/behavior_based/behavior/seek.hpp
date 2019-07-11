#ifndef INCLUDED_BEHAVIOR_BASED_BEHAVIOR_SEEK_HPP
#define INCLUDED_BEHAVIOR_BASED_BEHAVIOR_SEEK_HPP

#include <behavior_based/configure.hpp>
#include <behavior_based/semantics/extractor.hpp>
#include <behavior_based/semantics/current_velocity.hpp>

namespace NAMESPACE { namespace behavior
{
  template <typename CurrentVelocity, typename Target>
  struct seek
  {
    constexpr seek() = default;

    template <typename Environment>
    auto operator()(const Environment& environment) const
      -> typename semantics::current_velocity_traits<CurrentVelocity>::output_type
    {
      using namespace semantics;

      const auto current_velocity {
        extract<CurrentVelocity>().from(environment)
      };

      const auto target {extract<Target>().from(environment).normalized()};

      const auto desired_velocity {
        target * current_velocity_traits<CurrentVelocity>::linear_max
      };

      return desired_velocity - current_velocity; // steering
    }
  };
}} // namespace NAMESPACE::behavior

#endif // INCLUDED_BEHAVIOR_BASED_BEHAVIOR_SEEK_HPP

