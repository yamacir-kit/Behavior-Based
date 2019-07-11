#ifndef INCLUDED_BEHAVIOR_BASED_BEHAVIOR_SEEK_HPP
#define INCLUDED_BEHAVIOR_BASED_BEHAVIOR_SEEK_HPP

#include <behavior_based/configure.hpp>
#include <behavior_based/semantics/extractor.hpp>
#include <behavior_based/semantics/current_velocity.hpp>

namespace NAMESPACE { namespace behavior
{
  #define CURRENT_VELOCITY semantics::extract<CurrentVelocity>().from(environment)
  #define TARGET semantics::extract<Target>().from(environment)
  #define VELOCITY_MAX semantics::current_velocity_traits<CurrentVelocity>::linear_max

  template <typename CurrentVelocity, typename Target>
  struct seek
  {
    using vector_type
      = typename semantics::current_velocity_traits<CurrentVelocity>::vector_type;

    template <typename Environment>
    vector_type operator()(const Environment& environment) const
    {
      return TARGET.normalized() * VELOCITY_MAX - CURRENT_VELOCITY;
    }
  };

  #undef CURRENT_VELOCITY
  #undef TARGET
  #undef VELOCITY_MAX
}} // namespace NAMESPACE::behavior

#endif // INCLUDED_BEHAVIOR_BASED_BEHAVIOR_SEEK_HPP

