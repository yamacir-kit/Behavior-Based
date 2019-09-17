#ifndef INCLUDED_BEHAVIORS_BEHAVIOR_SEEK_HPP
#define INCLUDED_BEHAVIORS_BEHAVIOR_SEEK_HPP

#include <behaviors/semantics/extractor.hpp>
#include <behaviors/semantics/current_velocity.hpp>

namespace behaviors { namespace behavior
{
  #define CURRENT_VELOCITY semantics::extract< CurrentVelocity >().from(environment)
  #define TARGET           semantics::extract< Target          >().from(environment)

  #define VELOCITY_MAX     semantics::current_velocity_traits<CurrentVelocity>::linear_max

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

  template <typename CurrentVelocity, typename Target>
  struct flee
  {
    using vector_type
      = typename semantics::current_velocity_traits<CurrentVelocity>::vector_type;

    template <typename Environment>
    vector_type operator()(const Environment& environment) const
    {
      return  CURRENT_VELOCITY - TARGET.normalized() * VELOCITY_MAX;
    }
  };

  #undef CURRENT_VELOCITY
  #undef TARGET
  #undef VELOCITY_MAX
}} // namespace behaviors::behavior

#endif // INCLUDED_BEHAVIORS_BEHAVIOR_SEEK_HPP

