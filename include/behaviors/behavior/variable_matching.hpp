#ifndef INCLUDED_BEHAVIORS_BEHAVIOR_SEEK_HPP
#define INCLUDED_BEHAVIORS_BEHAVIOR_SEEK_HPP

#include <behaviors/semantics/extractor.hpp>
#include <behaviors/semantics/current_velocity.hpp>

namespace behaviors { namespace behavior
{
  #define CURRENT semantics::extract<Current>().from(environment)
  #define TARGET semantics::extract<Target>().from(environment)

  #define VARIABLE_MAX semantics::current_velocity_traits<Current>::linear_max

  template <typename Current, typename Target>
  struct seek
  {
    using vector_type
      = typename semantics::current_velocity_traits<Current>::vector_type;

    template <typename Environment>
    vector_type operator()(const Environment& environment) const
    {
      return TARGET.normalized() * VARIABLE_MAX - CURRENT;
    }
  };

  template <typename Current, typename Target>
  struct flee
  {
    using vector_type
      = typename semantics::current_velocity_traits<Current>::vector_type;

    template <typename Environment>
    vector_type operator()(const Environment& environment) const
    {
      return  CURRENT - TARGET.normalized() * VARIABLE_MAX;
    }
  };

  #undef CURRENT
  #undef TARGET
  #undef VARIABLE_MAX
}} // namespace behaviors::behavior

#endif // INCLUDED_BEHAVIORS_BEHAVIOR_SEEK_HPP

