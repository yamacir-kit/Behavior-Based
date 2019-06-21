#ifndef INCLUDED_BEHAVIOR_BASED_BEHAVIOR_SEEK_HPP
#define INCLUDED_BEHAVIOR_BASED_BEHAVIOR_SEEK_HPP

#include <behavior_based/configure.hpp>
#include <behavior_based/semantics/extractor.hpp>
#include <behavior_based/semantics/velocity.hpp>

namespace NAMESPACE { namespace behavior
{
  template <typename Velocity, typename Target>
  struct seek
  {
    constexpr seek() = default;

    template <typename Environment>
    auto operator()(Environment&& environment) const
      -> typename semantics::velocity_traits<Velocity>::output_type
    {
      using namespace semantics;

      const auto current {
        extract<Velocity>().from(environment)
      };

      const auto desired {
        extract<Target>().from(environment).normalized() * velocity_traits<Velocity>::max
      };

      return desired - current; // steering
    }
  };
}} // namespace NAMESPACE::behavior

#endif // INCLUDED_BEHAVIOR_BASED_BEHAVIOR_SEEK_HPP

