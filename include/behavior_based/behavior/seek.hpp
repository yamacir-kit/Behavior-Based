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
    auto operator()(const Environment& environment) const
      -> typename semantics::velocity_traits<Velocity>::output_type
    {
      using namespace semantics;

      const auto current {extract<Velocity>().from(environment)};
      std::cerr << "[debug] current: " << current << std::endl;

      std::cerr << "[target] " << extract<Target>().from(environment) << std::endl;
      std::cerr << "[normalized] " << extract<Target>().from(environment).normalized() << std::endl;
      const auto desired {
          extract<Target>().from(environment).normalized()
        * velocity_traits<Velocity>::linear_max
      };
      std::cerr << "[debug] desired: " << desired << std::endl;

      return desired - current; // steering
    }
  };
}} // namespace NAMESPACE::behavior

#endif // INCLUDED_BEHAVIOR_BASED_BEHAVIOR_SEEK_HPP

