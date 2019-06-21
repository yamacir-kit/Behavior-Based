#ifndef INCLUDED_BEHAVIOR_BASED_BEHAVIOR_SEEK_HPP
#define INCLUDED_BEHAVIOR_BASED_BEHAVIOR_SEEK_HPP

#include <behavior_based/configure.hpp>
// #include <behavior_based/semantics/extractor.hpp>
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

      const auto current {
        // extract<Velocity>().from(environment)
        static_cast<const Velocity&>(environment)()
      };

      // TODO
      // セマンティクスの関数呼び出し演算子をキャスト演算子に変更して、
      // ２段キャストをセマンティックキャストとしてAPIを用意すること

      const auto desired {
          // extract<Target>().from(environment).normalized()
          static_cast<const Target&>(environment)().normalized()
        * velocity_traits<Velocity>::linear_max
      };

      return desired - current; // steering
    }
  };
}} // namespace NAMESPACE::behavior

#endif // INCLUDED_BEHAVIOR_BASED_BEHAVIOR_SEEK_HPP

