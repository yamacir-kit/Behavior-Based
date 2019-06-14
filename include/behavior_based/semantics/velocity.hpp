#ifndef INCLUDED_BEHAVIOR_BASED_SEMANTICS_VELOCITY_HPP
#define INCLUDED_BEHAVIOR_BASED_SEMANTICS_VELOCITY_HPP

#include <cmath>

#include <Eigen/Core>

#include <nav_msgs/Odometry.h>

#include <behavior_based/configure.hpp>
#include <behavior_base/expression/list.hpp>

namespace NAMESPACE { namespace semantics
{
  template <typename Velocity>
  struct velocity_traits
  {
    using output_type = typename Velocity::output_type;

    constexpr max = 10.0; // [m/s]
  };

  template <typename...>
  struct velocity;

  // 意味が抽出できない場合の返り値の規定
  template <>
  struct velocity<unit>
    : public facade<velocity, unit>
    , public unit // 環境のキャスト保証のため
  {
    static inline const auto default_value {output_type::Zero()};

    template <typename... Ts>
    constexpr velocity(Ts&&... xs)
      : unit {std::forward<Ts>(xs)...}
    {}

    template <typename... Ts>
    constexpr decltype(auto) operator()(Ts&&...) const noexcept
    {
      return default_value;
    }
  };

  #define VELOCITY_SRMANTICS_FOR(TYPE, ...)                                    \
  template <>                                                                  \
  struct velocity<TYPE>                                                        \
    : public facade<velocity, TYPE>                                            \
    , public TYPE                                                              \
  {                                                                            \
    template <typename... Ts>                                                  \
    constexpr velocity(Ts&&... xs)                                             \
      : TYPE {std::forward<Ts>(xs)...}                                         \
    {}                                                                         \
                                                                               \
    output_type operator()(const message_type& message) const                  \
    {                                                                          \
      if (message) __VA_ARGS__ else return velocity<unit>::default_value;      \
    }                                                                          \
  };

  VELOCITY_SRMANTICS_FOR(nav_msgs::Odometry,
  {
    const auto& twist {(*message).twist.twist};

    const output_type value {
      std::cos(twist.angular.z),
      std::sin(twist.angular.z)
    };

    return value * twist.linear.x;
  })
}} // namespace NAMESPACE::semantics

#endif // INCLUDED_BEHAVIOR_BASED_SEMANTICS_VELOCITY_HPP

