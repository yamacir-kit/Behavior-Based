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
    using value_type = typename Velocity::value_type;

    constexpr max = 10.0; // [m/s]
  };

  template <typename...>
  struct velocity;

  // 意味が抽出できない場合の返り値の規定
  template <>
  struct velocity<>
    : public facade<velocity<unit>>
    , public unit // 環境のキャスト保証のため
  {
    using value_type = Eigen::Vector2d;

    using message_type = unit;

    static inline const auto default_value {value_type::Zero()};

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
    : public facade<velocity<TYPE>>                                            \
    , public TYPE                                                              \
  {                                                                            \
    using message_type = TYPE::ConstPtr;                                       \
    using value_type = Eigen::Vector2d;                                        \
                                                                               \
    template <typename... Ts>                                                  \
    constexpr velocity(Ts&&... xs)                                             \
      : TYPE {std::forward<Ts>(xs)...}                                         \
    {}                                                                         \
                                                                               \
    value_type operator()(const message_type& message) const                   \
    {                                                                          \
      if (message) __VA_ARGS__ else return velocity<>::default_value;          \
    }                                                                          \
  };

  VELOCITY_SRMANTICS_FOR(nav_msgs::Odometry,
  {
    const auto& twist {(*message).twist.twist};

    const value_type value {
      std::cos(twist.angular.z),
      std::sin(twist.angular.z)
    };

    return value * twist.linear.x;
  })
}} // namespace NAMESPACE::semantics

#endif // INCLUDED_BEHAVIOR_BASED_SEMANTICS_VELOCITY_HPP

