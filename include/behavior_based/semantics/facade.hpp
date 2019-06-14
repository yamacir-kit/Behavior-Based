#ifndef INCLUDED_BEHAVIOR_BASED_SEMANTICS_FACADE_HPP
#define INCLUDED_BEHAVIOR_BASED_SEMANTICS_FACADE_HPP

#include <behavior_based/configure.hpp>

namespace NAMESPACE { namespace semantics
{
  template <typename T>
  struct facade
  {
    template <typename... Ts>
    constexpr decltype(auto) operator()(Ts&&... xs) const
    {
      return static_cast<const T&>(*this)(std::forward<Ts>(xs)...);
    }

    template <typename... Ts>
    constexpr decltype(auto) operator()(Ts&&... xs)
    {
      return static_cast<T&>(*this)(std::forward<Ts>(xs)...);
    }

    template <typename... Ts>
    [[deprecated]] constexpr decltype(auto) from(Ts&&... xs) const
    {
      return operator()(std::forward<Ts>(xs)...);
    }

    template <typename... Ts>
    [[deprecated]] constexpr decltype(auto) from(Ts&&... xs)
    {
      return operator()(std::forward<Ts>(xs)...);
    }
  };
}} // namespace NAMESPACE::semantics

#endif // INCLUDED_BEHAVIOR_BASED_SEMANTICS_FACADE_HPP

