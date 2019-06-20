#ifndef INCLUDED_BEHAVIOR_BASED_SEMANTICS_UNIT_HPP
#define INCLUDED_BEHAVIOR_BASED_SEMANTICS_UNIT_HPP

#include <utility>

#include <behavior_based/configure.hpp>
#include <behavior_based/expression/list.hpp>
#include <behavior_based/semantics/facade.hpp>

#define SEMANTICS(CATEGORY, ...)                                               \
template <typename...>
struct CATEGORY;

template <>
struct CATEGORY<unit>
  : public facade<CATEGORY, unit>
  , unit
{
  static inline const auto default_output {__VA_ARGS__};

  template <typename... Ts>
  constexpr CATEGORY(Ts&&... xs)
    : unit {std::forward<Ts>(xs)...}
  {}

  template <>
};

#endif // INCLUDED_BEHAVIOR_BASED_SEMANTICS_UNIT_HPP

