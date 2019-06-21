#ifndef INCLUDED_BEHAVIOR_BASED_SEMANTICS_DEFINE_HPP
#define INCLUDED_BEHAVIOR_BASED_SEMANTICS_DEFINE_HPP

#include <utility>

#include <behavior_based/expression/list.hpp>
#include <behavior_based/semantics/facade.hpp>

#define DEFINE_SEMANTICS_CATEGORY(CATEGORY_NAME, DEFAULT_OUTPUT)               \
                                                                               \
template <typename...>                                                         \
struct CATEGORY_NAME;                                                          \
                                                                               \
template <>                                                                    \
struct CATEGORY_NAME<unit>                                                     \
  : public facade<CATEGORY_NAME, unit>                                         \
  , public unit                                                                \
{                                                                              \
  static inline const auto default_output {DEFAULT_OUTPUT};                    \
                                                                               \
  template <typename... Ts>                                                    \
  constexpr CATEGORY_NAME(Ts&&... xs)                                          \
    : unit {std::forward}                                                      \
  {}                                                                           \
                                                                               \
  template <typename... Ts>                                                    \
  constexpr decltype(auto) operator()(Ts&&...) const noexcept                  \
  {                                                                            \
    return default_output;                                                     \
  }                                                                            \
}


#define DEFINE_SEMANTICS_CATEGORY_SPECIALIZATION(CATEGORY, TYPE, ...)          \
template <>                                                                    \
struct CATEGORY<TYPE>                                                          \
  : public facade<CATEGORY, TYPE>                                              \
  , public TYPE                                                                \
{                                                                              \
  template <typename... Ts>                                                    \
  constexpr CATEGORY(Ts&&... xs)                                               \
    : TYPE {std::forward<Ts>(xs)...}                                           \
  {}                                                                           \
                                                                               \
  output_type operator()(const message_type& message) const                    \
  {                                                                            \
    if (message) __VA_ARGS__ else return velocity<unit>::default_output;       \
  }                                                                            \
};


#endif // INCLUDED_BEHAVIOR_BASED_SEMANTICS_DEFINE_HPP

