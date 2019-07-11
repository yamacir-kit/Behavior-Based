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
struct CATEGORY_NAME<expression::unit>                                         \
  : public facade<CATEGORY_NAME, expression::unit>                             \
  , public expression::unit                                                    \
{                                                                              \
  template <typename... Ts>                                                    \
  constexpr CATEGORY_NAME(Ts&&... xs)                                          \
    : expression::unit {std::forward<Ts>(xs)...}                               \
  {}                                                                           \
                                                                               \
  template <typename... Ts>                                                    \
  decltype(auto) operator()(Ts&&...) const noexcept                            \
  {                                                                            \
    return DEFAULT_OUTPUT;                                                     \
  }                                                                            \
}


#define DEFINE_SEMANTICS_CATEGORY_SPECIALIZATION(CATEGORY, TYPE, ...)          \
template <>                                                                    \
struct CATEGORY<TYPE>                                                          \
  : public facade<CATEGORY, TYPE>                                              \
{                                                                              \
  output_type operator()(const TYPE::ConstPtr& message) const                  \
  {                                                                            \
    if (message) __VA_ARGS__ else return CATEGORY<expression::unit> {}();      \
  }                                                                            \
};


#endif // INCLUDED_BEHAVIOR_BASED_SEMANTICS_DEFINE_HPP

