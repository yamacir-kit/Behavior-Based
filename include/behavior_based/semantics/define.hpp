#ifndef INCLUDED_BEHAVIOR_BASED_SEMANTICS_DEFINE_HPP
#define INCLUDED_BEHAVIOR_BASED_SEMANTICS_DEFINE_HPP

#include <utility>

#include <behavior_based/expression/list.hpp>
#include <behavior_based/semantics/facade.hpp>

// 関数呼び出し演算子になってるのはAPI変更のための一時的な仕様なので勘弁

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
  decltype(auto) operator()() const noexcept                                   \
  {                                                                            \
    return DEFAULT_OUTPUT;                                                     \
  }                                                                            \
}


#define DEFINE_SEMANTICS_CATEGORY_SPECIALIZATION(CATEGORY, TYPE, ...)          \
template <>                                                                    \
struct CATEGORY<TYPE>                                                          \
  : public facade<CATEGORY, TYPE>                                              \
  , public TYPE::ConstPtr                                                      \
{                                                                              \
  template <typename... Ts>                                                    \
  constexpr CATEGORY(Ts&&... xs)                                               \
    : TYPE::ConstPtr {std::forward<Ts>(xs)...}                                 \
  {}                                                                           \
                                                                               \
  output_type operator()() const                                               \
  {                                                                            \
    if (*this) __VA_ARGS__ else return velocity<expression::unit> {}();        \
  }                                                                            \
};


#endif // INCLUDED_BEHAVIOR_BASED_SEMANTICS_DEFINE_HPP

