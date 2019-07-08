#ifndef INCLUDED_BEHAVIOR_BASED_EXPRESSION_LIST_HPP
#define INCLUDED_BEHAVIOR_BASED_EXPRESSION_LIST_HPP

#include <type_traits>

#include <behavior_based/configure.hpp>

// XXX 線形リスト前提のペア型という点で少し歪な存在

namespace NAMESPACE { namespace expression
{
  template <typename...>
  struct list;

  template <>
  struct list<>
    : public std::false_type
  {};

  using unit = list<>;

  template <typename E>
  struct list<E>
    : public E
    , public unit
  {
    using car = E;
    using cdr = unit;

    constexpr list() = default;

    template <typename... Ts>
    constexpr list(Ts&&... xs)
      : car {std::forward<Ts>(xs)...}
    {}
  };

  template <typename E1, typename E2, typename... Es>
  struct list<E1, E2, Es...>
    : public E1
    , public list<E2, Es...>
  {
    using car = E1;
    using cdr = list<E1, Es...>;

    constexpr list() = default;

    template <typename T, typename... Ts>
    constexpr list(T&& x, Ts&&... xs)
      : car {std::forward<T>(x)}
      , cdr {std::forward<Ts>(xs)...}
    {}
  };

  #define SELECTOR(SLOT) \
  template <typename List> \
  constexpr decltype(auto) SLOT(const List& list) noexcept \
  { \
    return static_cast<const typename List::SLOT&>(list); \
  }

  SELECTOR(car)
  SELECTOR(cdr)

  template <typename... Ts>
  constexpr auto make_list(Ts&&... xs)
    -> list<typename std::decay<Ts>::type...>
  {
    return {std::forward<Ts>(xs)...};
  }
}} // namespace NAMESPACE::expression

#endif // INCLUDED_BEHAVIOR_BASED_EXPRESSION_LIST_HPP

