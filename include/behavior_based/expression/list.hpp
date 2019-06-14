#ifndef INCLUDED_BEHAVIOR_BASED_EXPRESSION_LIST_HPP
#define INCLUDED_BEHAVIOR_BASED_EXPRESSION_LIST_HPP

#include <type_traits>

#include <behavior_based/configure.hpp>

namespace NAMESPACE { namespace expression
{
  template <typename...>
  struct list;

  template <>
  struct list<>
    : public std::false_type
  {};

  template <typename E>
  struct list<E>
    : public E
    , public list<>
  {
    using car = E;
    using cdr = list<>;

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

  constexpr list<> unit {};

  #define SELECTOR(NAME) \
  template <typename List> \
  constexpr decltype(auto) NAME(const List& list) noexcept \
  { \
    return static_cast<const typename List::NAME&>(list); \
  }

  SELECTOR(car)
  SELECTOR(cdr)

  // template <typename... Ts>
  // constexpr auto list(Ts&&... xs)
  //   -> list<typename std::decay<Ts>::type...>
  // {
  //   return {std::forward<Ts>(xs)...};
  // }
}} // namespace NAMESPACE::expression

#endif // INCLUDED_BEHAVIOR_BASED_EXPRESSION_LIST_HPP

