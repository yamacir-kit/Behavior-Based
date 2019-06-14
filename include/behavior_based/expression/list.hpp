#ifndef INCLUDED_BEHAVIOR_BASED_EXPRESSION_LIST_HPP
#define INCLUDED_BEHAVIOR_BASED_EXPRESSION_LIST_HPP

#include <type_traits>

#include <behavior_based/configure.hpp>

namespace NAMESPACE { namespace expression
{
  template <typename...>
  struct cons;

  template <>
  struct cons<>
    : public std::false_type
  {};

  template <typename E>
  struct cons<E>
    : public E
    , public cons<>
  {
    using car = E;
    using cdr = cons<>;

    constexpr cons() = default;

    template <typename... Ts>
    constexpr cons(Ts&&... xs)
      : car {std::forward<Ts>(xs)...}
    {}
  };

  template <typename E1, typename E2, typename... Es>
  struct cons<E1, E2, Es...>
    : public E1
    , public cons<E2, Es...>
  {
    using car = E1;
    using cdr = cons<E1, Es...>;

    constexpr cons() = default;

    template <typename T, typename... Ts>
    constexpr cons(T&& x, Ts&&... xs)
      : car {std::forward<T>(x)}
      , cdr {std::forward<Ts>(xs)...}
    {}
  };

  constexpr cons<> unit {};

  template <typename T>
  constexpr decltype(auto) car(const T& x) noexcept
  {
    return static_cast<const typename T::car&>(x);
  }

  template <typename T>
  constexpr decltype(auto) cdr(const T& x) noexcept
  {
    return static_cast<const typename T::cdr&>(x);
  }

  template <typename... Ts>
  constexpr auto list(Ts&&... xs)
    -> cons<typename std::decay<Ts>::type...>
  {
    return {std::forward<Ts>(xs)...};
  }
}} // namespace NAMESPACE::expression

#endif // INCLUDED_BEHAVIOR_BASED_EXPRESSION_LIST_HPP

