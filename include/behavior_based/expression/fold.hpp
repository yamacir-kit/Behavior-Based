#ifndef INCLUDED_BEHAVIOR_BASED_EXPRESSION_FOLD_HPP
#define INCLUDED_BEHAVIOR_BASED_EXPRESSION_FOLD_HPP

#include <utility>

#include <behavior_based/configure.hpp>

namespace NAMESPACE { namespace expression
{
  template <typename T, typename Initial, typename BinaryOperation>
  constexpr decltype(auto) fold_left(const list<T>& list, Initial&& initial, BinaryOperation&& operation)
  {
    return operation(std::forward<Initial>(initial), car(list));
  }

  template <typename T, typename U, typename... Ts, typename Initial, typename BinaryOperation>
  constexpr decltype(auto) fold_left(const list<T, U, Ts...>& list, Initial&& initial, BinaryOperation&& operation)
  {
    /**
     * Many C ++ compilers generate code that evaluates function arguments in
     * order from the last argument. Buffer the results once to ensure that the
     * side effects occur in order from the left element of the list.
     */
    auto lhs {operation(std::forward<Initial>(initial), car(list))};

    return fold_left(
             static_cast<const typename std::decay<decltype(list)>::type::cdr&>(list),
             lhs,
             std::forward<BinaryOperation>(operation)
           );
  }
}} // namespace NAMESPACE::expression

#endif // INCLUDED_BEHAVIOR_BASED_EXPRESSION_FOLD_HPP

