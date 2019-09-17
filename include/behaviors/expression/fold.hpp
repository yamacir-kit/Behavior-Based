#ifndef INCLUDED_BEHAVIORS_EXPRESSION_FOLD_HPP
#define INCLUDED_BEHAVIORS_EXPRESSION_FOLD_HPP

#include <utility>

namespace behaviors { namespace expression
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

    return fold_left(cdr(list), lhs, std::forward<BinaryOperation>(operation));
  }
}} // namespace behaviors::expression

#endif // INCLUDED_BEHAVIORS_EXPRESSION_FOLD_HPP

