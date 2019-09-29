#ifndef INCLUDED_BEHAVIORS_UTILITY_FORWARD_HPP
#define INCLUDED_BEHAVIORS_UTILITY_FORWARD_HPP

#include <utility>

namespace behaviors { namespace utility
{
  #define FORWARD(...) std::forward<decltype(__VA_ARGS__)>(__VA_ARGS__)

  template <typename... Ts>
  constexpr decltype(auto) forward_capture(Ts&&... xs)
  {
    return std::tuple<Ts...>(FORWARD(xs)...);
  }

  #define FORWARD_CAPTURE(...) forward_capture(FORWARD(__VA_ARGS__))

  template <typename... Ts>
  constexpr decltype(auto) forward_variadic_capture(Ts&&... xs)
  {
    return std::make_tuple(FORWARD_CAPTURE(xs)...);
  }

  #define FORWARD_VARIADIC_CAPTURE(...) forward_variadic_capture(FORWARD(__VA_ARGS__)...)

  template <typename T>
  constexpr decltype(auto) captured(T&& x)
  {
    return std::get<0>(FORWARD(x));
  }
}} // namespace behaviors::utility

#endif // INCLUDED_BEHAVIORS_UTILITY_FORWARD_HPP

