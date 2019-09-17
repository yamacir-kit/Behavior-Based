#ifndef INCLUDED_BEHAVIORS_SEMANTICS_FACADE_HPP
#define INCLUDED_BEHAVIORS_SEMANTICS_FACADE_HPP

#include <Eigen/Core>

namespace behaviors { namespace semantics
{
  template <template <typename...> typename Category, typename Message>
  struct facade
  {
    using vector_type = Eigen::Vector2d;

    using message_type = Message;

    template <typename... Ts>
    constexpr decltype(auto) operator()(Ts&&... xs) const
    {
      return static_cast<const Category<Message>&>(*this)(std::forward<Ts>(xs)...);
    }

    template <typename... Ts>
    constexpr decltype(auto) from(Ts&&... xs) const
    {
      return operator()(std::forward<Ts>(xs)...);
    }

    template <typename... Ts>
    constexpr decltype(auto) from(Ts&&... xs)
    {
      return operator()(std::forward<Ts>(xs)...);
    }
  };
}} // namespace behaviors::semantics

#endif // INCLUDED_BEHAVIORS_SEMANTICS_FACADE_HPP

