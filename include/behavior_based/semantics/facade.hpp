#ifndef INCLUDED_BEHAVIOR_BASED_SEMANTICS_FACADE_HPP
#define INCLUDED_BEHAVIOR_BASED_SEMANTICS_FACADE_HPP

#include <Eigen/Core>

#include <behavior_based/configure.hpp>

namespace NAMESPACE { namespace semantics
{
  template <template <typename...> typename Category, typename Message, typename Vector = Eigen::Vector2d>
  struct facade
  {
    using vector_type = Vector;

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
}} // namespace NAMESPACE::semantics

#endif // INCLUDED_BEHAVIOR_BASED_SEMANTICS_FACADE_HPP

