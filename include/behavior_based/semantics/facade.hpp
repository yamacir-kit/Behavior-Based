#ifndef INCLUDED_BEHAVIOR_BASED_SEMANTICS_FACADE_HPP
#define INCLUDED_BEHAVIOR_BASED_SEMANTICS_FACADE_HPP

#include <Eigen/Core>

#include <behavior_based/configure.hpp>

namespace NAMESPACE { namespace semantics
{
  template <template <typename...> typename Category, typename Message>
  struct facade
  {
    // 十中八九こいつになるのでもうファサードに埋め込んでおく。
    // 当てはまらない奴はテンプレートパラメータ Category を特殊化して対応すること。
    using output_type = Eigen::Vector2d;

    // using category = Category;

    using message_type = Message;

    template <typename... Ts>
    constexpr decltype(auto) operator()(Ts&&... xs) const
    {
      return static_cast<const Category<Message>&>(*this)(std::forward<Ts>(xs)...);
    }

    // template <typename... Ts>
    // constexpr decltype(auto) operator()(Ts&&... xs)
    // {
    //   return static_cast<Category<Message>&>(*this)(std::forward<Ts>(xs)...);
    // }

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

