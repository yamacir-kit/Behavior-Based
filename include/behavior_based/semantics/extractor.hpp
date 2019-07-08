#ifndef INCLUDED_BEHAVIOR_BASED_SEMANTICS_EXTRACTOR_HPP
#define INCLUDED_BEHAVIOR_BASED_SEMANTICS_EXTRACTOR_HPP

#include <behavior_based/configure.hpp>

namespace NAMESPACE { namespace semantics
{
  template <typename Semantics, typename... Ts>
  auto extract(Ts&&... xs) noexcept
  {
    Semantics semantics {std::forward<Ts>(xs)...};
    return semantics;
  }
}} // namespace NAMESPACE::semantics

#endif // INCLUDED_BEHAVIOR_BASED_SEMANTICS_EXTRACTOR_HPP

