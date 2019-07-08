#ifndef INCLUDED_BEHAVIOR_BASED_SEMANTICS_EXTRACTOR_HPP
#define INCLUDED_BEHAVIOR_BASED_SEMANTICS_EXTRACTOR_HPP

#include <behavior_based/configure.hpp>

namespace NAMESPACE { namespace semantics
{
  template <typename Semantics, typename... Ts>
  Semantics extract(Ts&&... xs) noexcept
  {
    return {std::forward<Ts>(xs)...};
  }
}} // namespace NAMESPACE::semantics

#endif // INCLUDED_BEHAVIOR_BASED_SEMANTICS_EXTRACTOR_HPP

