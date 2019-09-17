#ifndef INCLUDED_BEHAVIORS_SEMANTICS_EXTRACTOR_HPP
#define INCLUDED_BEHAVIORS_SEMANTICS_EXTRACTOR_HPP

namespace behaviors { namespace semantics
{
  template <typename Semantics, typename... Ts>
  Semantics extract(Ts&&... xs) noexcept
  {
    return {std::forward<Ts>(xs)...};
  }
}} // namespace behaviors::semantics

#endif // INCLUDED_BEHAVIORS_SEMANTICS_EXTRACTOR_HPP

