#ifndef INCLUDED_BEHAVIORS_TYPE_TRATIS_VOID_T_HPP
#define INCLUDED_BEHAVIORS_TYPE_TRATIS_VOID_T_HPP

#include <type_traits>

namespace behaviors { namespace type_traits
{
  template <typename...>
  using void_t = void;
}} // namespace behaviors::type_traits

#endif // INCLUDED_BEHAVIORS_TYPE_TRATIS_VOID_T_HPP

