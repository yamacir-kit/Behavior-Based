#ifndef INCLUDED_BEHAVIOR_BASED_TYPE_TRATIS_VOID_T_HPP
#define INCLUDED_BEHAVIOR_BASED_TYPE_TRATIS_VOID_T_HPP

#include <type_traits>

#include <behavior_based/configure.hpp>

namespace NAMESPACE { namespace type_traits
{
  template <typename...>
  using void_t = void;
}} // namespace NAMESPACE::type_traits

#endif // INCLUDED_BEHAVIOR_BASED_TYPE_TRATIS_VOID_T_HPP

