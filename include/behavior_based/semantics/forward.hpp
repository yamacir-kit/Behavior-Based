#ifndef INCLUDED_BEHAVIOR_BASED_SEMANTICS_FORWARD_HPP

#include <behavior_based/configure.hpp>
#include <behavior_based/semantics/facade.hpp>

namespace NAMESPACE { namespace semantics
{
  DEFINE_SEMANTICS_CATEGORY(forward, output_type::Zero());

  /**
   * This semantics is implemented for debug purpose.
   */
  template <typename Vector>
  struct forward<Vector>
    : public facade<forward, Vector>
  {
    template <typename... Ts>
    Vector operator()(Ts&&...) const
    {
      return Vector::UnitX();
    }
  };
}} // namespace NAMESPACE::semantics

#endif // INCLUDED_BEHAVIOR_BASED_SEMANTICS_FORWARD_HPP

