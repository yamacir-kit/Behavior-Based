#ifndef INCLUDED_BEHAVIORS_SEMANTICS_FORWARD_HPP

#include <behaviors/semantics/facade.hpp>

namespace behaviors { namespace semantics
{
  DEFINE_SEMANTICS_CATEGORY(forward, vector_type::Zero());

  /**
   * This semantics is implemented for debug purpose.
   *
   * This semantics extract a vector that always advances the robot (note that
   * the vector is the robot's coordinate system) even if empty arguments.
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
}} // namespace behaviors::semantics

#endif // INCLUDED_BEHAVIORS_SEMANTICS_FORWARD_HPP

