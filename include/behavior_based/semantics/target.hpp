#ifndef INCLUDED_BEHAVIOR_BASED_SEMANTICS_TARGET_HPP
#define INCLUDED_BEHAVIOR_BASED_SEMANTICS_TARGET_HPP

#include <sensor_msgs/Joy.h>

#include <behavior_based/configure.hpp>
#include <behavior_based/semantics/define.hpp>

namespace NAMESPACE { namespace semantics
{
  DEFINE_SEMANTICS_CATEGORY(target, vector_type::Zero());

  template <typename Target>
  struct target_traits
  {
    using vector_type = typename Target::vector_type;
  };

  DEFINE_SEMANTICS_CATEGORY_SPECIALIZATION(target, sensor_msgs::Joy,
  {
    return {message->axes[1], message->axes[2]};
  });
}} // namespace NAMESPACE::semantics

#endif // INCLUDED_BEHAVIOR_BASED_SEMANTICS_TARGET_HPP

