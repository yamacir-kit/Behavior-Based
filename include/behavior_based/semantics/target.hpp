#ifndef INCLUDED_BEHAVIOR_BASED_SEMANTICS_TARGET_HPP
#define INCLUDED_BEHAVIOR_BASED_SEMANTICS_TARGET_HPP

#include <algorithm>

#include <lgsvl_msgs/Detection3DArray.h>
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

  DEFINE_SEMANTICS_CATEGORY_SPECIALIZATION(target, lgsvl_msgs::Detection3DArray,
  {
    // std::cerr << "message->detections.size() = " << message->detections.size() << std::endl;

    if (not message->detections.empty())
    {
      auto distance = [](const auto& detection)
      {
        return std::sqrt(
                 std::pow(detection.bbox.position.position.x, 2)
               + std::pow(detection.bbox.position.position.y, 2)
               );
      };

      const auto iter {std::min_element(
        message->detections.begin(),
        message->detections.end(),
        [&](const auto& lhs, const auto& rhs)
        {
          return distance(lhs) < distance(rhs);
        }
      )};

      return {iter->bbox.position.position.x, iter->bbox.position.position.y};
    }
    else
    {
      return vector_type::Zero();
    }
  });
}} // namespace NAMESPACE::semantics

#endif // INCLUDED_BEHAVIOR_BASED_SEMANTICS_TARGET_HPP

