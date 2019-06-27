#ifndef INCLUDED_BEHAVIOR_BASED_SEMANTICS_TARGET_HPP
#define INCLUDED_BEHAVIOR_BASED_SEMANTICS_TARGET_HPP

#include <sensor_msgs/Joy.h>

#include <behavior_based/configure.hpp>
#include <behavior_based/semantics/define.hpp>

// ターゲットセマンティクスは対話環境から何かしらの「目標地点」を自己からの相対ベクトルとして抽出する。
// 現在は車両型ロボットにしか対応してないためベクトルは二次元

namespace NAMESPACE { namespace semantics
{
  DEFINE_SEMANTICS_CATEGORY(target, output_type::Zero());

  template <typename Target>
  struct target_traits
  {
    using output_type = typename Target::output_type;
  };

  DEFINE_SEMANTICS_CATEGORY_SPECIALIZATION(target, sensor_msgs::Joy,
  {
    std::cerr << "axes[1] " << message->axes[1] << std::endl;
    std::cerr << "axes[2] " << message->axes[2] << std::endl;
    return output_type {message->axes[1], message->axes[2]};
  });
}} // namespace NAMESPACE::semantics

#endif // INCLUDED_BEHAVIOR_BASED_SEMANTICS_TARGET_HPP

