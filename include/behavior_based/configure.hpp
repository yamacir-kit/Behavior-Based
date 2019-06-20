#ifndef INCLUDED_BEHAVIOR_BASED_CONFIGURE_HPP
#define INCLUDED_BEHAVIOR_BASED_CONFIGURE_HPP

#include <string>

#define NAMESPACE behavior_based

// TODO?
// std::string => std::array<char, N>

namespace NAMESPACE { namespace configure
{
  static const std::string project_name {"behavior_based"};

  static const std::string build_time {"2019/06/21 08:00:22"};
  static const std::string build_type {"Debug"};

  static const std::string major_version {"0"},
                           minor_version {"0"},
                           patch_version {"24"};

  static const std::string version {"0.0.24"};
}} // namespace NAMESPACE::configure

#endif // INCLUDED_BEHAVIOR_BASED_CONFIGURE_HPP

