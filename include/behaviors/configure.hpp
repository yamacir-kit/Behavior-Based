#ifndef INCLUDED_BEHAVIORS_CONFIGURE_HPP
#define INCLUDED_BEHAVIORS_CONFIGURE_HPP

#include <string>

// TODO?
// std::string => std::array<char, N>

namespace behaviors { namespace configure
{
  static const std::string project_name {"behaviors"};

  static const std::string build_time {"2019/09/17 11:01:23"};
  static const std::string build_type {"Release"};

  static const std::string major_version {"0"},
                           minor_version {"0"},
                           patch_version {"74"};

  static const std::string version {"0.0.74"};

  // static const struct
  //   : public std::string
  // {
  //   const std::string major {"0"},
  //                     minor {"0"},
  //                     patch {"74"};
  // } version {"0.0.74"};
}} // namespace behaviors::configure

#endif // INCLUDED_BEHAVIORS_CONFIGURE_HPP

