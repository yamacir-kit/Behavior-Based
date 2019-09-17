#ifndef INCLUDED_BEHAVIORS_CONFIGURE_HPP
#define INCLUDED_BEHAVIORS_CONFIGURE_HPP

#include <string>

// TODO?
// std::string => std::array<char, N>

namespace behaviors { namespace configure
{
  static const std::string project_name {"${PROJECT_NAME}"};

  static const std::string build_time {"${${PROJECT_NAME}_TIMESTAMP}"};
  static const std::string build_type {"${CMAKE_BUILD_TYPE}"};

  static const std::string major_version {"${PROJECT_VERSION_MAJOR}"},
                           minor_version {"${PROJECT_VERSION_MINOR}"},
                           patch_version {"${PROJECT_VERSION_PATCH}"};

  static const std::string version {"${PROJECT_VERSION}"};

  // static const struct
  //   : public std::string
  // {
  //   const std::string major {"${PROJECT_VERSION_MAJOR}"},
  //                     minor {"${PROJECT_VERSION_MINOR}"},
  //                     patch {"${PROJECT_VERSION_PATCH}"};
  // } version {"${PROJECT_VERSION}"};
}} // namespace behaviors::configure

#endif // INCLUDED_BEHAVIORS_CONFIGURE_HPP

