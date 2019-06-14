#ifndef INCLUDED_BEHAVIOR_BASED_CONFIGURE_HPP
#define INCLUDED_BEHAVIOR_BASED_CONFIGURE_HPP

#include <string>

#include <boost/vesion.hpp>

#define NAMESPACE ${PROJECT_NAME}

// TODO?
// std::string => std::array<char, N>

namespace NAMESPACE { namespace configure
{
  static const std::string project_name {"${PROJECT_NAME}"};

  static const std::string build_time {"${${PROJECT_NAME}_TIMESTAMP}"};
  static const std::string build_type {"${CMAKE_BUILD_TYPE}"};

  static const std::string major_version {"${PROJECT_VERSION_MAJOR}"},
                           minor_version {"${PROJECT_VERSION_MINOR}"},
                           patch_version {"${PROJECT_VERSION_PATCH}"};

  static const std::string version {"${PROJECT_VERSION}"};

  static const std::string boost_major_version {"${Boost_MAJOR_VERSION}"},
                           boost_minor_version {"${Boost_MINOR_VERSION}"},
                           boost_patch_version {"${Boost_SUBMINOR_VERSION}"};

  static const std::string boost_version {
                          boost_major_version
    + std::string {"."} + boost_minor_version
    + std::string {"."} + boost_patch_version
  };
}} // namespace NAMESPACE::configure

#endif // INCLUDED_BEHAVIOR_BASED_CONFIGURE_HPP

