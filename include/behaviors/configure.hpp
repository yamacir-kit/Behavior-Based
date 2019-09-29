#ifndef INCLUDED_BEHAVIORS_CONFIGURE_HPP
#define INCLUDED_BEHAVIORS_CONFIGURE_HPP

#include <string>
#include <utility>

// TODO?
// std::string => std::array<char, N>

namespace behaviors { namespace configure
{
  static const std::string project_name {"behaviors"};

  static const std::string build_hash {""};
  static const std::string build_time {"2019/09/29 23:43:08"};
  static const std::string build_type {"Debug"};

  static const struct version
    : public std::string
  {
    template <typename... Ts>
    explicit constexpr version(Ts&&... xs)
      : std::string {std::forward<Ts>(xs)...}
    {}

    const std::string major {"0"},
                      minor {"0"},
                      patch {"85"};
  } version {"0.0.85"};
}} // namespace behaviors::configure

#endif // INCLUDED_BEHAVIORS_CONFIGURE_HPP

