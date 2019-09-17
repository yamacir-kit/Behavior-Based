#ifndef INCLUDED_BEHAVIORS_UTILITY_DEMANGLE_HPP
#define INCLUDED_BEHAVIORS_UTILITY_DEMANGLE_HPP

#include <cstdlib>
#include <memory>
#include <string>
#include <typeinfo>

#include <cxxabi.h>

namespace behaviors { namespace utility
{
  std::string demangle(const char* name)
  {
    int failed {};

    std::unique_ptr<char, decltype(&std::free)> demangled
    {
      abi::__cxa_demangle(name, nullptr, nullptr, &failed),
      [](void* pointer) noexcept { std::free(pointer); }
    };

    return {failed ? name : demangled.get()};
  }

  decltype(auto) demangle(const std::type_info& info)
  {
    return demangle(info.name());
  }
}} // namespace behaviors::utility

#endif // INCLUDED_BEHAVIORS_UTILITY_DEMANGLE_HPP

