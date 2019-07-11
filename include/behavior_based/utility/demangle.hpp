#ifndef INCLUDED_BEHAVIOR_BASED_UTILITY_DEMANGLE_HPP
#define INCLUDED_BEHAVIOR_BASED_UTILITY_DEMANGLE_HPP

#include <cstdlib>
#include <memory>
#include <string>
#include <typeinfo>

#include <cxxabi.h>

#include <behavior_based/configure.hpp>

namespace NAMESPACE { namespace utility
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
}} // namespace NAMESPACE::utility

#endif // INCLUDED_BEHAVIOR_BASED_UTILITY_DEMANGLE_HPP

