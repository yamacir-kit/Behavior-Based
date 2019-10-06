#ifndef INCLUDED_BEHAVIORS_SEMANTICS_DEFINE_HPP
#define INCLUDED_BEHAVIORS_SEMANTICS_DEFINE_HPP

#include <utility>

#include <behaviors/expression/list.hpp>
#include <behaviors/semantics/facade.hpp>

#ifndef NDEBUG
#define PRINT_SEMANTICS_CATEGORY_INFORMATION(CATEGORY)                         \
std::cerr << ";\n"                                                             \
          << "; semantics\t; category " << #CATEGORY << std::endl;             \
std::cerr << ";\t\t; invoked default value!\n"                                 \
          << ";\t\t; two situations are possible;\n"                           \
          << ";\t\t;   (1) the message has not been accepted (a.k.a meessage == nullptr)\n" \
          << ";\t\t;   (2) called explicitly (in many cases, meaning no action is required)\n" \
          << std::flush;
#else // NDEBUG
#define PRINT_SEMANTICS_CATEGORY_INFORMATION(...)
#endif // NDEBUG

#define DEFINE_SEMANTICS_CATEGORY(CATEGORY, DEFAULT_VECTOR)                    \
template <typename...>                                                         \
struct CATEGORY;                                                               \
template <>                                                                    \
struct CATEGORY<expression::unit>                                              \
  : public facade<CATEGORY, expression::unit>                                  \
  , public expression::unit                                                    \
{                                                                              \
  template <typename... Ts>                                                    \
  constexpr CATEGORY(Ts&&... xs)                                               \
    : expression::unit {std::forward<Ts>(xs)...}                               \
  {}                                                                           \
                                                                               \
  template <typename... Ts>                                                    \
  decltype(auto) operator()(Ts&&...) const noexcept                            \
  {                                                                            \
    PRINT_SEMANTICS_CATEGORY_INFORMATION(CATEGORY)                             \
    return DEFAULT_VECTOR;                                                     \
  }                                                                            \
}

#ifndef NDEBUG
#define PRINT_SEMANTICS_CATEGORY_SPECIALIZAION_INFORMATION(CATEGORY, TYPE)     \
std::cerr << ";\n"                                                             \
          << "; semantics\t; category " << #CATEGORY << std::endl;             \
std::cerr << ";\t\t; specialized for " << #TYPE << std::endl;
#else // NDEBUG
#define PRINT_SEMANTICS_CATEGORY_SPECIALIZAION_INFORMATION(...)
#endif // NDEBUG

#define DEFINE_SEMANTICS_CATEGORY_SPECIALIZATION(CATEGORY, TYPE, ...)          \
template <>                                                                    \
struct CATEGORY<TYPE>                                                          \
  : public facade<CATEGORY, TYPE>                                              \
{                                                                              \
  vector_type operator()(const TYPE::ConstPtr& message) const                  \
  {                                                                            \
    PRINT_SEMANTICS_CATEGORY_SPECIALIZAION_INFORMATION(CATEGORY, TYPE)         \
    if (message) __VA_ARGS__ else return CATEGORY<expression::unit> {}();      \
  }                                                                            \
};


#endif // INCLUDED_BEHAVIORS_SEMANTICS_DEFINE_HPP

