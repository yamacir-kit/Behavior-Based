#ifndef INCLUDED_BEHAVIORS_UTILITY_PRINT_HPP
#define INCLUDED_BEHAVIORS_UTILITY_PRINT_HPP

#define PRINT_PORT std::cerr

#ifndef NDEBUG
#define PRINT_VECTOR2D(X)                                                      \
PRINT_PORT << ";\t\t; " #X " = (" << X[0] << " " << X[1] << ")" << std::endl
#else
#define PRINT_VECTOR2D(...)
#endif

#endif // INCLUDED_BEHAVIORS_UTILITY_PRINT_HPP

