#ifndef INCLUDED_BEHAVIORS_UTILITY_PRINT_HPP
#define INCLUDED_BEHAVIORS_UTILITY_PRINT_HPP

#define PORT std::cerr

#ifndef NDEBUG
#define PRINT_VECTOR2D(X)                                                      \
PORT << ";\t\t; " #X " = (" << X[0] << " " << X[1] << ")" << std::endl
#else
#define PRINT_VECTOR2D(...)
#endif // NDEBUG

#ifndef NDEBUG
#define PRINT_VALUE(X)                                                         \
PORT << ";\t\t; " #X " = " << X << std::endl
#else
#define PRINT_VALUE(...)
#endif // NDEBUG

#endif // INCLUDED_BEHAVIORS_UTILITY_PRINT_HPP

