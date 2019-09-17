#ifndef INCLUDE_BEHAVIORS_GEOMETRY_ANGLE_HPP
#define INCLUDE_BEHAVIORS_GEOMETRY_ANGLE_HPP

#include <cmath>
#include <type_traits>

#include <boost/algorithm/clamp.hpp>
#include <boost/geometry/algorithms/distance.hpp>

namespace behaviors { namespace geometry
{
  template <typename V1, typename V2>
  decltype(auto) angle(const V1& v1, const V2& v2)
  {
    const auto norm {v1.norm() * v2.norm()};

    if (boost::geometry::math::equals(norm, 0))
    {
      return 0.0; // XXX Probably mathematically incorrect
    }
    else
    {
      // TODO Clamp has been standardized since C++17
      return std::acos(boost::algorithm::clamp(v1.dot(v2) / norm, -1.0, +1.0));
    }
  }
}} // namespace behaviors::geometry

#endif // INCLUDE_BEHAVIORS_GEOMETRY_ANGLE_HPP
