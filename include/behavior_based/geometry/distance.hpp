#ifndef INCLUDED_BEHAVIOR_BASED_GEOMETRY_DISTANCE_HPP
#define INCLUDED_BEHAVIOR_BASED_GEOMETRY_DISTANCE_HPP

#include <utility>

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/register/point.hpp>

#include <sensor_msgs/NavSatFix.h>

#include <behavior_based/configure.hpp>

BOOST_GEOMETRY_REGISTER_POINT_2D(
  sensor_msgs::NavSatFix, double, cs::geographic<degree>, longitude, latitude
);

namespace NAMESPACE { namespace geometry
{
  template <typename... Ts>
  inline constexpr decltype(auto) distance(Ts&&... xs)
  {
    return boost::geometry::distance(std::forward<Ts>(xs)...);
  }
}} // namespace NAMESPACE::geometry

#endif // INCLUDED_BEHAVIOR_BASED_GEOMETRY_DISTANCE_HPP

