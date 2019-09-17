#ifndef INCLUDED_BEHAVIORS_GEOMETRY_DISTANCE_HPP
#define INCLUDED_BEHAVIORS_GEOMETRY_DISTANCE_HPP

#include <utility>

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/register/point.hpp>

#include <sensor_msgs/NavSatFix.h>

BOOST_GEOMETRY_REGISTER_POINT_2D(
  sensor_msgs::NavSatFix, double, cs::geographic<degree>, longitude, latitude
);

namespace behaviors { namespace geometry
{
  template <typename... Ts>
  inline constexpr decltype(auto) distance(Ts&&... xs)
  {
    return boost::geometry::distance(std::forward<Ts>(xs)...);
  }
}} // namespace behaviors::geometry

#endif // INCLUDED_BEHAVIORS_GEOMETRY_DISTANCE_HPP

