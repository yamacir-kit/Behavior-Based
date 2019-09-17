#ifndef INCLUDED_BEHAVIORS_GEOMETRY_AZIMUTH_HPP
#define INCLUDED_BEHAVIORS_GEOMETRY_AZIMUTH_HPP

#include <cmath>

#include <boost/math/constants/constants.hpp>

#include <Eigen/Core>

#include <sensor_msgs/NavSatFix.h>

#include <behaviors/geometry/distance.hpp>

namespace behaviors { namespace geometry
{
  // 国土地理院あたりからパクってきた数式なのは覚えてるが詳細は忘れた。
  // これ自体が正しいことは確認済み
  //
  // 理由は忘れたけど距離まで出す謎仕様なので注意
  //
  template <typename GeographicPoint>
  auto azimuth(const GeographicPoint& from, const GeographicPoint& to)
    -> Eigen::Vector2d
  {
    const auto phi {
        boost::math::constants::half_pi<double>()
      - std::atan2(
          std::sin(to.longitude - from.longitude),
          std::cos(from.latitude) * std::tan(to.latitude) - std::sin(from.latitude) * std::cos(to.longitude - from.longitude)
        )
    };

    // north   0   0pi ->  0.0pi
    // east   90 0.5pi -> -0.5pi
    // south 180 1.0pi -> -1.0pi
    // west  270 1.5pi -> +0.5pi

    const auto d {distance(from, to)};

    return {
      -std::sin(phi) * d,
      -std::cos(phi) * d
    };
  }
}} // namespace behaviors::geometry

#endif // INCLUDED_BEHAVIORS_GEOMETRY_AZIMUTH_HPP

