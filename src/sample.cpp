#include <boost/cstdlib.hpp>

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>

#include <behavior_based/behavior/seek.hpp>
#include <behavior_based/configure.hpp>
#include <behavior_based/expression/dispatch.hpp>
#include <behavior_based/expression/list.hpp>
#include <behavior_based/semantics/target.hpp>
#include <behavior_based/semantics/velocity.hpp>

int main(int argc, char** argv)
{
  using namespace behavior_based;

  ros::init(argc, argv, configure::project_name);

  ros::NodeHandle handle {"~"};

  using environment
    = expression::list<
        nav_msgs::Odometry::ConstPtr,
        sensor_msgs::Joy::ConstPtr
      >;

  environment interaction_environment {};

  using slave
    = behavior::seek<
        semantics::velocity<nav_msgs::Odometry>,
        semantics::target<sensor_msgs::Joy>
      >;

  /**
   * ビヘイビアの線形リスト。
   * 各ビヘイビアはファンクタで対話環境から興味のある情報の抽出方法を知っている。
   * 対話環境に対する各ビヘイビアの出力をフォールドしてシステム全体の出力となる。
   */
  // constexpr expression::list<slave> behaviors {};
  constexpr slave behaviors {};

  /**
   * 最終出力をメッセージとして送信するためののヘルパ関数。
   * 内部にパブリッシャを隠し持ってる。
   * MIMOを想定したAPIだからディスパッチャになってるけど、今回はMISOなのでひとつだけ。
   */
  auto output {expression::dispatch(
    [&](const geometry_msgs::TwistStamped& data)
    {
      static auto p {handle.advertise<geometry_msgs::TwistStamped>("/twist_raw", 1)};
      return p.publish(data);
    }
  )};

  // auto evaluate = [&](auto&& v)
  // {
  //   return geometry::angle
  // };

  #define CALLBACK(TYPENAME, ...)                                              \
  std::function<void (const TYPENAME::ConstPtr&)>                              \
  {                                                                            \
    [&](auto&& message) __VA_ARGS__                                            \
  }

  #define CONNECT(TYPENAME, TOPICNAME)                                         \
  handle.subscribe<TYPENAME>(TOPICNAME, 1,                                     \
    CALLBACK(TYPENAME,                                                         \
    {                                                                          \
      std::cerr << "callback" << std::endl;                                    \
      static_cast<TYPENAME::ConstPtr&>(interaction_environment) = message;     \
                                                                               \
      const auto reaction {behaviors(interaction_environment)};                \
    })                                                                         \
  )

  std::vector<ros::Subscriber> sensory {
    CONNECT(nav_msgs::Odometry, "/odom")
  };

  ros::spin();

  // for (ros::Rate rate {10}; ros::ok(); rate.sleep())
  // {
  //   geometry_msgs::TwistStamped twist {};
  //   twist.header.stamp = ros::Time::now();
  //   twist.twist.linear.x = 1.0;
  //
  //   publish(twist);
  // }

  return boost::exit_success;
}

