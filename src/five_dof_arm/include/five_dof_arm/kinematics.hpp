// Original arm geometry: modified DH, dimensions in millimetres.
#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace five_dof_arm
{
constexpr double pi = 3.14159265358979323846;
using Positions = std::array<int32_t, 5>;
using Angles = Eigen::Matrix<double, 5, 1>;
inline double ticks_to_radians(double ticks) {return (ticks - 2048.0) * pi / 2048.0;}
inline int32_t radians_to_ticks(double angle)
{return static_cast<int32_t>(std::lround(angle * 2048.0 / pi + 2048.0));}

inline Eigen::Matrix4d mdh(double theta, double alpha, double a, double d)
{
  const double c = std::cos(theta), s = std::sin(theta);
  const double ca = std::cos(alpha), sa = std::sin(alpha);
  Eigen::Matrix4d t;
  t << c, -s, 0, a, s * ca, c * ca, -sa, -d * sa,
    s * sa, c * sa, ca, d * ca, 0, 0, 0, 1;
  return t;
}

class Kinematics
{
public:
  double a2 = 246.06, a3 = 242.25, tool = 255.55;
  Eigen::Matrix4d position_target(const Positions & previous, const Eigen::Vector3d & xyz) const
  {
    Eigen::Matrix4d target = forward(previous);
    // XYZ input does not specify a full pose. Follow the target's base yaw,
    // keeping tool pitch and roll relative to the arm's radial plane.
    if (xyz.head<2>().norm() > 1e-8) {
      const double delta = std::atan2(xyz.y(), xyz.x()) -
        std::atan2(target(1, 3), target(0, 3));
      target.block<3, 3>(0, 0) = Eigen::AngleAxisd(delta, Eigen::Vector3d::UnitZ()) *
        target.block<3, 3>(0, 0).eval();
    }
    target.block<3, 1>(0, 3) = xyz;
    return target;
  }
  Eigen::Matrix4d forward(const Angles & q) const
  {
    return mdh(pi / 2 + q[0], 0, 0, 0) * mdh(-1.444 + q[1], -pi / 2, 0, 0) *
           mdh(-1.444 + q[2], pi, a2, 0) * mdh(pi / 2 + q[3], 0, a3, 0) *
           mdh(q[4], pi / 2, 0, tool);
  }
  Eigen::Matrix4d forward(const Positions & p) const
  {
    Angles q;
    for (size_t i = 0; i < 5; ++i) {q[i] = ticks_to_radians(p[i]);}
    return forward(q);
  }

  // Enumerate base/elbow branches, verify the complete pose and choose the
  // nearest valid solution. A 5 DOF arm cannot realise every 6 DOF pose.
  std::optional<Positions> inverse(
    const Eigen::Matrix4d & target, const Positions & previous,
    const Positions & lower, const Positions & upper) const
  {
    if (!target.allFinite()) {return std::nullopt;}
    Eigen::Matrix4d wrist = target;
    wrist.block<3, 1>(0, 3) -= tool * target.block<3, 1>(0, 2);
    const double x = wrist(0, 3), y = wrist(1, 3), z = wrist(2, 3);
    const double c3 = (x*x + y*y + z*z - a2*a2 - a3*a3) / (2*a2*a3);
    if (std::abs(c3) > 1.0 + 1e-9) {return std::nullopt;}
    std::optional<Positions> best;
    double best_cost = std::numeric_limits<double>::infinity();
    const double base = std::hypot(x, y) < 1e-8 ?
      pi / 2 + ticks_to_radians(previous[0]) : std::atan2(y, x);
    for (double t1 : {base, base + pi}) {
      const double radial = std::cos(t1)*x + std::sin(t1)*y;
      for (double sign : {-1.0, 1.0}) {
        const double t3 = sign * std::acos(std::clamp(c3, -1.0, 1.0));
        const double t2 = std::atan2(a3 * std::sin(t3), a2 + a3 * std::cos(t3)) -
          std::atan2(z, radial);
        const Eigen::Matrix4d t03 = mdh(t1, 0, 0, 0) *
          mdh(t2, -pi / 2, 0, 0) * mdh(t3, pi, a2, 0);
        const Eigen::Matrix3d r = t03.block<3, 3>(0, 0).transpose() *
          target.block<3, 3>(0, 0);
        Angles q;
        q << t1 - pi / 2, t2 + 1.444, t3 + 1.444,
          std::atan2(r(0, 2), -r(1, 2)) - pi / 2, std::atan2(r(2, 0), r(2, 1));
        for (int i = 0; i < 5; ++i) {q[i] = std::remainder(q[i], 2 * pi);}
        if ((forward(q) - target).norm() > 1e-5) {continue;}
        Positions p;
        double cost = 0;
        bool valid = true;
        for (size_t i = 0; i < 5; ++i) {
          p[i] = radians_to_ticks(q[i]);
          valid = valid && p[i] >= lower[i] && p[i] <= upper[i];
          cost += std::pow(static_cast<double>(p[i]) - previous[i], 2);
        }
        if (valid && cost < best_cost) {best = p; best_cost = cost;}
      }
    }
    return best;
  }
};
}  // namespace five_dof_arm
