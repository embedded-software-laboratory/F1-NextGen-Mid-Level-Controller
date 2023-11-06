#include "mid_level_controller/TrajectoryInterpolation.hpp"
#include <cmath>

namespace mid_level_controller
{
auto TrajectoryInterpolation::interpolate(lab_msgs::msg::TrajectoryPoint start, lab_msgs::msg::TrajectoryPoint end,
                                          const rclcpp::Time& now) -> InterPolatedTrajectoryPoint
{
  const double start_time = rclcpp::Time(start.t.nanoseconds, RCL_ROS_TIME).seconds();
  const double end_time = rclcpp::Time(end.t.nanoseconds, RCL_ROS_TIME).seconds();
  const double current_time = now.seconds();

  const double delta_t = end_time - start_time;
  const double tau = (current_time - start_time) / delta_t;

  const double tau2 = tau * tau;
  const double tau3 = tau * tau2;

  const double position_start_x = start.px;
  const double position_start_y = start.py;
  const double position_end_x = end.px;
  const double position_end_y = end.py;

  const double velocity_start_x = start.vx * delta_t;
  const double velocity_start_y = start.vy * delta_t;
  const double velocity_end_x = end.vx * delta_t;
  const double velocity_end_y = end.vy * delta_t;

  // Hermite spline coefficients
  const double p0 = 2 * tau3 - 3 * tau2 + 1;
  const double m0 = tau3 - 2 * tau2 + tau;
  const double p1 = -2 * tau3 + 3 * tau2;
  const double m1 = tau3 - tau2;

  // Hermite spline derivative coefficients
  const double dp0 = 6 * tau2 - 6 * tau;
  const double dm0 = 3 * tau2 - 4 * tau + 1;
  const double dp1 = -6 * tau2 + 6 * tau;
  const double dm1 = 3 * tau2 - 2 * tau;

  // Hermite spline second derivative coefficients
  const double ddp0 = 12 * tau - 6;
  const double ddm0 = 6 * tau - 4;
  const double ddp1 = -12 * tau + 6;
  const double ddm1 = 6 * tau - 2;

  InterPolatedTrajectoryPoint interpolation;
  interpolation.position_x = position_start_x * p0 + velocity_start_x * m0 + position_end_x * p1 + velocity_end_x * m1;
  interpolation.position_y = position_start_y * p0 + velocity_start_y * m0 + position_end_y * p1 + velocity_end_y * m1;
  interpolation.velocity_x =
      (position_start_x * dp0 + velocity_start_x * dm0 + position_end_x * dp1 + velocity_end_x * dm1) / delta_t;
  interpolation.velocity_y =
      (position_start_y * dp0 + velocity_start_y * dm0 + position_end_y * dp1 + velocity_end_y * dm1) / delta_t;
  interpolation.acceleration_x =
      (position_start_x * ddp0 + velocity_start_x * ddm0 + position_end_x * ddp1 + velocity_end_x * ddm1) /
      (delta_t * delta_t);
  interpolation.acceleration_y =
      (position_start_y * ddp0 + velocity_start_y * ddm0 + position_end_y * ddp1 + velocity_end_y * ddm1) /
      (delta_t * delta_t);

  interpolation.yaw = atan2(interpolation.velocity_y, interpolation.velocity_x);
  interpolation.speed =
      sqrt(interpolation.velocity_x * interpolation.velocity_x + interpolation.velocity_y * interpolation.velocity_y);
  interpolation.curvature = (interpolation.velocity_x * interpolation.acceleration_y -
                             interpolation.velocity_y * interpolation.acceleration_x) /
                            (interpolation.speed * interpolation.speed * interpolation.speed);
  return interpolation;
}
}  // namespace mid_level_controller