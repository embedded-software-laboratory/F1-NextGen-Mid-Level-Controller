#include "mid_level_controller/Controller.hpp"
#include "mid_level_controller/PathTrackingController.hpp"

#include <iostream>
#include <functional>
#include <utility>

#include <rclcpp/rclcpp.hpp>

namespace mid_level_controller
{

Controller::Controller() : mpcController()
{
}

double Controller::speedController(const double speed_measured, const double speed_target)
{
  // steady-state curve, from curve fitting
  double motor_throttle = ((speed_target >= 0) ? (1.0) : (-1.0)) * pow(fabs(0.152744 * speed_target), (0.627910));

  const double speed_error = speed_target - speed_measured;

  // PI controller for the speed
  const double integral_gain = 0.01;
  const double proportional_gain = 0.3;
  this->speed_throttle_error_integral += integral_gain * speed_error;

  this->speed_throttle_error_integral = fmin(0.5, fmax(-0.5,
                                                       this->speed_throttle_error_integral));  // integral clamping

  motor_throttle += this->speed_throttle_error_integral;
  motor_throttle += proportional_gain * speed_error;
  return motor_throttle;
}

void Controller::updateVehicleState(lab_msgs::msg::VehicleState vehicleState)
{
  this->m_vehicleState = vehicleState;
}

double steering_curvature_calibration(double curvature)
{
  // steady-state curve, from curve fitting
  double steering_servo = (0.241857) * curvature;
  return steering_servo;
}

std::shared_ptr<InterPolatedTrajectoryPoint> Controller::interpolateTrajectory(const rclcpp::Time& t_now) const
{
  auto trajectory_points = m_vehicleCommandTrajectory.trajectory_points;
  if (trajectory_points.size() < 2)
    return nullptr;

  // m_vehicleCommandTrajectory is updated in receive_commands, which gets called in get_control_signals
  // The reason for this confusing structure is that it was most compatible to the already existing solution for the
  // other data types
  if (rclcpp::Time(m_vehicleCommandTrajectory.timings.create_stamp.nanoseconds, RCL_ROS_TIME).seconds() <= 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("Controller"),
                "Warning: Controller: Trajectory interpolation error: No valid trajectory data.");
    return nullptr;
  }

  // Get current segment (trajectory points) in current trajectory for interpolation
  auto start_point = lab_msgs::msg::TrajectoryPoint();
  start_point.t.nanoseconds = 0;
  auto end_point = lab_msgs::msg::TrajectoryPoint();
  end_point.t.nanoseconds = 0;

  // When looking up the current segment, start at 1, because start and end must follow each other (we look up end, and
  // from that determine start)
  for (size_t i = 1; i < trajectory_points.size(); ++i)
  {
    if (rclcpp::Time(trajectory_points.at(i).t.nanoseconds, RCL_ROS_TIME) >= t_now)
    {
      start_point = trajectory_points.at(i);
      end_point = trajectory_points.at(i - 1);
      break;
    }
  }

  // Log an error if we could not find a valid trajectory segment w.r.t. start
  if (rclcpp::Time(start_point.t.nanoseconds, RCL_ROS_TIME) >= t_now)
  {
    RCLCPP_WARN(rclcpp::get_logger("Controller"),
                "Missing trajectory point in the past. Current time is %llu ns. First trajectory point time is "
                "%llu ns.",
                t_now.nanoseconds(), rclcpp::Time(start_point.t.nanoseconds, RCL_ROS_TIME).nanoseconds());
    return nullptr;
  }

  // Log an error if we could not find a valid trajectory segment w.r.t. end
  if (end_point.t.nanoseconds == 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("Controller"),
                "Warning: Controller: Trajectory interpolation error: Missing trajectory point in the future.");
    return nullptr;
  }

  assert(t_now >= rclcpp::Time(start_point.t.nanoseconds, RCL_ROS_TIME));
  assert(t_now <= rclcpp::Time(end_point.t.nanoseconds, RCL_ROS_TIME));

  // We have a valid trajectory segment.
  // Interpolate for the current time.
  return std::make_shared<InterPolatedTrajectoryPoint>(
      TrajectoryInterpolation::interpolate(start_point, end_point, t_now));
}

static inline double square(double x)
{
  return x * x;
}

void Controller::trajectoryTrackingStatisticsUpdate(const rclcpp::Time& t_now)
{
  std::shared_ptr<InterPolatedTrajectoryPoint> trajectory_interpolation = interpolateTrajectory(t_now);

  if (!trajectory_interpolation)
  {
    return;
  }

  const double x_ref = trajectory_interpolation->position_x;
  const double y_ref = trajectory_interpolation->position_y;
  const double yaw_ref = trajectory_interpolation->yaw;

  const double x = m_vehicleState.pose.x;
  const double y = m_vehicleState.pose.y;

  double longitudinal_error = cos(yaw_ref) * (x - x_ref) + sin(yaw_ref) * (y - y_ref);
  double lateral_error = -sin(yaw_ref) * (x - x_ref) + cos(yaw_ref) * (y - y_ref);

  trajectory_tracking_statistics_longitudinal_errors[trajectory_tracking_statistics_index] = longitudinal_error;
  trajectory_tracking_statistics_lateral_errors[trajectory_tracking_statistics_index] = lateral_error;

  trajectory_tracking_statistics_index =
      (trajectory_tracking_statistics_index + 1) % TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE;

  if (trajectory_tracking_statistics_index == 0)
  {
    // output summary

    double longitudinal_error_sum = 0;
    double lateral_error_sum = 0;
    double longitudinal_error_max = 0;
    double lateral_error_max = 0;

    for (int i = 0; i < TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE; ++i)
    {
      longitudinal_error_sum += trajectory_tracking_statistics_longitudinal_errors[i];
      lateral_error_sum += trajectory_tracking_statistics_lateral_errors[i];

      longitudinal_error_max =
          fmax(longitudinal_error_max, fabs(trajectory_tracking_statistics_longitudinal_errors[i]));
      lateral_error_max = fmax(lateral_error_max, fabs(trajectory_tracking_statistics_lateral_errors[i]));
    }

    const double longitudinal_error_mean = longitudinal_error_sum / TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE;
    const double lateral_error_mean = lateral_error_sum / TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE;

    double longitudinal_error_variance_sum = 0;
    double lateral_error_variance_sum = 0;

    for (int i = 0; i < TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE; ++i)
    {
      longitudinal_error_variance_sum +=
          square(trajectory_tracking_statistics_longitudinal_errors[i] - longitudinal_error_mean);
      lateral_error_variance_sum += square(trajectory_tracking_statistics_lateral_errors[i] - lateral_error_mean);
    }

    const double longitudinal_error_variance =
        longitudinal_error_variance_sum / (TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE - 1);
    const double lateral_error_variance = lateral_error_variance_sum / (TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE - 1);

    const double longitudinal_error_std = sqrt(longitudinal_error_variance);
    const double lateral_error_std = sqrt(lateral_error_variance);

    RCLCPP_ERROR(rclcpp::get_logger("Controller"),
                 "Vehicle Controller Tracking Errors:"
                 "long,mean: %f  "
                 "long,std: %f  "
                 "long,max: %f  "
                 "lat,mean: %f  "
                 "lat,std: %f  "
                 "lat,max: %f  ",
                 longitudinal_error_mean, longitudinal_error_std, longitudinal_error_max, lateral_error_mean,
                 lateral_error_std, lateral_error_max);
  }
}

void Controller::getControlSignals(const rclcpp::Time& t_now, double& out_motor_throttle, double& out_steering_servo)
{
  double motor_throttle = 0;
  double steering_servo = 0;

  // auto ips_deviation =
  //     rclcpp::Time(t_now, RCL_ROS_TIME) - rclcpp::Time(this->m_vehicleState.ips_update_age_nanoseconds,
  //     RCL_ROS_TIME);
  // if (ips_deviation.seconds() > 3 && state == ControllerState::Trajectory)
  // {
  //   // Use %s, else we get a warning that this is no string literal (we do not want unnecessary warnings to show up)
  //   RCLCPP_ERROR(rclcpp::get_logger("Controller"),
  //                "Lost IPS position reference. Deviation is %.2f seconds. IPS stamp is %.2f seconds. Current Time is
  //                "
  //                "%.2f seconds.",
  //                ips_deviation.seconds(),
  //                rclcpp::Time(this->m_vehicleState.ips_update_age_nanoseconds, RCL_ROS_TIME).seconds(),
  //                t_now.seconds());

  //   state = ControllerState::Stop;
  // }

  switch (state)
  {
    case ControllerState::Stop: {
      // Use function that calculates motor values for stopping immediately - which is also already used in main
      getStopSignals(motor_throttle, steering_servo);
    }
    break;

    case ControllerState::Direct: {
      motor_throttle = m_vehicleCommandDirect.motor_throttle;
      steering_servo = m_vehicleCommandDirect.steering_servo;
    }
    break;

    case ControllerState::SpeedCurvature: {
      const double speed_target = m_vehicleCommandSpeedCurvature.speed;
      const double curvature = m_vehicleCommandSpeedCurvature.curvature;
      const double speed_measured = m_vehicleState.speed;

      steering_servo = steering_curvature_calibration(curvature);
      motor_throttle = speedController(speed_measured, speed_target);
    }
    break;

    case ControllerState::Trajectory: {
      trajectoryTrackingStatisticsUpdate(t_now);

      // Run controller
      mpcController.update(t_now, m_vehicleState, m_vehicleCommandTrajectory, motor_throttle, steering_servo);
    }
    break;

    case ControllerState::PathTracking: {
      // Speed: PID
      const double speed_target = m_vehicleCommandPathTracking.speed;
      const double speed_measured = m_vehicleState.speed;
      motor_throttle = speedController(speed_measured, speed_target);

      // Steering: Stanley
      steering_servo = mid_level_controller::PathTrackingController::control_steering_servo(
          m_vehicleState, m_vehicleCommandPathTracking);
    }
    break;
  }

  motor_throttle = fmax(-1.0, fmin(1.0, motor_throttle));
  steering_servo = fmax(-1.0, fmin(1.0, steering_servo));

  out_motor_throttle = motor_throttle;
  out_steering_servo = steering_servo;
}

void Controller::getStopSignals(double& out_motor_throttle, double& out_steering_servo) const
{
  // Init. values
  double steering_servo = 0;
  double speed_target = 0;
  const double speed_measured = this->m_vehicleState.speed;

  // P controller to reach 0 speed (without "backshooting" like with the PI controller in speed controller)
  double motor_throttle = ((speed_target >= 0) ? (1.0) : (-1.0)) * pow(fabs(0.152744 * speed_target), (0.627910));
  const double speed_error = speed_target - speed_measured;
  const double proportional_gain =
      0.5;  // We tested, and this seems to be an acceptable value (at least for central routing)
  motor_throttle += proportional_gain * speed_error;

  motor_throttle = fmax(-1.0, fmin(1.0, motor_throttle));
  steering_servo = fmax(-1.0, fmin(1.0, steering_servo));

  out_motor_throttle = motor_throttle;
  out_steering_servo = steering_servo;
}

void Controller::reset()
{
  // Set current state to stop until new commands are received
  state = ControllerState::Stop;
}

void Controller::updateDirectCommand(lab_msgs::msg::VehicleCommandDirect msg)
{
  this->m_vehicleCommandDirect = msg;
  this->state = ControllerState::Direct;
}

void Controller::updateSpeedCurvatureCommand(lab_msgs::msg::VehicleCommandSpeedCurvature msg)
{
  this->m_vehicleCommandSpeedCurvature = msg;
  this->state = ControllerState::SpeedCurvature;
}

void Controller::updateTrajectoryCommand(lab_msgs::msg::VehicleCommandTrajectory msg)
{
  this->m_vehicleCommandTrajectory = std::move(msg);
  this->state = ControllerState::Trajectory;
}

void Controller::updatePathTrackingCommand(lab_msgs::msg::VehicleCommandPathTracking msg)
{
  this->m_vehicleCommandPathTracking = std::move(msg);
  this->state = ControllerState::PathTracking;
}
}  // namespace mid_level_controller