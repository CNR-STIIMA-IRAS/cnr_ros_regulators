#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INPUTS__H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INPUTS__H

#include <ros/time.h>
#include <eigen3/Eigen/Dense>
#include <memory>

namespace cnr_regulator_interface
{


/**
 * @brief The RegulatorInputBase struct
 */
struct RegulatorInputBase
{
  RegulatorInputBase() = default;
  virtual ~RegulatorInputBase() = default;
  RegulatorInputBase(const RegulatorInputBase&) = delete;
  RegulatorInputBase& operator=(const RegulatorInputBase&) = delete;
  RegulatorInputBase(RegulatorInputBase&&) = delete;
  RegulatorInputBase& operator=(RegulatorInputBase&&) = delete;

  ros::Duration    period;
  ros::Duration    scaled_time;
  double           global_override;
  ros::Duration    time_from_start_ref;
};

typedef std::shared_ptr<RegulatorInputBase> RegulatorInputBasePtr;
typedef const std::shared_ptr<RegulatorInputBase const> RegulatorInputBaseConstPtr;

struct JointRegulatorInput : public cnr_regulator_interface::RegulatorInputBase
{
  JointRegulatorInput() = default;
  virtual ~JointRegulatorInput() = default;
  JointRegulatorInput(const JointRegulatorInput&) = delete;
  JointRegulatorInput& operator=(const JointRegulatorInput&) = delete;
  JointRegulatorInput(JointRegulatorInput&&) = delete;
  JointRegulatorInput& operator=(JointRegulatorInput&&) = delete;

  Eigen::VectorXd  q_setpoint;
  Eigen::VectorXd  qd_setpoint;
  Eigen::VectorXd  qdd_setpoint;
  Eigen::VectorXd  effort_setpoint;
};

typedef std::shared_ptr<JointRegulatorInput> JointRegulatorInputPtr;
typedef const std::shared_ptr<JointRegulatorInput const> JointRegulatorInputConstPtr;


/**
 * @brief The CartesianRegulatorInput struct
 */
struct CartesianRegulatorInput : public cnr_regulator_interface::RegulatorInputBase
{
  CartesianRegulatorInput() = default;
  virtual ~CartesianRegulatorInput() = default;
  CartesianRegulatorInput(const CartesianRegulatorInput&) = delete;
  CartesianRegulatorInput& operator=(const CartesianRegulatorInput&) = delete;
  CartesianRegulatorInput(CartesianRegulatorInput&&) = delete;
  CartesianRegulatorInput& operator=(CartesianRegulatorInput&&) = delete;

  Eigen::Affine3d x_setpoint;
  Eigen::Vector6d twist_setpoint;
  Eigen::Vector6d twistd_setpoint;
};
typedef std::shared_ptr<CartesianRegulatorInput> CartesianRegulatorInputPtr;
typedef const std::shared_ptr<CartesianRegulatorInput const> CartesianRegulatorInputConstPtr;


}  // namespace cnr_regulator_interface

#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INPUTS__H
