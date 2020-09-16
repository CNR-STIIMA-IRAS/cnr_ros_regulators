#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_OUTPUTS__H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_OUTPUTS__H

#include <ros/time.h>
#include <eigen3/Eigen/Dense>
#include <memory>

namespace cnr_regulator_interface
{


/**
 * @brief The RegulatorOutputBase struct
 */
struct RegulatorOutputBase
{
  RegulatorOutputBase() = default;
  virtual ~RegulatorOutputBase() = default;
  RegulatorOutputBase(const RegulatorOutputBase&) = delete;
  RegulatorOutputBase& operator=(const RegulatorOutputBase&) = delete;
  RegulatorOutputBase(RegulatorOutputBase&&) = delete;
  RegulatorOutputBase& operator=(RegulatorOutputBase&&) = delete;

  void setOutputDimension(const size_t& sz)
  {
    u.resize(sz);
    u.setZero();
  }
  Eigen::VectorXd u;
};

typedef std::shared_ptr<RegulatorOutputBase> RegulatorOutputBasePtr;
typedef const std::shared_ptr<RegulatorOutputBase const > RegulatorOutputBaseConstPtr;

/**
 * @brief The RegulatorOutputBase struct
 */
struct JointRegulatorOutput : public RegulatorOutputBase
{
  JointRegulatorOutput() = default;
  virtual ~JointRegulatorOutput() = default;
  JointRegulatorOutput(const JointRegulatorOutput&) = delete;
  JointRegulatorOutput& operator=(const JointRegulatorOutput&) = delete;
  JointRegulatorOutput(JointRegulatorOutput&&) = delete;
  JointRegulatorOutput& operator=(JointRegulatorOutput&&) = delete;

  size_t dof;
  void set_dof(const size_t& nAx)
  {
    dof = nAx;
    u.resize( dof * 4 + 1 );
  }
  void set_time_from_start( const ros::Duration& time_from_start) {  u(0)= time_from_start.toSec(); }
  void set_q     (const Eigen::VectorXd& q     ) { u.segment(0*dof+1,dof)= q     ; }
  void set_qd    (const Eigen::VectorXd& qd    ) { u.segment(1*dof+1,dof)= qd    ; }
  void set_qdd   (const Eigen::VectorXd& qdd   ) { u.segment(2*dof+1,dof)= qdd   ; }
  void set_effort(const Eigen::VectorXd& effort) { u.segment(3*dof+1,dof)= effort; }

  ros::Duration   get_time_from_start( ) const { return ros::Duration(u(0))  ;  }
  Eigen::VectorXd get_q     ( ) const { return u.segment(0*dof+1,dof); }
  Eigen::VectorXd get_qd    ( ) const { return u.segment(1*dof+1,dof); }
  Eigen::VectorXd get_qdd   ( ) const { return u.segment(2*dof+1,dof); }
  Eigen::VectorXd get_effort( ) const { return u.segment(3*dof+1,dof); }

  bool   in_tolerance = false;
  bool   in_path_tolerance = false;
  double scaling = 1.0;
};

typedef std::shared_ptr<JointRegulatorOutput> JointRegulatorOutputPtr;
typedef const std::shared_ptr<JointRegulatorOutput const> JointRegulatorOutputConstPtr;

/**
 * @brief The CartesianRegulatorOutput struct
 */
struct CartesianRegulatorOutput : public cnr_regulator_interface::JointRegulatorOutput
{
  CartesianRegulatorOutput() = default;
  virtual ~CartesianRegulatorOutput() = default;
  CartesianRegulatorOutput(const CartesianRegulatorOutput&) = delete;
  CartesianRegulatorOutput& operator=(const CartesianRegulatorOutput&) = delete;
  CartesianRegulatorOutput(CartesianRegulatorOutput&&) = delete;
  CartesianRegulatorOutput& operator=(CartesianRegulatorOutput&&) = delete;
};
typedef std::shared_ptr<CartesianRegulatorOutput> CartesianRegulatorOutputPtr;
typedef const std::shared_ptr<CartesianRegulatorOutput const> CartesianRegulatorOutputConstPtr;


}  // namespace cnr_regulator_interface

#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_OUTPUTS__H
