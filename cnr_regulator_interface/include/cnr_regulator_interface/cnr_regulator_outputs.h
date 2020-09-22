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
  RegulatorOutputBase() = delete;
  virtual ~RegulatorOutputBase() = default;
  RegulatorOutputBase(const RegulatorOutputBase&) = delete;
  RegulatorOutputBase& operator=(const RegulatorOutputBase&) = delete;
  RegulatorOutputBase(RegulatorOutputBase&&) = delete;
  RegulatorOutputBase& operator=(RegulatorOutputBase&&) = delete;

  RegulatorOutputBase(const size_t& sz) : dof(sz)
  {
    u.resize(dof);
    u.setZero();
  }
  size_t dof;
  Eigen::VectorXd u;
};

typedef std::shared_ptr<RegulatorOutputBase> RegulatorOutputBasePtr;
typedef const std::shared_ptr<RegulatorOutputBase const > RegulatorOutputBaseConstPtr;

/**
 * @brief The RegulatorOutputBase struct
 */
struct JointRegulatorOutput : public RegulatorOutputBase
{
  JointRegulatorOutput() = delete;
  virtual ~JointRegulatorOutput() = default;
  JointRegulatorOutput(const JointRegulatorOutput&) = delete;
  JointRegulatorOutput& operator=(const JointRegulatorOutput&) = delete;
  JointRegulatorOutput(JointRegulatorOutput&&) = delete;
  JointRegulatorOutput& operator=(JointRegulatorOutput&&) = delete;

  JointRegulatorOutput(const size_t& nAx)
    : RegulatorOutputBase(4 + nAx*4)
  {
    dof = nAx;
  }
  virtual void set_time_from_start  (const ros::Duration& time_from_start) { u(0)= time_from_start.toSec()  ; }
  virtual void set_in_goal_tolerance(const bool& in_goal_tolerance       ) { u(1)= in_goal_tolerance        ; }
  virtual void set_in_path_tolerance(const bool& in_path_tolerance       ) { u(2)= in_path_tolerance        ; }
  virtual void set_scaling          (const double& scaling               ) { u(3)= scaling                  ; }
  virtual void set_q                (const Eigen::VectorXd& q            ) { u.segment(0*dof+4,dof) = q     ; }
  virtual void set_qd               (const Eigen::VectorXd& qd           ) { u.segment(1*dof+4,dof) = qd    ; }
  virtual void set_qdd              (const Eigen::VectorXd& qdd          ) { u.segment(2*dof+4,dof) = qdd   ; }
  virtual void set_effort           (const Eigen::VectorXd& effort       ) { u.segment(3*dof+4,dof) = effort; }

  virtual void set_q(const std::vector<double>& q)
  {
    u.segment(0*dof+4,dof) = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(&q[0],dof);
  }
  virtual void set_qd(const std::vector<double>& qd)
  {
    u.segment(1*dof+4,dof) = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(&qd[0],dof);
  }
  virtual void set_qdd(const std::vector<double>& qdd)
  {
    u.segment(2*dof+4,dof) = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(&qdd[0],dof);
  }
  virtual void set_effort(const std::vector<double>& effort)
  {
    u.segment(3*dof+4,dof) = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(&effort[0],dof);
  }

  virtual ros::Duration   get_time_from_start  ( ) const { return ros::Duration(u(0))  ;  }
  virtual bool            get_in_goal_tolerance( ) const { return u(1); }
  virtual bool            get_in_path_tolerance( ) const { return u(2); }
  virtual double          get_scaling          ( ) const { return u(3); }
  virtual Eigen::VectorXd get_q                ( ) const { return u.segment(0*dof+4,dof); }
  virtual Eigen::VectorXd get_qd               ( ) const { return u.segment(1*dof+4,dof); }
  virtual Eigen::VectorXd get_qdd              ( ) const { return u.segment(2*dof+4,dof); }
  virtual Eigen::VectorXd get_effort           ( ) const { return u.segment(3*dof+4,dof); }
};

typedef std::shared_ptr<JointRegulatorOutput> JointRegulatorOutputPtr;
typedef const std::shared_ptr<JointRegulatorOutput const> JointRegulatorOutputConstPtr;

/**
 * @brief The CartesianRegulatorOutput struct
 */
struct CartesianRegulatorOutput : public cnr_regulator_interface::JointRegulatorOutput
{
  CartesianRegulatorOutput(const size_t ndof = 6)
    : JointRegulatorOutput(ndof)
  {
  }
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
