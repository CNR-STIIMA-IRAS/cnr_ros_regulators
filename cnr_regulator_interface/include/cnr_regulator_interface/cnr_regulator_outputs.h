#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_OUTPUTS__H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_OUTPUTS__H

#include <ros/time.h>
#include <eigen3/Eigen/Dense>
#include <memory>

namespace cnr_regulator_interface
{


/**
 * @brief The BaseRegulatorOutput struct
 */
struct BaseRegulatorOutput
{
  BaseRegulatorOutput() = delete;
  virtual ~BaseRegulatorOutput() = default;
  BaseRegulatorOutput(const BaseRegulatorOutput&) = delete;
  BaseRegulatorOutput& operator=(const BaseRegulatorOutput&) = delete;
  BaseRegulatorOutput(BaseRegulatorOutput&&) = delete;
  BaseRegulatorOutput& operator=(BaseRegulatorOutput&&) = delete;

  BaseRegulatorOutput(const size_t& sz) : dim(sz)
  {
    u.resize(dim);
    u.setZero();
  }
  
  void set_dimension(const size_t& n_dim) { dim = n_dim; u.resize(6 + n_dim); u.setZero();}
  
  size_t dim;
  Eigen::VectorXd u;
};

typedef std::shared_ptr<BaseRegulatorOutput> BaseRegulatorOutputPtr;
typedef const std::shared_ptr<BaseRegulatorOutput const > BaseRegulatorOutputConstPtr;

/**
 * @brief The BaseRegulatorOutput struct
 */
struct JointRegulatorOutput : public BaseRegulatorOutput
{
  JointRegulatorOutput() = delete;
  virtual ~JointRegulatorOutput() = default;
  JointRegulatorOutput(const JointRegulatorOutput&) = delete;
  JointRegulatorOutput& operator=(const JointRegulatorOutput&) = delete;
  JointRegulatorOutput(JointRegulatorOutput&&) = delete;
  JointRegulatorOutput& operator=(JointRegulatorOutput&&) = delete;

  JointRegulatorOutput(const size_t& nAx)
    : BaseRegulatorOutput(4 + nAx*4)
  {
    dim = nAx;
  }
  virtual void set_time_from_start  (const ros::Duration& time_from_start) { u(0)= time_from_start.toSec()  ; }
  virtual void set_in_goal_tolerance(const bool& in_goal_tolerance       ) { u(1)= in_goal_tolerance        ; }
  virtual void set_in_path_tolerance(const bool& in_path_tolerance       ) { u(2)= in_path_tolerance        ; }
  virtual void set_scaling          (const double& scaling               ) { u(3)= scaling                  ; }
  virtual void set_q                (const Eigen::VectorXd& q            ) { u.segment(0*dim+4,dim) = q     ; }
  virtual void set_qd               (const Eigen::VectorXd& qd           ) { u.segment(1*dim+4,dim) = qd    ; }
  virtual void set_qdd              (const Eigen::VectorXd& qdd          ) { u.segment(2*dim+4,dim) = qdd   ; }
  virtual void set_effort           (const Eigen::VectorXd& effort       ) { u.segment(3*dim+4,dim) = effort; }

  virtual void set_q(const std::vector<double>& q)
  {
    u.segment(0*dim+4,dim) = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(&q[0],dim);
  }
  virtual void set_qd(const std::vector<double>& qd)
  {
    u.segment(1*dim+4,dim) = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(&qd[0],dim);
  }
  virtual void set_qdd(const std::vector<double>& qdd)
  {
    u.segment(2*dim+4,dim) = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(&qdd[0],dim);
  }
  virtual void set_effort(const std::vector<double>& effort)
  {
    u.segment(3*dim+4,dim) = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(&effort[0],dim);
  }

  virtual ros::Duration   get_time_from_start  ( ) const { return ros::Duration(u(0))  ;  }
  virtual bool            get_in_goal_tolerance( ) const { return u(1); }
  virtual bool            get_in_path_tolerance( ) const { return u(2); }
  virtual double          get_scaling          ( ) const { return u(3); }
  virtual Eigen::VectorXd get_q                ( ) const { return u.segment(0*dim+4,dim); }
  virtual Eigen::VectorXd get_qd               ( ) const { return u.segment(1*dim+4,dim); }
  virtual Eigen::VectorXd get_qdd              ( ) const { return u.segment(2*dim+4,dim); }
  virtual Eigen::VectorXd get_effort           ( ) const { return u.segment(3*dim+4,dim); }
};

typedef std::shared_ptr<JointRegulatorOutput> JointRegulatorOutputPtr;
typedef const std::shared_ptr<JointRegulatorOutput const> JointRegulatorOutputConstPtr;

/**
 * @brief The CartesianRegulatorOutput struct
 */
struct CartesianRegulatorOutput : public cnr_regulator_interface::JointRegulatorOutput
{
  CartesianRegulatorOutput(const size_t ndim = 6)
    : JointRegulatorOutput(ndim)
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
