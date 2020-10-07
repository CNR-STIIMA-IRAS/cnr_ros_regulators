#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INPUTS__H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INPUTS__H

#include <ros/time.h>
#include <eigen3/Eigen/Dense>
#include <rosdyn_core/spacevect_algebra.h>
#include <memory>


namespace cnr_regulator_interface
{


/**
 * @brief The BaseRegulatorInput struct
 */
struct BaseRegulatorInput
{
  
  typedef std::shared_ptr<BaseRegulatorInput> Ptr;
  typedef const std::shared_ptr<BaseRegulatorInput const> ConstPtr;
  
  BaseRegulatorInput() = default;
  virtual ~BaseRegulatorInput() = default;
  BaseRegulatorInput(const BaseRegulatorInput&) = delete;
  BaseRegulatorInput(BaseRegulatorInput&&) = delete;
  BaseRegulatorInput& operator=(BaseRegulatorInput&&) = delete;

  BaseRegulatorInput(size_t n_dim)
    : dim(n_dim){u.resize(6 + n_dim); u.setZero();}
  size_t           dim;
  Eigen::VectorXd  u;

  void set_dimension      (const size_t&   n_dim)                { dim = n_dim; u.resize(6 + n_dim); u.setZero();}
  void set_period         (const ros::Duration& period         ) { u(0) = period.toSec();         }
  void set_target_override(const double& global_override       ) { u(2) = global_override;        }
  void set_time_from_start(const ros::Duration& time_from_start) { u(3) = time_from_start.toSec();}
  void set_goal_tolerance (const double& goal_tolerance        ) { u(4) = goal_tolerance;         }
  void set_path_tolerance (const double& path_tolerance        ) { u(5) = path_tolerance;         }
  void set_u              (const Eigen::VectorXd& input_values )
  {
    assert(input_values.rows()==dim);
    u.segment(6,dim)=input_values;
  }

  ros::Duration   get_period         ( ) const { return ros::Duration(u(0));  }
  double          get_target_override( ) const { return u(2);  }
  ros::Duration   get_time_from_start( ) const { return ros::Duration(u(3));  }
  double          get_goal_tolerance ( ) const { return u(4); }
  double          get_path_tolerance ( ) const { return u(5); }
  Eigen::VectorXd get_u              ( ) const { return u.segment(6,dim); }
  
  BaseRegulatorInput& operator=(const BaseRegulatorInput& rhs)
  {
    this->dim = rhs.dim;
    this->u   = rhs.u;
    return *this;
  }
  

};

typedef BaseRegulatorInput::Ptr BaseRegulatorInputPtr;
typedef BaseRegulatorInput::ConstPtr BaseRegulatorInputConstPtr;

struct JointRegulatorInput : public cnr_regulator_interface::BaseRegulatorInput
{
  
  typedef std::shared_ptr<JointRegulatorInput> Ptr;
  typedef const std::shared_ptr<JointRegulatorInput const> ConstPtr;

  JointRegulatorInput() = default;
  virtual ~JointRegulatorInput() = default;
  JointRegulatorInput(const JointRegulatorInput&) = delete;
  JointRegulatorInput(JointRegulatorInput&&) = delete;
  JointRegulatorInput& operator=(JointRegulatorInput&&) = delete;

  JointRegulatorInput(size_t nAx): BaseRegulatorInput( (nAx*4) ){ dim = nAx; }

  void set_dof       (const size_t& n_dof          ) { set_dimension(n_dof * 4);                    }
  void set_q         (const Eigen::VectorXd& q     ) { u.segment(6+0*dim,dim) = q     ; }
  void set_qd        (const Eigen::VectorXd& qd    ) { u.segment(6+1*dim,dim) = qd    ; }
  void set_qdd       (const Eigen::VectorXd& qdd   ) { u.segment(6+2*dim,dim) = qdd   ; }
  void set_effort    (const Eigen::VectorXd& effort) { u.segment(6+3*dim,dim) = effort; }

  Eigen::VectorXd get_q     ( ) const { return u.segment(6+0*dim,dim); }
  Eigen::VectorXd get_qd    ( ) const { return u.segment(6+1*dim,dim); }
  Eigen::VectorXd get_qdd   ( ) const { return u.segment(6+2*dim,dim); }
  Eigen::VectorXd get_effort( ) const { return u.segment(6+3*dim,dim); }
  
  JointRegulatorInput& operator=(const JointRegulatorInput& rhs)
  {
    this->dim = rhs.dim;
    this->u   = rhs.u;
    return *this;
  }

};

typedef JointRegulatorInput::Ptr JointRegulatorInputPtr;
typedef JointRegulatorInput::ConstPtr JointRegulatorInputConstPtr;


/**
 * @brief The CartesianRegulatorInput struct
 */
struct CartesianRegulatorInput : public cnr_regulator_interface::BaseRegulatorInput
{
  CartesianRegulatorInput( ): BaseRegulatorInput( (7 + 6 + 6) ){}

  virtual ~CartesianRegulatorInput() = default;
  CartesianRegulatorInput(const CartesianRegulatorInput&) = delete;
  CartesianRegulatorInput(CartesianRegulatorInput&&) = delete;
  CartesianRegulatorInput& operator=(CartesianRegulatorInput&&) = delete;

  
  void set_x(const Eigen::Affine3d& x)
  {
    Eigen::Quaterniond q(x.linear());
    u.segment(6+0,3)= x.translation();
    u(6+3) = q.w();
    u(6+4) = q.x();
    u(6+5) = q.y();
    u(6+6) = q.z();
  }
  void set_twist         (const Eigen::Vector6d& twist ) { u.segment(6+ 7,6) = twist ; }
  void set_twistd        (const Eigen::Vector6d& twistd) { u.segment(6+13,6) = twistd; }

  Eigen::Affine3d get_x( ) const
  {
    Eigen::Affine3d ret;
    ret.translation() = u.segment(6+0,3);
    Eigen::Quaterniond q(u(6+3), u(6+4), u(6+5), u(6+6) );
    ret.linear() = q.toRotationMatrix();
    return ret;
  }
  Eigen::Vector6d get_twist() const
  {
    Eigen::Vector6d ret;
    ret = u.segment(6+7,6);
    return ret;
  }
  Eigen::Vector6d get_twistd() const
  {
    Eigen::Vector6d ret;
    ret = u.segment(6+13,6);
    return ret;
  }
  
  CartesianRegulatorInput& operator=(const CartesianRegulatorInput& rhs)
  {
    this->dim = rhs.dim;
    this->u   = rhs.u;
    return *this;
  }

  
};
typedef std::shared_ptr<CartesianRegulatorInput> CartesianRegulatorInputPtr;
typedef const std::shared_ptr<CartesianRegulatorInput const> CartesianRegulatorInputConstPtr;


}  // namespace cnr_regulator_interface

#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INPUTS__H
