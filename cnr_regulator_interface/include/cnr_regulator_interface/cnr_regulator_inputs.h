#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INPUTS__H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INPUTS__H

#include <ros/time.h>
#include <eigen3/Eigen/Dense>
#include <rosdyn_core/spacevect_algebra.h>
#include <memory>


namespace cnr_regulator_interface
{


/**
 * @brief The RegulatorInputBase struct
 */
struct RegulatorInputBase
{
  RegulatorInputBase() = delete;
  virtual ~RegulatorInputBase() = default;
  RegulatorInputBase(const RegulatorInputBase&) = delete;
  RegulatorInputBase& operator=(const RegulatorInputBase&) = delete;
  RegulatorInputBase(RegulatorInputBase&&) = delete;
  RegulatorInputBase& operator=(RegulatorInputBase&&) = delete;

  RegulatorInputBase(size_t n_dof)
    : dof(n_dof){u.resize(6 + n_dof); u.setZero();}
  size_t           dof;
  Eigen::VectorXd  u;

  void set_period         (const ros::Duration& period         ) { u(0) = period.toSec();         }
  //void set_scaled_time    (const ros::Duration& scaled_time    ) { u(1) = scaled_time.toSec();    }
  void set_target_override(const double& global_override       ) { u(2) = global_override;        }
  void set_time_from_start(const ros::Duration& time_from_start) { u(3) = time_from_start.toSec();}
  void set_goal_tolerance (const double& goal_tolerance        ) { u(4) = goal_tolerance;         }
  void set_path_tolerance (const double& path_tolerance        ) { u(5) = path_tolerance;         }
  void set_u              (const Eigen::VectorXd& input_values )
  {
    assert(input_values.rows()==dof);
    u.segment(6,dof)=input_values;
  }

  ros::Duration   get_period         ( ) const { return ros::Duration(u(0));  }
  //ros::Duration   get_scaled_time    ( ) const { return ros::Duration(u(1));  }
  double          get_target_override( ) const { return u(2);  }
  ros::Duration   get_time_from_start( ) const { return ros::Duration(u(3));  }
  double          get_goal_tolerance ( ) const { return u(4); }
  double          get_path_tolerance ( ) const { return u(5); }
  Eigen::VectorXd get_u              ( ) const { return u.segment(6,dof); }

};

typedef std::shared_ptr<RegulatorInputBase> RegulatorInputBasePtr;
typedef const std::shared_ptr<RegulatorInputBase const> RegulatorInputBaseConstPtr;

struct JointRegulatorInput : public cnr_regulator_interface::RegulatorInputBase
{
  JointRegulatorInput() = delete;
  virtual ~JointRegulatorInput() = default;
  JointRegulatorInput(const JointRegulatorInput&) = delete;
  JointRegulatorInput& operator=(const JointRegulatorInput&) = delete;
  JointRegulatorInput(JointRegulatorInput&&) = delete;
  JointRegulatorInput& operator=(JointRegulatorInput&&) = delete;

  JointRegulatorInput(size_t nAx): RegulatorInputBase( (nAx*4) ){ dof = nAx; }

  void set_q             (const Eigen::VectorXd& q     ) { u.segment(6+0*dof,dof) = q     ; }
  void set_qd            (const Eigen::VectorXd& qd    ) { u.segment(6+1*dof,dof) = qd    ; }
  void set_qdd           (const Eigen::VectorXd& qdd   ) { u.segment(6+2*dof,dof) = qdd   ; }
  void set_effort        (const Eigen::VectorXd& effort) { u.segment(6+3*dof,dof) = effort; }

  Eigen::VectorXd get_q     ( ) const { return u.segment(6+0*dof,dof); }
  Eigen::VectorXd get_qd    ( ) const { return u.segment(6+1*dof,dof); }
  Eigen::VectorXd get_qdd   ( ) const { return u.segment(6+2*dof,dof); }
  Eigen::VectorXd get_effort( ) const { return u.segment(6+3*dof,dof); }
};

typedef std::shared_ptr<JointRegulatorInput> JointRegulatorInputPtr;
typedef const std::shared_ptr<JointRegulatorInput const> JointRegulatorInputConstPtr;


/**
 * @brief The CartesianRegulatorInput struct
 */
struct CartesianRegulatorInput : public cnr_regulator_interface::RegulatorInputBase
{
  virtual ~CartesianRegulatorInput() = default;
  CartesianRegulatorInput(const CartesianRegulatorInput&) = delete;
  CartesianRegulatorInput& operator=(const CartesianRegulatorInput&) = delete;
  CartesianRegulatorInput(CartesianRegulatorInput&&) = delete;
  CartesianRegulatorInput& operator=(CartesianRegulatorInput&&) = delete;

  CartesianRegulatorInput( ): RegulatorInputBase( (7 + 6 + 6) ){}

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
};
typedef std::shared_ptr<CartesianRegulatorInput> CartesianRegulatorInputPtr;
typedef const std::shared_ptr<CartesianRegulatorInput const> CartesianRegulatorInputConstPtr;


}  // namespace cnr_regulator_interface

#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INPUTS__H
