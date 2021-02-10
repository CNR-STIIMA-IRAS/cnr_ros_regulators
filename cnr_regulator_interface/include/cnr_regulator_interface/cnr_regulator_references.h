#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INPUTS__H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INPUTS__H

#include <memory>
#include <type_traits>
#include <ros/time.h>
#include <Eigen/Dense>
#include <rosdyn_core/spacevect_algebra.h>

#include <eigen_matrix_utils/overloads.h>

namespace eu = eigen_utils;

namespace cnr
{
namespace control
{

/**
 * @brief The BaseRegulatorReference struct
 */
struct BaseRegulatorReference
{
  typedef std::shared_ptr<BaseRegulatorReference> Ptr;
  typedef std::shared_ptr<BaseRegulatorReference const> ConstPtr;
  
  virtual ~BaseRegulatorReference() = default;
  BaseRegulatorReference(const BaseRegulatorReference&) = delete;
  BaseRegulatorReference(BaseRegulatorReference&&) = delete;
  BaseRegulatorReference& operator=(BaseRegulatorReference&&) = delete;

  BaseRegulatorReference() = default;

  int dof;
  ros::Duration period;
  ros::Duration time_from_start;
  double target_override;
  
  BaseRegulatorReference& operator=(const BaseRegulatorReference& rhs)
  {
    period = rhs.period;
    time_from_start = rhs.time_from_start;
    target_override = rhs.target_override;
    return *this;
  }
};

typedef BaseRegulatorReference::Ptr BaseRegulatorReferencePtr;
typedef BaseRegulatorReference::ConstPtr BaseRegulatorReferenceConstPtr;


/**
 * @brief The JointRegulatorReference struct
 */
template<int N, int MaxN=N>
struct JointRegulatorReference : public BaseRegulatorReference
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  enum {DimAtCompileTime = N, MaxDimAtCompileTime = MaxN };

  using Value = typename std::conditional<N==1, double, Eigen::Matrix<double,N,1,Eigen::ColMajor,MaxN> >::type;
  using Ptr = std::shared_ptr<JointRegulatorReference>;
  using ConstPtr = std::shared_ptr<JointRegulatorReference const>;

  Value q;
  Value qd;
  Value qdd;
  Value effort;

  Value goal_tolerance;
  Value path_tolerance;

  int dof() const {return eu::rows(q);}

  void set_dimension(const int& dim)
  {
    if(eu::rows(q)==1 && dim!=1) throw std::runtime_error("The command size is a scalar, and it cannot be resized.");
    if(!eu::resize(q,dim))
    {
      throw std::runtime_error("The command is fixed-size, and it cannot be resized.");
    }
    eu::setZero(q);
    eu::resize(qd,dim); eu::setZero(qd);
    eu::resize(qdd,dim); eu::setZero(qdd);
    eu::resize(effort,dim); eu::setZero(effort);
  }

  JointRegulatorReference() = default;
  virtual ~JointRegulatorReference() = default;
  JointRegulatorReference(const JointRegulatorReference&) = delete;
  JointRegulatorReference(JointRegulatorReference&&) = delete;
  JointRegulatorReference& operator=(JointRegulatorReference&&) = delete;
};

template<int N, int MaxN=N>
using JointRegulatorReferencePtr = typename JointRegulatorReference<N,MaxN>::Ptr ;

template<int N, int MaxN=N>
using JointRegulatorReferenceConstPtr = typename JointRegulatorReference<N,MaxN>::ConstPtr ;

/**
 * @brief The CartesianRegulatorReference struct
 */
struct CartesianRegulatorReference : public BaseRegulatorReference
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  using Ptr = std::shared_ptr<CartesianRegulatorReference>;
  using ConstPtr = std::shared_ptr<CartesianRegulatorReference const>;

  Eigen::Vector3d     x;
  Eigen::Quaterniond  q;
  Eigen::Vector6d     twist;
  Eigen::Vector6d     twistd;

  void set_x(const Eigen::Affine3d& T)
  {
    q = Eigen::Quaterniond(T.linear());
    x = T.translation();
  }

  Eigen::Affine3d get_x( ) const
  {
    Eigen::Affine3d ret;
    ret.translation() = x;
    ret.linear() = q.toRotationMatrix();
    return ret;
  }
  
  CartesianRegulatorReference( ) = default;
  virtual ~CartesianRegulatorReference() = default;
  CartesianRegulatorReference(const CartesianRegulatorReference&) = delete;
  CartesianRegulatorReference(CartesianRegulatorReference&&) = delete;
  CartesianRegulatorReference& operator=(CartesianRegulatorReference&&) = delete;

};

typedef std::shared_ptr<CartesianRegulatorReference> CartesianRegulatorReferencePtr;
typedef std::shared_ptr<CartesianRegulatorReference const> CartesianRegulatorReferenceConstPtr;


}  // namespace control
}  // namespace cnr

#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INPUTS__H
