#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_FEEDBACK__H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_FEEDBACK__H

#include <memory>
#include <Eigen/Dense>
#include <rosdyn_core/spacevect_algebra.h>
#include <rosdyn_utilities/chain_state.h>

namespace cnr
{
namespace control
{


/**
 * @brief The BaseRegulatorFeedback struct
 */
struct BaseRegulatorFeedback
{
  typedef std::shared_ptr<BaseRegulatorFeedback> Ptr;
  typedef std::shared_ptr<BaseRegulatorFeedback const> ConstPtr;

  BaseRegulatorFeedback() = default;
  virtual ~BaseRegulatorFeedback() = default;
  BaseRegulatorFeedback(const BaseRegulatorFeedback&) = delete;
  BaseRegulatorFeedback& operator=(const BaseRegulatorFeedback&) = delete;
  BaseRegulatorFeedback(BaseRegulatorFeedback&&) = delete;
  BaseRegulatorFeedback& operator=(BaseRegulatorFeedback&&) = delete;
};

typedef BaseRegulatorFeedback::Ptr BaseRegulatorFeedbackPtr;
typedef BaseRegulatorFeedback::ConstPtr BaseRegulatorFeedbackConstPtr;


/**
 * @brief JointRegulatorFeedback
 */
template<int N, int MaxN=N>
class JointRegulatorFeedback : public BaseRegulatorFeedback
{
protected:
  rosdyn::ChainState<N,MaxN> robot_state;
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef std::shared_ptr<JointRegulatorFeedback> Ptr;
  typedef std::shared_ptr<JointRegulatorFeedback const> ConstPtr;

  JointRegulatorFeedback() = default;
  virtual ~JointRegulatorFeedback() = default;
  JointRegulatorFeedback(const JointRegulatorFeedback&) = delete;
  JointRegulatorFeedback(JointRegulatorFeedback&&) = delete;
  JointRegulatorFeedback& operator=(JointRegulatorFeedback&&) = delete;

  JointRegulatorFeedback(const rosdyn::Chain& kin)
  {
    robot_state.init(kin);
  }

  rosdyn::ChainState<N,MaxN>& robotState()
  {
    return robot_state;
  }

  const rosdyn::ChainState<N,MaxN>& robotState() const
  {
    return robot_state;
  }
};

template<int N, int MaxN=N>
using JointRegulatorFeedbackPtr = typename JointRegulatorFeedback<N,MaxN>::Ptr;

template<int N, int MaxN=N>
using JointRegulatorFeedbackConstPtr = typename JointRegulatorFeedback<N,MaxN>::ConstPtr;


/**
 * @brief The CartesianRegulatorFeedback struct
 */
template<int N,int MaxN>
struct CartesianRegulatorFeedback : public JointRegulatorFeedback<N,MaxN>
{ 
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef std::shared_ptr<CartesianRegulatorFeedback> Ptr;
  typedef std::shared_ptr<CartesianRegulatorFeedback const> ConstPtr;
  
  virtual ~CartesianRegulatorFeedback() = default;
  CartesianRegulatorFeedback(const CartesianRegulatorFeedback&) = delete;
  
  CartesianRegulatorFeedback(CartesianRegulatorFeedback&&) = delete;
  CartesianRegulatorFeedback& operator=(CartesianRegulatorFeedback&&) = delete;

  CartesianRegulatorFeedback( ) = default;
  CartesianRegulatorFeedback(rosdyn::Chain& kin) : JointRegulatorFeedback<N,MaxN>(kin)
  {
  }

  CartesianRegulatorFeedback(const rosdyn::ChainState<N,MaxN>& status) : JointRegulatorFeedback<N,MaxN>(status)
  {
  }
};

template<int N, int MaxN=N>
using CartesianRegulatorFeedbackPtr = typename CartesianRegulatorFeedback<N,MaxN>::Ptr;

template<int N, int MaxN=N>
using CartesianRegulatorFeedbackConstPtr = typename CartesianRegulatorFeedback<N,MaxN>::ConstPtr;


}  // namespace control
}  // namespace cnr


#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_FEEDBACK__H
