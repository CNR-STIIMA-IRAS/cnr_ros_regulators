#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_FEEDBACK__H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_FEEDBACK__H

#include <memory>
#include <Eigen/Dense>
#include <rosdyn_core/spacevect_algebra.h>
#include <rosdyn_utilities/chain_state.h>

namespace cnr_regulator_interface
{


/**
 * @brief The BaseRegulatorFeedback struct
 */
struct BaseRegulatorFeedback
{
  BaseRegulatorFeedback() = default;
  virtual ~BaseRegulatorFeedback() = default;
  BaseRegulatorFeedback(const BaseRegulatorFeedback&) = delete;
  BaseRegulatorFeedback& operator=(const BaseRegulatorFeedback&) = delete;
  BaseRegulatorFeedback(BaseRegulatorFeedback&&) = delete;
  BaseRegulatorFeedback& operator=(BaseRegulatorFeedback&&) = delete;

};

typedef std::shared_ptr<BaseRegulatorFeedback> BaseRegulatorFeedbackPtr;
typedef std::shared_ptr<BaseRegulatorFeedback const> BaseRegulatorFeedbackConstPtr;

class JointRegulatorFeedback : public cnr_regulator_interface::BaseRegulatorFeedback
{
protected:
  rosdyn::ChainStatePtr robot_state;
  
public:
  typedef std::shared_ptr<JointRegulatorFeedback> Ptr;
  typedef std::shared_ptr<JointRegulatorFeedback const> ConstPtr;

  JointRegulatorFeedback() = default;
  virtual ~JointRegulatorFeedback() = default;
  JointRegulatorFeedback(const JointRegulatorFeedback&) = delete;
  JointRegulatorFeedback(JointRegulatorFeedback&&) = delete;
  JointRegulatorFeedback& operator=(JointRegulatorFeedback&&) = delete;

  JointRegulatorFeedback(rosdyn::ChainInterfacePtr kin)
  {
    robot_state.reset(new rosdyn::ChainState(kin));
  }
  
  JointRegulatorFeedback(const rosdyn::ChainState& status)
  {
    setRobotState(status);
  }
  
  rosdyn::ChainStateConstPtr getRobotState() const
  {
    return robot_state;
  }
  
  JointRegulatorFeedback& setRobotState(const rosdyn::ChainState& status)
  {
    if(!robot_state)
    {
      robot_state.reset(new rosdyn::ChainState(status.getKin()));
    }
    *robot_state = status;
    
    return *this;
  }
  
  JointRegulatorFeedback& setRobotState(rosdyn::ChainStateConstPtr& status)
  {
    if(!robot_state)
    {
      robot_state.reset(new rosdyn::ChainState(status->getKin()));
    }

    *robot_state = *status;
    
    return *this;
  }
};

typedef JointRegulatorFeedback::Ptr JointRegulatorFeedbackPtr;
typedef JointRegulatorFeedback::ConstPtr JointRegulatorFeedbackConstPtr;


/**
 * @brief The CartesianRegulatorFeedback struct
 */
struct CartesianRegulatorFeedback : public cnr_regulator_interface::JointRegulatorFeedback
{
  
public:
  typedef std::shared_ptr<CartesianRegulatorFeedback> Ptr;
  typedef std::shared_ptr<CartesianRegulatorFeedback const> ConstPtr;
  
  virtual ~CartesianRegulatorFeedback() = default;
  CartesianRegulatorFeedback(const CartesianRegulatorFeedback&) = delete;
  
  CartesianRegulatorFeedback(CartesianRegulatorFeedback&&) = delete;
  CartesianRegulatorFeedback& operator=(CartesianRegulatorFeedback&&) = delete;

  CartesianRegulatorFeedback( ) = default;
  CartesianRegulatorFeedback(rosdyn::ChainInterfacePtr kin) : JointRegulatorFeedback(kin)
  {
  }
  CartesianRegulatorFeedback(const rosdyn::ChainState& status) : JointRegulatorFeedback(status)
  {
  }
  
  CartesianRegulatorFeedback& setRobotState(const rosdyn::ChainState& state) 
  {
    *robot_state = state;
    robot_state->updateTransformations();
    return *this;
  }
  

};

typedef CartesianRegulatorFeedback::Ptr CartesianRegulatorFeedbackPtr;
typedef CartesianRegulatorFeedback::ConstPtr CartesianRegulatorFeedbackConstPtr;


}  // namespace cnr_regulator_interface

#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_FEEDBACK__H
