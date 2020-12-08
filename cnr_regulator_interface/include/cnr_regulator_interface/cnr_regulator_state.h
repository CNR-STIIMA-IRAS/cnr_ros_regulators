#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_STATE__H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_STATE__H

#include <memory>
#include <eigen3/Eigen/Dense>
#include <rosdyn_core/spacevect_algebra.h>
#include <rosdyn_utilities/chain_state.h>

namespace cnr_regulator_interface
{


/**
 * @brief The BaseRegulatorState struct
 */
struct BaseRegulatorState
{
  BaseRegulatorState() = default;
  virtual ~BaseRegulatorState() = default;
  BaseRegulatorState(const BaseRegulatorState&) = delete;
  BaseRegulatorState& operator=(const BaseRegulatorState&) = delete;
  BaseRegulatorState(BaseRegulatorState&&) = delete;
  BaseRegulatorState& operator=(BaseRegulatorState&&) = delete;
};

typedef std::shared_ptr<BaseRegulatorState> BaseRegulatorStatePtr;
typedef std::shared_ptr<BaseRegulatorState const> BaseRegulatorStateConstPtr;

class JointRegulatorState : public cnr_regulator_interface::BaseRegulatorState
{
protected:
  rosdyn::ChainStateXPtr robot_state;
  
public:
  typedef std::shared_ptr<JointRegulatorState> Ptr;
  typedef std::shared_ptr<JointRegulatorState const> ConstPtr;

  JointRegulatorState() = default;
  virtual ~JointRegulatorState() = default;
  JointRegulatorState(const JointRegulatorState&) = delete;
  JointRegulatorState(JointRegulatorState&&) = delete;
  JointRegulatorState& operator=(JointRegulatorState&&) = delete;

  JointRegulatorState(rosdyn::ChainInterfacePtr kin)
  {
    robot_state.reset(new rosdyn::ChainStateX(kin));
  }
  
  JointRegulatorState(const rosdyn::ChainStateX& status)
  {
    setRobotState(status);
  }

  rosdyn::ChainStateXPtr getRobotState() 
  { 
    return robot_state;
  }

  rosdyn::ChainStateXConstPtr getRobotState() const 
  { 
    return robot_state; 
   }
  
  virtual void setRobotState(const rosdyn::ChainStateX& status)
  {
    if(!robot_state)
    {
      robot_state.reset(new rosdyn::ChainStateX(status.getKin()));
    }
    *robot_state = status;
  }
  
  virtual void setRobotState(rosdyn::ChainStateXConstPtr& status)
  {
    if(!robot_state)
    {
      robot_state.reset(new rosdyn::ChainStateX(status->getKin()));
    }
    *robot_state = *status;
  }
};

typedef JointRegulatorState::Ptr JointRegulatorStatePtr;
typedef JointRegulatorState::ConstPtr JointRegulatorStateConstPtr;


/**
 * @brief The CartesianRegulatorState struct
 */
struct CartesianRegulatorState : public cnr_regulator_interface::JointRegulatorState
{
  
public:
  typedef std::shared_ptr<CartesianRegulatorState> Ptr;
  typedef std::shared_ptr<CartesianRegulatorState const> ConstPtr;
  
  virtual ~CartesianRegulatorState() = default;
  CartesianRegulatorState(const CartesianRegulatorState&) = delete;
  
  CartesianRegulatorState(CartesianRegulatorState&&) = delete;
  CartesianRegulatorState& operator=(CartesianRegulatorState&&) = delete;

  CartesianRegulatorState( ) = default;
  CartesianRegulatorState(rosdyn::ChainInterfacePtr kin) : JointRegulatorState(kin)
  {
  }
  CartesianRegulatorState(const rosdyn::ChainStateX& status) : JointRegulatorState(status)
  {
  }
  
  virtual void setRobotState(const rosdyn::ChainStateX& status)
  {
    JointRegulatorState::setRobotState(status);
    robot_state->updateTransformations();
  }
  
  virtual void setRobotState(rosdyn::ChainStateXConstPtr& status) override
  {
    JointRegulatorState::setRobotState(status);
    robot_state->updateTransformations();
  }
  

};

typedef CartesianRegulatorState::Ptr CartesianRegulatorStatePtr;
typedef CartesianRegulatorState::ConstPtr CartesianRegulatorStateConstPtr;


}  // namespace cnr_regulator_interface

#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_STATE__H
