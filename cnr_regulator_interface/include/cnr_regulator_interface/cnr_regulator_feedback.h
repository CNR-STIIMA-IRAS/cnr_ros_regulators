#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_FEEDBACK__H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_FEEDBACK__H

#include <memory>
#include <eigen3/Eigen/Dense>
#include <rosdyn_core/spacevect_algebra.h>
#include <cnr_controller_interface/utils/cnr_kinematics_utils.h>

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
  cnr_controller_interface::KinematicStatusPtr robot_state;
  
public:
  typedef std::shared_ptr<JointRegulatorFeedback> Ptr;
  typedef std::shared_ptr<JointRegulatorFeedback const> ConstPtr;

  JointRegulatorFeedback() = default;
  virtual ~JointRegulatorFeedback() = default;
  JointRegulatorFeedback(const JointRegulatorFeedback&) = delete;
  JointRegulatorFeedback(JointRegulatorFeedback&&) = delete;
  JointRegulatorFeedback& operator=(JointRegulatorFeedback&&) = delete;

  JointRegulatorFeedback(cnr_controller_interface::KinematicsStructPtr kin)
  {
    robot_state.reset(new cnr_controller_interface::KinematicStatus(kin));
  }
  
  JointRegulatorFeedback(const cnr_controller_interface::KinematicStatus& status)
  {
    setRobotState(status);
  }
  
  cnr_controller_interface::KinematicStatusConstPtr getRobotState() const
  {
    return robot_state;
  }
  
  JointRegulatorFeedback& setRobotState(const cnr_controller_interface::KinematicStatus& status)
  {
    if(!robot_state)
    {
      robot_state.reset(new cnr_controller_interface::KinematicStatus(status.getKin()));
    }
    *robot_state = status;
    
    return *this;
  }
  
  JointRegulatorFeedback& setRobotState(cnr_controller_interface::KinematicStatusConstPtr& status)
  {
    if(!robot_state)
    {
      robot_state.reset(new cnr_controller_interface::KinematicStatus(status->getKin()));
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
  CartesianRegulatorFeedback(cnr_controller_interface::KinematicsStructPtr kin) : JointRegulatorFeedback(kin)
  {
  }
  CartesianRegulatorFeedback(const cnr_controller_interface::KinematicStatus& status) : JointRegulatorFeedback(status)
  {
  }
  
  CartesianRegulatorFeedback& setRobotState(const cnr_controller_interface::KinematicStatus& state) 
  {
    *robot_state = state;
    robot_state->updateTransformation();
    return *this;
  }
  

};

typedef CartesianRegulatorFeedback::Ptr CartesianRegulatorFeedbackPtr;
typedef CartesianRegulatorFeedback::ConstPtr CartesianRegulatorFeedbackConstPtr;


}  // namespace cnr_regulator_interface

#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_FEEDBACK__H
