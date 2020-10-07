#ifndef CNR_IMPEDANCE_REGULATOR_INTERFACE__CNR_IMPEDANCE_REGULATOR_STATE__H
#define CNR_IMPEDANCE_REGULATOR_INTERFACE__CNR_IMPEDANCE_REGULATOR_STATE__H

#include <memory>
#include <eigen3/Eigen/Dense>
#include <rosdyn_core/spacevect_algebra.h>
#include <cnr_controller_interface/utils/cnr_kinematics_utils.h>
#include <cnr_regulator_interface/cnr_regulator_state.h>

namespace cnr_impedance_regulator
{


struct ImpedanceRegulatorState : public cnr_regulator_interface::JointRegulatorState
{
  typedef std::shared_ptr<ImpedanceRegulatorState> Ptr;
  typedef const std::shared_ptr<ImpedanceRegulatorState const> ConstPtr;
  
  ImpedanceRegulatorState() = default;
  virtual ~ImpedanceRegulatorState() = default;
  ImpedanceRegulatorState(const ImpedanceRegulatorState&) = delete;
  ImpedanceRegulatorState(ImpedanceRegulatorState&&) = delete;
  ImpedanceRegulatorState& operator=(ImpedanceRegulatorState&&) = delete;
  
  ImpedanceRegulatorState(cnr_controller_interface::KinematicsStructPtr kin) : JointRegulatorState(kin) {}
  
  ImpedanceRegulatorState(const cnr_controller_interface::KinematicStatus& status) : JointRegulatorState(status) {}
  
  struct Model
  {
    Eigen::VectorXd x;
    Eigen::VectorXd xd;
    Eigen::VectorXd xdd;
    Eigen::VectorXd effort;
    Model(const size_t& dim)
    {
      x     .resize(dim); x     .setZero();
      xd    .resize(dim); xd    .setZero();
      xdd   .resize(dim); xdd   .setZero();
      effort.resize(dim); effort.setZero();
    }
  };
  
  std::shared_ptr<Model> model;
  
  ImpedanceRegulatorState& setModelState(const cnr_controller_interface::KinematicStatus& status)
  {
    if(!model)
    {
      model.reset(new Model(status.nAx()));
    }
    model->x = status.q();
    model->xd = status.qd();
    model->xdd = status.qdd();
    model->effort = status.effort();
  
    return *this;
  }
  
  

};

typedef ImpedanceRegulatorState::Ptr ImpedanceRegulatorStatePtr;
typedef ImpedanceRegulatorState::ConstPtr ImpedanceRegulatorStateConstPtr;


}  // namespace cnr_impedance_regulator

#endif  // CNR_IMPEDANCE_REGULATOR_INTERFACE__CNR_IMPEDANCE_REGULATOR_STATE__H
