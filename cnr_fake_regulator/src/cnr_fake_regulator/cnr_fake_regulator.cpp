#include <cnr_fake_regulator/cnr_fake_regulator.h>
#include <cnr_interpolator_interface/cnr_interpolator_inputs.h>
#include <cnr_interpolator_interface/cnr_interpolator_outputs.h>
#include <cnr_regulator_interface/cnr_regulator_inputs.h>
#include <cnr_regulator_interface/cnr_regulator_outputs.h>

#include <pluginlib/class_list_macros.h> // header for PLUGINLIB_EXPORT_CLASS. NOTE IT SHOULD STAY IN THE CPP FILE NOTE

PLUGINLIB_EXPORT_CLASS(cnr_fake_regulator::FakeRegulator, cnr_regulator_interface::RegulatorBase)

namespace cnr_fake_regulator
{

using cnr_regulator_interface::CartesianRegulatorInput;
using cnr_regulator_interface::CartesianRegulatorInputPtr;
using cnr_regulator_interface::CartesianRegulatorInputConstPtr;
using cnr_regulator_interface::CartesianRegulatorOutput;
using cnr_regulator_interface::CartesianRegulatorOutputPtr;
using cnr_regulator_interface::CartesianRegulatorOutputConstPtr;

using cnr_regulator_interface::JointRegulatorInput;
using cnr_regulator_interface::JointRegulatorInputPtr;
using cnr_regulator_interface::JointRegulatorInputConstPtr;
using cnr_regulator_interface::JointRegulatorOutput;
using cnr_regulator_interface::JointRegulatorOutputPtr;
using cnr_regulator_interface::JointRegulatorOutputConstPtr;


bool FakeRegulator::initialize(ros::NodeHandle&                              root_nh,
                               ros::NodeHandle&                              controller_nh,
                               cnr_logger::TraceLoggerPtr                    logger,
                               cnr_controller_interface::KinematicsStructPtr kin,
                               cnr_controller_interface::KinematicStatusPtr  state)
{
  CNR_RETURN_BOOL(*logger, cnr_regulator_interface::RegulatorBase::initialize(root_nh,controller_nh,logger,kin,state));
}

bool FakeRegulator::starting(const ros::Time& time)
{
CNR_RETURN_BOOL(*m_logger, cnr_regulator_interface::RegulatorBase::starting(time));
}


bool FakeRegulator::update(cnr_interpolator_interface::InterpolatorInterfacePtr interpolator,
                           cnr_regulator_interface::RegulatorInputBaseConstPtr  input,
                           cnr_regulator_interface::RegulatorOutputBasePtr      output)
{
  JointRegulatorInputConstPtr     j_in  = std::dynamic_pointer_cast<JointRegulatorInput const>(input);
  JointRegulatorOutputPtr         j_out = std::dynamic_pointer_cast<JointRegulatorOutput>(output);

  if(!j_in || !j_out)
  {
    CNR_RETURN_FALSE(*m_logger, "Error in casting the input and outputs pointers. Abort.");
  }

  j_out->set_q(j_in->q_setpoint);
  j_out->set_qd(j_in->qd_setpoint);
  j_out->set_qdd(j_in->qd_setpoint);
  j_out->set_effort(j_in->effort_setpoint);

  return true;
}


}


