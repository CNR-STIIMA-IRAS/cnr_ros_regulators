#include <cnr_fake_regulator/cnr_fake_regulator.h>
#include <cnr_interpolator_interface/cnr_interpolator_inputs.h>
#include <cnr_interpolator_interface/cnr_interpolator_outputs.h>
#include <cnr_regulator_interface/cnr_regulator_references.h>
#include <cnr_regulator_interface/cnr_regulator_control_commands.h>

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS(cnr_fake_regulator::FakeRegulator, cnr_regulator_interface::BaseRegulator)

namespace cnr_fake_regulator
{

using cnr_regulator_interface::JointRegulatorReference;
using cnr_regulator_interface::JointRegulatorReferenceConstPtr;
using cnr_regulator_interface::JointRegulatorControlCommand;
using cnr_regulator_interface::JointRegulatorControlCommandPtr;

bool FakeRegulator::initialize(ros::NodeHandle&            root_nh,
                               ros::NodeHandle&            controller_nh, 
                               cnr_regulator_interface::BaseRegulatorParamsPtr param)
{
  if(!cnr_regulator_interface::BaseJointRegulator::initialize(root_nh,controller_nh,param))
  {
    return false;
  }
  CNR_RETURN_TRUE(logger());
}

bool FakeRegulator::starting(cnr_regulator_interface::BaseRegulatorStateConstPtr state0, const ros::Time& time)
{
  CNR_TRACE_START(logger());
  if(!cnr_regulator_interface::BaseJointRegulator::starting(state0, time))
  {
    CNR_RETURN_FALSE(logger());
  }
  CNR_RETURN_TRUE(logger());
}


bool FakeRegulator::update(cnr_regulator_interface::BaseRegulatorReferenceConstPtr _r,
                           cnr_regulator_interface::BaseRegulatorControlCommandPtr _u)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(logger());
  if(!cnr_regulator_interface::BaseJointRegulator::update(_r,_u))
  {
    CNR_RETURN_FALSE(logger());
  }

  cnr_interpolator_interface::JointInputPtr  interpolator_input(new cnr_interpolator_interface::JointInput());
  cnr_interpolator_interface::JointOutputPtr interpolator_output(new cnr_interpolator_interface::JointOutput());
  interpolator_input->override = r()->get_target_override();
  interpolator_input->time     = regulator_time_;

  if(!interpolator())
  {
    CNR_RETURN_FALSE(logger());
  }
  interpolator()->interpolate(interpolator_input, interpolator_output);

  regulator_time_ += period() * u()->get_scaling();

  u()->set_q(interpolator_output->pnt.positions);
  u()->set_qd(interpolator_output->pnt.velocities);
  u()->set_qdd(interpolator_output->pnt.accelerations);
  u()->set_effort(interpolator_output->pnt.effort);
  u()->set_scaling(r()->get_target_override());
  u()->set_in_path_tolerance((regulator_time_-interpolator()->trjTime()).toSec()>0);
  u()->set_in_goal_tolerance((regulator_time_-interpolator()->trjTime()).toSec()>0);
  u()->set_time_from_start(regulator_time_);

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(logger());
}


}


