#include <cnr_fake_regulator/cnr_fake_regulator.h>
#include <cnr_interpolator_interface/cnr_interpolator_inputs.h>
#include <cnr_interpolator_interface/cnr_interpolator_outputs.h>
#include <cnr_regulator_interface/cnr_regulator_inputs.h>
#include <cnr_regulator_interface/cnr_regulator_outputs.h>

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS(cnr_fake_regulator::FakeRegulator, cnr_regulator_interface::BaseRegulator)

namespace cnr_fake_regulator
{

using cnr_regulator_interface::JointRegulatorInput;
using cnr_regulator_interface::JointRegulatorInputConstPtr;
using cnr_regulator_interface::JointRegulatorOutput;
using cnr_regulator_interface::JointRegulatorOutputPtr;

bool FakeRegulator::initialize(ros::NodeHandle&            root_nh,
                               ros::NodeHandle&            controller_nh, 
                               cnr_regulator_interface::BaseRegulatorOptionsConstPtr opts)
{
  if(!cnr_regulator_interface::BaseJointRegulator::initialize(root_nh,controller_nh,opts))
  {
    return false;
  }
  CNR_RETURN_TRUE(this->m_opts.logger);
}

bool FakeRegulator::starting(cnr_regulator_interface::BaseRegulatorStatePtr state0, const ros::Time& time)
{
  CNR_RETURN_BOOL(m_opts.logger, cnr_regulator_interface::BaseJointRegulator::starting(state0, time));
}


bool FakeRegulator::update(cnr_interpolator_interface::InterpolatorInterfacePtr interpolator,
                           cnr_regulator_interface::BaseRegulatorInputConstPtr  input,
                           cnr_regulator_interface::BaseRegulatorOutputPtr      output)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(m_opts.logger);
  if(!cnr_regulator_interface::BaseJointRegulator::update(interpolator, input, output))
  {
    CNR_RETURN_FALSE(m_opts.logger);
  }

  cnr_interpolator_interface::JointInputPtr  interpolator_input(new cnr_interpolator_interface::JointInput());
  cnr_interpolator_interface::JointOutputPtr interpolator_output(new cnr_interpolator_interface::JointOutput());
  interpolator_input->override = m_input.get_target_override();
  interpolator_input->time     = m_regulator_time;

  if(!interpolator)
  {
    CNR_RETURN_FALSE(m_opts.logger);
  }
  interpolator->interpolate(interpolator_input, interpolator_output);

  m_regulator_time += m_period * m_output->get_scaling();

  m_output->set_q(interpolator_output->pnt.positions);
  m_output->set_qd(interpolator_output->pnt.velocities);
  m_output->set_qdd(interpolator_output->pnt.accelerations);
  m_output->set_effort(interpolator_output->pnt.effort);
  m_output->set_scaling(m_input.get_target_override());
  
  m_output->set_in_path_tolerance((m_regulator_time-interpolator->trjTime()).toSec()>0);

  m_output->set_in_goal_tolerance((m_regulator_time-interpolator->trjTime()).toSec()>0);
  m_output->set_time_from_start(m_regulator_time);

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(m_opts.logger);
}


}


