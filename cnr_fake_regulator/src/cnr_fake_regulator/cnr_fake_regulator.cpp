#include <cnr_fake_regulator/cnr_fake_regulator.h>
#include <cnr_interpolator_interface/cnr_interpolator_inputs.h>
#include <cnr_interpolator_interface/cnr_interpolator_outputs.h>
#include <cnr_regulator_interface/cnr_regulator_inputs.h>
#include <cnr_regulator_interface/cnr_regulator_outputs.h>

#include <pluginlib/class_list_macros.h> // header for PLUGINLIB_EXPORT_CLASS. NOTE IT SHOULD STAY IN THE CPP FILE NOTE

PLUGINLIB_EXPORT_CLASS(cnr_fake_regulator::FakeRegulator, cnr_regulator_interface::RegulatorBase)

namespace cnr_fake_regulator
{

using cnr_regulator_interface::JointRegulatorInput;
using cnr_regulator_interface::JointRegulatorInputConstPtr;
using cnr_regulator_interface::JointRegulatorOutput;
using cnr_regulator_interface::JointRegulatorOutputPtr;

bool FakeRegulator::initialize(ros::NodeHandle&                              root_nh,
                               ros::NodeHandle&                              controller_nh,
                               cnr_logger::TraceLoggerPtr                    logger,
                               cnr_controller_interface::KinematicsStructPtr kin,
                               cnr_controller_interface::KinematicStatusPtr  state,
                               const ros::Duration&                          period)
{
  CNR_RETURN_BOOL(*logger,
                  cnr_regulator_interface::RegulatorBase::initialize(root_nh,controller_nh,logger,kin,state, period));
}

bool FakeRegulator::starting(const ros::Time& time)
{
  CNR_RETURN_BOOL(*m_logger, cnr_regulator_interface::RegulatorBase::starting(time));
}


bool FakeRegulator::update(cnr_interpolator_interface::InterpolatorInterfacePtr interpolator,
                           cnr_regulator_interface::RegulatorInputBaseConstPtr  input,
                           cnr_regulator_interface::RegulatorOutputBasePtr      output)
{
  JointRegulatorInputConstPtr j_in  = std::dynamic_pointer_cast<JointRegulatorInput const>(input);
  JointRegulatorOutputPtr     j_out = std::dynamic_pointer_cast<JointRegulatorOutput>(output);

  if(!j_in || !j_out)
  {
    CNR_RETURN_FALSE(*m_logger, "Error in casting the input and outputs pointers. Abort.");
  }

  cnr_interpolator_interface::JointInputPtr  interpolator_input(new cnr_interpolator_interface::JointInput());
  cnr_interpolator_interface::JointOutputPtr interpolator_output(new cnr_interpolator_interface::JointOutput());
  interpolator_input->override = j_in->get_target_override();
  interpolator_input->time     = m_regulator_time;

//  CNR_INFO_THROTTLE(*m_logger, 20.0
//                    , "SCALED TIME: " + std::to_string(interpolator_input->time.toSec())
//                    + " ovr: "     + std::to_string(interpolator_input->override)
//                    + " scaling: " + std::to_string(j_out->get_scaling())
//                    + " period: "  + std::to_string(m_period.toSec()));
  interpolator->interpolate(interpolator_input, interpolator_output);

  m_regulator_time += m_period * j_out->get_scaling();

  j_out->set_q(interpolator_output->pnt.positions);
  j_out->set_qd(interpolator_output->pnt.velocities);
  j_out->set_qdd(interpolator_output->pnt.accelerations);
  j_out->set_effort(interpolator_output->pnt.effort);
  j_out->set_scaling(j_in->get_target_override());

  j_out->set_in_path_tolerance((m_regulator_time-interpolator->trjTime()).toSec()>0);
//  if( (m_regulator_time-interpolator->trjTime()).toSec()>0 )
//  {
//    CNR_INFO(m_logger, "Yeah, the goal has been achieved (am I right?)");
//    CNR_INFO(m_logger, "Regulator time: " << m_regulator_time );
//    CNR_INFO(m_logger, "Trajectory time: " << interpolator->trjTime() );
//  }
  j_out->set_in_goal_tolerance((m_regulator_time-interpolator->trjTime()).toSec()>0);
  j_out->set_time_from_start(m_regulator_time);

  //CNR_INFO_THROTTLE(*m_logger, 5, "interpolator output velocities: " << j_out->get_qd().transpose() );
  return true;
}


}


