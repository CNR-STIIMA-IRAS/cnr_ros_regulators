#include <cnr_regulator_interface/cnr_regulator_base.h>
namespace cnr_regulator_interface
{

bool RegulatorBase::initialize(ros::NodeHandle&   root_nh,
                               ros::NodeHandle&   controller_nh,
                               cnr_logger::TraceLoggerPtr logger,
                               cnr_controller_interface::KinematicsStructPtr kin,
                               cnr_controller_interface::KinematicStatusPtr state,
                               const ros::Duration &period)
{
  if(!logger)
  {
    ROS_ERROR("RegulatorBase::initialize Null pointer to logger! Abort.");
    return false;
  }
  m_logger = logger;
  CNR_TRACE_START(*m_logger);
  m_kin = kin;
  m_kin_state = state;
  m_regulator_time = ros::Duration(0);
  m_period = period;
  CNR_RETURN_TRUE(*m_logger);
}


}  // namespace cnr_regulator_interface
