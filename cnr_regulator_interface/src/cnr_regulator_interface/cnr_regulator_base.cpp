#include <cnr_regulator_interface/cnr_regulator_base.h>
namespace cnr_regulator_interface
{

bool RegulatorBase::initialize(ros::NodeHandle&   root_nh,
                               ros::NodeHandle&   controller_nh,
                               cnr_logger::TraceLoggerPtr logger,
                               cnr_controller_interface::KinematicsStructPtr kin,
                               cnr_controller_interface::KinematicStatusPtr state)
{
  m_logger = logger;
  m_kin = kin;
  m_state = state;
}


}  // namespace cnr_regulator_interface
