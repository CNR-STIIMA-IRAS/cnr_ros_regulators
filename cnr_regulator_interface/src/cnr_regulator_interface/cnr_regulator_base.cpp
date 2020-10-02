#include <cnr_regulator_interface/cnr_regulator_base.h>
namespace cnr_regulator_interface
{

void get_resource_names(ros::NodeHandle* nh, std::vector<std::string>& names)
{
  std::vector<std::string> alternative_keys =
    {"controlled_resources", "controlled_resource", "controlled_joints", "controlled_joint", "joint_names", "joint_name"};

  names.clear();
  for(auto const & key : alternative_keys)
  {
    if(!nh->getParam(key, names))
    {
      std::string joint_name;
      if(nh->getParam(key, joint_name))
      {
        names.push_back(joint_name);
      }
    }
  }
  return;
}

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

  if(!kin || !state)
  {
    ROS_ERROR("RegulatorBase::initialize Null pointer to null kinematic! Abort.");
    return false;
  }

  m_kin = kin;
  m_kin_state = state;
  m_regulator_time = ros::Duration(0);
  m_period = period;

  get_resource_names(&controller_nh, m_controlled_resources);
  if((m_controlled_resources.size()==1)&&(m_controlled_resources.front() == "all"))
  {
    m_controlled_resources.clear();
    m_controlled_resources = m_kin->jointNames();
  }

  CNR_RETURN_TRUE(*m_logger);
}


}  // namespace cnr_regulator_interface
