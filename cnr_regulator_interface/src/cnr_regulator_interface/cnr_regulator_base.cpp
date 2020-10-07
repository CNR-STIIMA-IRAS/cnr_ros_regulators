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

bool BaseRegulator::initialize(ros::NodeHandle&   root_nh,
                               ros::NodeHandle&   controller_nh,
                               cnr_regulator_interface::BaseRegulatorOptionsConstPtr opts)
{
  if(!opts)
  {
    ROS_ERROR("BaseRegulator::initialize Null pointer to options! Abort.");
    return false;
  }
  if(!opts->logger)
  {
    ROS_ERROR("BaseRegulator::initialize Null pointer to logger! Abort.");
    return false;
  }
  CNR_TRACE_START(opts->logger);
  
  m_regulator_time = ros::Duration(0);
  
  get_resource_names(&controller_nh, m_controlled_resources);
  if((m_controlled_resources.size()==1)&&(m_controlled_resources.front() == "all"))
  {
    m_controlled_resources.clear();
    m_controlled_resources = opts->robot_kin->jointNames();
  }

  CNR_RETURN_TRUE(opts->logger);
}

bool BaseRegulator::starting(cnr_regulator_interface::BaseRegulatorStateConstPtr state0, const ros::Time& time)
{
  if(!state0)
  {
    ROS_ERROR("BaseRegulator::initialize Null pointer to null kinematic! Abort.");
    return false;
  }
  setRegulatorTime(ros::Duration(0.0));
  return true;
}

bool BaseRegulator::update(cnr_interpolator_interface::InterpolatorInterfacePtr interpolator,
                      BaseRegulatorInputConstPtr   input,
                      BaseRegulatorOutputPtr       output)
{
  return true;
}

}  // namespace cnr_regulator_interface
