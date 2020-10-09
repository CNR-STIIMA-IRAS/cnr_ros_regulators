#include <cnr_impedance_regulator/cnr_impedance_regulator.h>
#include <cnr_interpolator_interface/cnr_interpolator_inputs.h>
#include <cnr_interpolator_interface/cnr_interpolator_outputs.h>
#include <cnr_impedance_regulator/cnr_impedance_regulator_inputs.h>
#include <cnr_impedance_regulator/cnr_impedance_regulator_outputs.h>

#include <pluginlib/class_list_macros.h> // header for PLUGINLIB_EXPORT_CLASS. NOTE IT SHOULD STAY IN THE CPP FILE NOTE

PLUGINLIB_EXPORT_CLASS(cnr_impedance_regulator::ImpedanceRegulator, cnr_regulator_interface::BaseRegulator)


inline std::string to_string( const std::vector<std::string>& vals )
{
  std::string ret = "< ";
  for( auto const & val : vals ) ret += val + ", ";
  ret += " >";
  return ret;
}

inline std::string to_string( const std::string& val )
{
  return val;
}

inline std::string to_string( const bool& val )
{
  return val ? "TRUE" : "FALSE";
}




#define GET_AND_RETURN(nh, param, value)\
  if (!nh.getParam(param,value) )\
  {\
    ROS_ERROR("The param '%s/%s' is not defined", nh.getNamespace().c_str(), std::string( param ).c_str() );\
    return false;\
  }



#define GET_AND_DEFAULT(nh, param, value, def)\
  if (!nh.getParam(param,value) )\
  {\
    ROS_WARN("The param '%s/%s' is not defined", nh.getNamespace().c_str(), std::string( param ).c_str() );\
    ROS_WARN("Default value '%s' superimposed. ", std::to_string( def ).c_str() );\
    value=def;\
  }
  
#define GET_PARAM_VECTOR_AND_RETURN(nh, P, X , N)\
  if (!controller_nh.getParam( std::string(P).c_str(), X))\
  {\
    ROS_FATAL_STREAM("[ " << nh.getNamespace() << "] Parameter '"<<  P <<"' does not exist");\
    ROS_FATAL_STREAM("[ " << nh.getNamespace() << "] ERROR DURING INITIALIZATION. ABORT.");\
    return false;\
  }\
  if( X.size() != N )\
  {\
    ROS_FATAL_STREAM("[ " << nh.getNamespace() << "] The size '"<< X.size() <<"' of the param '" << P << "' does not match with the foreseen dimension '"<< N <<"'");\
    ROS_FATAL_STREAM("[ " << nh.getNamespace() << "] ERROR DURING INITIALIZATION. ABORT.");\
    return false;\
  }


namespace cnr_impedance_regulator
{

using cnr_impedance_regulator::ImpedanceRegulatorReference;
using cnr_impedance_regulator::ImpedanceRegulatorReferenceConstPtr;
using cnr_impedance_regulator::ImpedanceRegulatorControlCommand;
using cnr_impedance_regulator::ImpedanceRegulatorControlCommandPtr;

bool ImpedanceRegulator::initialize(ros::NodeHandle&  root_nh,
                                    ros::NodeHandle&  controller_nh,
                                    cnr_regulator_interface::BaseRegulatorParamsPtr opts)
{
  if(!BaseImpedanceRegulator::initialize(root_nh,controller_nh,opts))
  {
    return false;
  }
  
  try
  {     
    m_Jinv.resize(dim());
    m_damping.resize(dim());
    m_damping_dafault.resize(dim());
    m_k.resize(dim());
    m_k_default.resize(dim());
    m_k_new.resize(dim());

    
    std::vector<double> inertia(dim(),0), damping(dim(),0), stiffness(dim(),0), eff_deadband(dim(),0);
    GET_PARAM_VECTOR_AND_RETURN(controller_nh, "inertia"  , inertia  , dim());
    GET_PARAM_VECTOR_AND_RETURN(controller_nh, "stiffness", stiffness, dim());

    if (controller_nh.hasParam("damping_ratio"))
    {
      std::vector<double> damping_ratio;
      GET_PARAM_VECTOR_AND_RETURN(controller_nh,  "damping_ratio", damping_ratio, dim());

      damping.resize(dim(),0);
      for (unsigned int iAx=0; iAx<dim(); iAx++)
      {
        if (stiffness.at(iAx)<=0)
        {
          ROS_ERROR("damping ratio can be specified only for positive stiffness values (stiffness of Joint is not positive)");
          return false;
        }
        damping.at(iAx)=2*damping_ratio.at(iAx)*std::sqrt(stiffness.at(iAx)*inertia.at(iAx));
      }
    }
    else
    {
      GET_PARAM_VECTOR_AND_RETURN(controller_nh, "damping", damping, dim());
    }

    
    for (unsigned int iAx=0;iAx<dim();iAx++)
    {
      if (inertia.at(iAx)<=0)
      {
        ROS_INFO("inertia value of Joint %d is not positive, disabling impedance control for this axis",iAx);
        m_Jinv(iAx)=0.0;
      }
      else
        m_Jinv(iAx)=1.0/inertia.at(iAx);
      
      if (damping.at(iAx)<=0)
      {
        ROS_INFO("damping value of Joint %d is not positive, setting equalt to 10/inertia",iAx);
        m_damping(iAx)         = 10.0 * m_Jinv(iAx);
        m_damping_dafault(iAx) = 10.0 * m_Jinv(iAx);
      }
      else
      {
        m_damping(iAx)          = damping.at(iAx);
        m_damping_dafault(iAx)  = damping.at(iAx);
      }
      
      
      if (stiffness.at(iAx)<0)
      {
        ROS_INFO("maximum fitness value of Joint %d is negative, setting equal to 0",iAx);
        m_k(iAx)=0.0;
        m_k_default(iAx)=0.0;
      }
      else
      {
        m_k(iAx)=stiffness.at(iAx);
        m_k_default(iAx)=stiffness.at(iAx);
      }
    }
  }
  catch(const  std::exception& e)
  {
    CNR_ERROR(logger(), "EXCEPTION: " << e.what());
    CNR_RETURN_FALSE(logger());
  }
  CNR_RETURN_TRUE(logger());
}

bool ImpedanceRegulator::starting(cnr_regulator_interface::BaseRegulatorStateConstPtr state0, const ros::Time& time)
{
  if(!BaseImpedanceRegulator::starting(state0, time))
  {
    CNR_RETURN_FALSE(logger());
  }
  x()->setRobotState(kin());
  CNR_RETURN_TRUE(logger());
}


bool ImpedanceRegulator::update(cnr_regulator_interface::BaseRegulatorReferenceConstPtr _r,
                                cnr_regulator_interface::BaseRegulatorControlCommandPtr _u)
{
  if(!BaseImpedanceRegulator::update(_r, _u))
  {
    CNR_RETURN_FALSE(logger());
  }
    
  x()->model->xdd = m_Jinv.cwiseProduct( m_k.cwiseProduct(r()->get_x() - x()->model->x) 
                     + m_damping.cwiseProduct(r()->get_xd() - x()->model->xd) + r()->get_effort( ));
  x()->model->x  += x()->model->xd  * period().toSec() + x()->model->xdd*std::pow(period().toSec(),2.0)*0.5;
  x()->model->xd += x()->model->xdd * period().toSec();
  
  regulator_time_  += period();

  u()->set_x(x()->model->x);
  u()->set_xd(x()->model->xd);
  u()->set_xdd(x()->model->xdd);
  u()->set_time_from_start(regulator_time_);

  return true;
}


}


