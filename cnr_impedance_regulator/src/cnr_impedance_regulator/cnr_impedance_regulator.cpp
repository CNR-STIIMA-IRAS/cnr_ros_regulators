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

using cnr_impedance_regulator::ImpedanceRegulatorInput;
using cnr_impedance_regulator::ImpedanceRegulatorInputConstPtr;
using cnr_impedance_regulator::ImpedanceRegulatorOutput;
using cnr_impedance_regulator::ImpedanceRegulatorOutputPtr;

bool ImpedanceRegulator::initialize(ros::NodeHandle&  root_nh,
                                    ros::NodeHandle&  controller_nh,
                                    cnr_regulator_interface::BaseRegulatorOptionsConstPtr opts)
{
  if(!BaseImpedanceRegulator::initialize(root_nh,controller_nh,opts))
  {
    return false;
  }
  
  try
  {     
    m_Jinv.resize(m_opts.dim);
    m_damping.resize(m_opts.dim);
    m_damping_dafault.resize(m_opts.dim);
    m_k.resize(m_opts.dim);
    m_k_default.resize(m_opts.dim);
    m_k_new.resize(m_opts.dim);

    
    std::vector<double> inertia(m_opts.dim,0), damping(m_opts.dim,0), stiffness(m_opts.dim,0), eff_deadband(m_opts.dim,0);
    GET_PARAM_VECTOR_AND_RETURN(controller_nh, "inertia"  , inertia  , m_opts.dim);
    GET_PARAM_VECTOR_AND_RETURN(controller_nh, "stiffness", stiffness, m_opts.dim);

    if (controller_nh.hasParam("damping_ratio"))
    {
      std::vector<double> damping_ratio;
      GET_PARAM_VECTOR_AND_RETURN(controller_nh,  "damping_ratio", damping_ratio, m_opts.dim);

      damping.resize(m_opts.dim,0);
      for (unsigned int iAx=0; iAx<m_opts.dim; iAx++)
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
      GET_PARAM_VECTOR_AND_RETURN(controller_nh, "damping", damping, m_opts.dim);
    }

    
    for (unsigned int iAx=0;iAx<m_opts.dim;iAx++)
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
    CNR_ERROR(m_logger, "EXCEPTION: " << e.what());
    CNR_RETURN_FALSE(m_logger);
  }
  CNR_RETURN_TRUE(m_logger);
}

bool ImpedanceRegulator::starting(cnr_regulator_interface::BaseRegulatorStateConstPtr state0, const ros::Time& time)
{
  if(!BaseImpedanceRegulator::starting(state0, time))
  {
    CNR_RETURN_FALSE(m_logger);
  }
  cnr_impedance_regulator::ImpedanceRegulatorStateConstPtr  x0 = 
                      std::dynamic_pointer_cast< cnr_impedance_regulator::ImpedanceRegulatorState const>(state0);
  m_state0->setRobotState(x0->getRobotState());
  m_state ->setRobotState(x0->getRobotState());
  
  CNR_RETURN_TRUE(m_logger);
}


bool ImpedanceRegulator::update(cnr_interpolator_interface::InterpolatorInterfacePtr interpolator,
                                cnr_regulator_interface::BaseRegulatorInputConstPtr  input,
                                cnr_regulator_interface::BaseRegulatorOutputPtr      output)
{
  if(!BaseImpedanceRegulator::update(interpolator, input, output))
  {
    CNR_RETURN_FALSE(m_logger);
  }
    
  m_state->model->xdd = m_Jinv.cwiseProduct( m_k.cwiseProduct(m_input.get_x() - m_state->model->x) 
                     + m_damping.cwiseProduct(m_input.get_xd()  - m_state->model->xd) + m_input.get_effort( ));
  m_state->model->x  += m_state->model->xd  * m_period.toSec() + m_state->model->xdd*std::pow(m_period.toSec(),2.0)*0.5;
  m_state->model->xd += m_state->model->xdd * m_period.toSec();
  
  m_regulator_time  += m_period;

  m_output->set_x(m_state->model->x);
  m_output->set_xd(m_state->model->xd);
  m_output->set_xdd(m_state->model->xdd);
  m_output->set_time_from_start(m_regulator_time);

  //CNR_INFO_THROTTLE(*m_logger, 5, "interpolator output velocities: " << j_out->get_qd().transpose() );
  return true;
}


}


