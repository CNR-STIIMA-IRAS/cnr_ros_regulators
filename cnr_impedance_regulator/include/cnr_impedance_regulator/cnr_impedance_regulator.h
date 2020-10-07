#ifndef CNR_IMPEDANCE_REGULATOR__CNR_IMPEDANCE_REGULATOR__H
#define CNR_IMPEDANCE_REGULATOR__CNR_IMPEDANCE_REGULATOR__H

#include <Eigen/Core>
#include <cnr_interpolator_interface/cnr_interpolator_trajectory.h>
#include <cnr_regulator_interface/cnr_regulator_interface.h>
#include <cnr_impedance_regulator/cnr_impedance_regulator_options.h>
#include <cnr_impedance_regulator/cnr_impedance_regulator_state.h>
#include <cnr_impedance_regulator/cnr_impedance_regulator_inputs.h>
#include <cnr_impedance_regulator/cnr_impedance_regulator_outputs.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>

namespace cnr_impedance_regulator
{

typedef cnr_regulator_interface::RegulatorInterface<ImpedanceRegulatorOptions, 
                                                    ImpedanceRegulatorState,
                                                    ImpedanceRegulatorInput,
                                                    ImpedanceRegulatorOutput> BaseImpedanceRegulator;
 
class ImpedanceRegulator: public BaseImpedanceRegulator
{
public:

  ImpedanceRegulator() = default;
  virtual ~ImpedanceRegulator() = default;
  ImpedanceRegulator(const ImpedanceRegulator&) = delete;
  ImpedanceRegulator& operator=(const ImpedanceRegulator&) = delete;
  ImpedanceRegulator(ImpedanceRegulator&&) = delete;
  ImpedanceRegulator& operator=(ImpedanceRegulator&&) = delete;

  virtual bool initialize(ros::NodeHandle&   root_nh,
                          ros::NodeHandle&   controller_nh,
                          cnr_regulator_interface::BaseRegulatorOptionsConstPtr opts);

  bool update(cnr_interpolator_interface::InterpolatorInterfacePtr,
              cnr_regulator_interface::BaseRegulatorInputConstPtr input,
              cnr_regulator_interface::BaseRegulatorOutputPtr output);

  virtual bool starting(cnr_regulator_interface::BaseRegulatorStateConstPtr state0, const ros::Time& time);
  
private:
  
  Eigen::VectorXd m_Jinv;
  Eigen::VectorXd m_damping;
  Eigen::VectorXd m_damping_dafault;
  Eigen::VectorXd m_k;
  Eigen::VectorXd m_k_default;
  Eigen::VectorXd m_k_new;

};


typedef std::shared_ptr<ImpedanceRegulator> ImpedanceRegulatorPtr;
typedef const std::shared_ptr<ImpedanceRegulator const > ImpedanceRegulatorConstPtr;



}  // namespace cnr_impedance_regulator


#endif  // CNR_IMPEDANCE_REGULATOR__CNR_IMPEDANCE_REGULATOR__H
