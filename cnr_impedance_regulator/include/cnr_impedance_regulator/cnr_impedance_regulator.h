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

typedef cnr_regulator_interface::RegulatorInterface<ImpedanceRegulatorParams, 
                                                    ImpedanceRegulatorState,
                                                    ImpedanceRegulatorReference,
                                                    ImpedanceRegulatorControlCommand,
                                                    cnr_regulator_interface::JointRegulatorFeedback
                                                    > BaseImpedanceRegulator;
 
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
                          cnr_regulator_interface::BaseRegulatorParamsPtr opts) override;

  bool update(cnr_regulator_interface::BaseRegulatorReferenceConstPtr r,
              cnr_regulator_interface::BaseRegulatorControlCommandPtr u) override;

  virtual bool starting(cnr_regulator_interface::BaseRegulatorStateConstPtr state0, const ros::Time& time) override;
  
private:
  
  Eigen::VectorXd m_Jinv;
  Eigen::VectorXd m_damping;
  Eigen::VectorXd m_damping_dafault;
  Eigen::VectorXd m_k;
  Eigen::VectorXd m_k_default;
  Eigen::VectorXd m_k_new;

};


typedef std::shared_ptr<ImpedanceRegulator> ImpedanceRegulatorPtr;
typedef std::shared_ptr<ImpedanceRegulator const > ImpedanceRegulatorConstPtr;



}  // namespace cnr_impedance_regulator


#endif  // CNR_IMPEDANCE_REGULATOR__CNR_IMPEDANCE_REGULATOR__H
