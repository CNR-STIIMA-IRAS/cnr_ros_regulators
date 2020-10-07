#ifndef CNR_IMPEDANCE_REGULATOR_INTERFACE__CNR_IMPEDANCE_REGULATOR_OPTIONS__H
#define CNR_IMPEDANCE_REGULATOR_INTERFACE__CNR_IMPEDANCE_REGULATOR_OPTIONS__H

#include <memory>
#include <eigen3/Eigen/Dense>
#include <rosdyn_core/spacevect_algebra.h>
#include <cnr_controller_interface/utils/cnr_kinematics_utils.h>
#include <cnr_regulator_interface/cnr_regulator_options.h>

namespace cnr_impedance_regulator
{

struct ImpedanceRegulatorOptions : public cnr_regulator_interface::BaseRegulatorOptions
{
  typedef std::shared_ptr<ImpedanceRegulatorOptions> Ptr;
  typedef const std::shared_ptr<ImpedanceRegulatorOptions const> ConstPtr;
  
  ImpedanceRegulatorOptions() = default;
  virtual ~ImpedanceRegulatorOptions() = default;
  ImpedanceRegulatorOptions(const ImpedanceRegulatorOptions&) = delete;
  ImpedanceRegulatorOptions(ImpedanceRegulatorOptions&&) = delete;
  
  ImpedanceRegulatorOptions& operator=(ImpedanceRegulatorOptions&&) = delete;
  ImpedanceRegulatorOptions(const size_t nAx) : BaseRegulatorOptions(nAx) {};
  
};

typedef ImpedanceRegulatorOptions::Ptr ImpedanceRegulatorOptionsPtr;
typedef ImpedanceRegulatorOptions::ConstPtr ImpedanceRegulatorOptionsConstPtr;


}  // namespace cnr_impedance_regulator

#endif  // CNR_IMPEDANCE_REGULATOR_INTERFACE__CNR_IMPEDANCE_REGULATOR_OPTIONS__H
