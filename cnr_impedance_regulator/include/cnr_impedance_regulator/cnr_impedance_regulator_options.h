#ifndef CNR_IMPEDANCE_REGULATOR_INTERFACE__CNR_IMPEDANCE_REGULATOR_OPTIONS__H
#define CNR_IMPEDANCE_REGULATOR_INTERFACE__CNR_IMPEDANCE_REGULATOR_OPTIONS__H

#include <memory>
#include <eigen3/Eigen/Dense>
#include <rosdyn_core/spacevect_algebra.h>
#include <rosdyn_core/chain_state.h>
#include <cnr_regulator_interface/cnr_regulator_params.h>

namespace cnr_impedance_regulator
{

struct ImpedanceRegulatorParams : public cnr_regulator_interface::BaseRegulatorParams
{
  typedef std::shared_ptr<ImpedanceRegulatorParams> Ptr;
  typedef std::shared_ptr<ImpedanceRegulatorParams const> ConstPtr;
  
  ImpedanceRegulatorParams() = default;
  virtual ~ImpedanceRegulatorParams() = default;
  ImpedanceRegulatorParams(const ImpedanceRegulatorParams&) = delete;
  ImpedanceRegulatorParams(ImpedanceRegulatorParams&&) = delete;
  
  ImpedanceRegulatorParams& operator=(ImpedanceRegulatorParams&&) = delete;
  ImpedanceRegulatorParams(const size_t nAx) : BaseRegulatorParams(nAx) {};
  
};

typedef ImpedanceRegulatorParams::Ptr ImpedanceRegulatorParamsPtr;
typedef ImpedanceRegulatorParams::ConstPtr ImpedanceRegulatorParamsConstPtr;


}  // namespace cnr_impedance_regulator

#endif  // CNR_IMPEDANCE_REGULATOR_INTERFACE__CNR_IMPEDANCE_REGULATOR_OPTIONS__H
