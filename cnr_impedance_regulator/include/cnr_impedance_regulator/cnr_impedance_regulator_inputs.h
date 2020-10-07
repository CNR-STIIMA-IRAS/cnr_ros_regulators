#ifndef CNR_IMPEDANCE_REGULATOR__CNR_IMPEDANCE_REGULATOR_INPUTS__H
#define CNR_IMPEDANCE_REGULATOR__CNR_IMPEDANCE_REGULATOR_INPUTS__H

#include <memory>
#include <Eigen/Core>
#include <cnr_regulator_interface/cnr_regulator_inputs.h>



namespace cnr_impedance_regulator
{
  
struct ImpedanceRegulatorInput : public cnr_regulator_interface::BaseRegulatorInput
{
  ImpedanceRegulatorInput() = default;
  virtual ~ImpedanceRegulatorInput() = default;
  ImpedanceRegulatorInput(const ImpedanceRegulatorInput&) = delete;
  ImpedanceRegulatorInput(ImpedanceRegulatorInput&&) = delete;
  ImpedanceRegulatorInput& operator=(ImpedanceRegulatorInput&&) = delete;

  ImpedanceRegulatorInput(size_t nAx): cnr_regulator_interface::BaseRegulatorInput( (nAx*4) ){}

  void set_x             (const Eigen::VectorXd& x     ) { u.segment(6+0*dim,dim) = x     ; }
  void set_xd            (const Eigen::VectorXd& xd    ) { u.segment(6+1*dim,dim) = xd    ; }
  void set_xdd           (const Eigen::VectorXd& xdd   ) { u.segment(6+2*dim,dim) = xdd   ; }
  void set_effort        (const Eigen::VectorXd& effort) { u.segment(6+3*dim,dim) = effort; }

  Eigen::VectorXd get_x     ( ) const { return u.segment(6+0*dim,dim); }
  Eigen::VectorXd get_xd    ( ) const { return u.segment(6+1*dim,dim); }
  Eigen::VectorXd get_xdd   ( ) const { return u.segment(6+2*dim,dim); }
  Eigen::VectorXd get_effort( ) const { return u.segment(6+3*dim,dim); }

  ImpedanceRegulatorInput& operator=(const ImpedanceRegulatorInput& rhs)
  {
    this->dim = rhs.dim;
    this->u   = rhs.u;
    return *this;
  }
};

typedef std::shared_ptr<ImpedanceRegulatorInput> ImpedanceRegulatorInputPtr;
typedef const std::shared_ptr<ImpedanceRegulatorInput const> ImpedanceRegulatorInputConstPtr;
  

}  // namespace cnr_joint_impedance_regulator

#endif  // CNR_IMPEDANCE_REGULATOR__CNR_IMPEDANCE_REGULATOR_INPUTS__H
