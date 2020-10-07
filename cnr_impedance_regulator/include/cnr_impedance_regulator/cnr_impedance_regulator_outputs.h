#ifndef CNR_IMPEDANCE_REGULATOR__CNR_IMPEDANCE_REGULATOR_OUTPUTS__H
#define CNR_IMPEDANCE_REGULATOR__CNR_IMPEDANCE_REGULATOR_OUTPUTS__H


#include <cnr_regulator_interface/cnr_regulator_outputs.h>

namespace cnr_impedance_regulator
{

  
struct ImpedanceRegulatorOutput : public cnr_regulator_interface::BaseRegulatorOutput
{
  size_t nAx;
  
  ImpedanceRegulatorOutput() = delete;
  virtual ~ImpedanceRegulatorOutput() = default;
  ImpedanceRegulatorOutput(const ImpedanceRegulatorOutput&) = delete;
  ImpedanceRegulatorOutput& operator=(const ImpedanceRegulatorOutput&) = delete;
  ImpedanceRegulatorOutput(ImpedanceRegulatorOutput&&) = delete;
  ImpedanceRegulatorOutput& operator=(ImpedanceRegulatorOutput&&) = delete;

  ImpedanceRegulatorOutput(const size_t& n_ax)
    : cnr_regulator_interface::BaseRegulatorOutput(4 + n_ax*4)
  {
    nAx = n_ax;
  }
  virtual void set_time_from_start  (const ros::Duration& time_from_start) { u(0)= time_from_start.toSec()  ; }
  virtual void set_in_goal_tolerance(const bool& in_goal_tolerance       ) { u(1)= in_goal_tolerance        ; }
  virtual void set_in_path_tolerance(const bool& in_path_tolerance       ) { u(2)= in_path_tolerance        ; }
  virtual void set_scaling          (const double& scaling               ) { u(3)= scaling                  ; }
  virtual void set_x                (const Eigen::VectorXd& x            ) { u.segment(0*nAx+4,nAx) = x     ; }
  virtual void set_xd               (const Eigen::VectorXd& xd           ) { u.segment(1*nAx+4,nAx) = xd    ; }
  virtual void set_xdd              (const Eigen::VectorXd& xdd          ) { u.segment(2*nAx+4,nAx) = xdd   ; }
  virtual void set_effort           (const Eigen::VectorXd& effort       ) { u.segment(3*nAx+4,nAx) = effort; }

  virtual void set_x(const std::vector<double>& x)
  {
    u.segment(0*nAx+4,nAx) = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(&x[0],nAx);
  }
  virtual void set_xd(const std::vector<double>& xd)
  {
    u.segment(1*nAx+4,nAx) = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(&xd[0],nAx);
  }
  virtual void set_xdd(const std::vector<double>& xdd)
  {
    u.segment(2*nAx+4,nAx) = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(&xdd[0],nAx);
  }
  virtual void set_effort(const std::vector<double>& effort)
  {
    u.segment(3*nAx+4,nAx) = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(&effort[0],nAx);
  }

  virtual ros::Duration   get_time_from_start  ( ) const { return ros::Duration(u(0))  ;  }
  virtual bool            get_in_goal_tolerance( ) const { return u(1); }
  virtual bool            get_in_path_tolerance( ) const { return u(2); }
  virtual double          get_scaling          ( ) const { return u(3); }
  virtual Eigen::VectorXd get_x                ( ) const { return u.segment(0*nAx+4,nAx); }
  virtual Eigen::VectorXd get_xd               ( ) const { return u.segment(1*nAx+4,nAx); }
  virtual Eigen::VectorXd get_xdd              ( ) const { return u.segment(2*nAx+4,nAx); }
  virtual Eigen::VectorXd get_effort           ( ) const { return u.segment(3*nAx+4,nAx); }
};


typedef std::shared_ptr<ImpedanceRegulatorOutput> ImpedanceRegulatorOutputPtr;
typedef const std::shared_ptr<ImpedanceRegulatorOutput const> RegulatorOutputConstPtr;

}  // namespace cnr_impedance_regulator

#endif  // CNR_IMPEDANCE_REGULATOR__CNR_IMPEDANCE_REGULATOR_OUTPUTS__H
