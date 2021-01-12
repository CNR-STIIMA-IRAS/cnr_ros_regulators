#ifndef CNR_IMPEDANCE_REGULATOR_INTERFACE__CNR_IMPEDANCE_REGULATOR_STATE__H
#define CNR_IMPEDANCE_REGULATOR_INTERFACE__CNR_IMPEDANCE_REGULATOR_STATE__H

#include <memory>
#include <Eigen/Dense>
#include <rosdyn_core/spacevect_algebra.h>
#include <rosdyn_utilities/chain_state.h>
#include <eigen_state_space_systems/utils/operations.h>
#include <cnr_regulator_interface/cnr_regulator_state.h>

namespace eu = eigen_utils;

namespace cnr
{
namespace control
{

template<int N, int MaxN>
struct MassSpringDamperModel
{
  using Value = typename std::conditional<N==1,double, Eigen::Matrix<double,N,1,Eigen::ColMajor,MaxN> >::type;

  Value x;
  Value xd;
  Value xdd;
  Value effort;

  void set_dim(const size_t& dim)
  {
    eu::resize(x     , dim); eu::setZero(x     );
    eu::resize(xd    , dim); eu::setZero(xd    );
    eu::resize(xdd   , dim); eu::setZero(xdd   );
    eu::resize(effort, dim); eu::setZero(effort);
  }

  MassSpringDamperModel& operator=(const MassSpringDamperModel& rhs)
  {
    x = rhs.x;
    xd = rhs.xd;
    xdd = rhs.xdd;
    xdd = rhs.xdd;
    return *this;
  }

  MassSpringDamperModel& operator=(const rosdyn::ChainState<N,MaxN>& status)
  {
    set_dim(status.nAx());
    x = status.q();
    xd = status.qd();
    xdd = status.qdd();
    effort = status.effort();
    return *this;
  }
};

template<int N, int MaxN>
class ImpedanceRegulatorState : public JointRegulatorState<N,MaxN>
{
public:
  typedef std::shared_ptr<ImpedanceRegulatorState> Ptr;
  typedef std::shared_ptr<ImpedanceRegulatorState const> ConstPtr;
  
  ImpedanceRegulatorState() = default;
  virtual ~ImpedanceRegulatorState() = default;
  ImpedanceRegulatorState(const ImpedanceRegulatorState&) = delete;
  ImpedanceRegulatorState(ImpedanceRegulatorState&&) = delete;
  ImpedanceRegulatorState& operator=(ImpedanceRegulatorState&&) = delete;
  
  ImpedanceRegulatorState(rosdyn::ChainInterfacePtr kin) : JointRegulatorState<N,MaxN>(kin) {}
  ImpedanceRegulatorState(const rosdyn::ChainState<N,MaxN>& status) : JointRegulatorState<N,MaxN>(status) {}
  ImpedanceRegulatorState& operator=(const ImpedanceRegulatorState& rhs)
  {
     msd.set_dim(eu::rows(rhs.msd.x));
     msd = rhs.msd;
     return *this;
  }


  MassSpringDamperModel<N,MaxN>& msdState()
  {
    return msd;
  }

  const MassSpringDamperModel<N,MaxN>& msdState() const
  {
    return msd;
  }

protected:
  MassSpringDamperModel<N,MaxN> msd;

};

template<int N, int MaxN>
using ImpedanceRegulatorStatePtr = typename ImpedanceRegulatorState<N,MaxN>::Ptr;

template<int N, int MaxN>
using ImpedanceRegulatorStateConstPtr = typename ImpedanceRegulatorState<N,MaxN>::ConstPtr;


}  // namespace control
}  // namespace cnr

#endif  // CNR_IMPEDANCE_REGULATOR_INTERFACE__CNR_IMPEDANCE_REGULATOR_STATE__H
