#ifndef CNR_IMPEDANCE_REGULATOR__CNR_IMPEDANCE_REGULATOR__H
#define CNR_IMPEDANCE_REGULATOR__CNR_IMPEDANCE_REGULATOR__H

#include <type_traits>
#include <Eigen/Core>
#include <cnr_interpolator_interface/cnr_interpolator_trajectory.h>
#include <cnr_regulator_interface/cnr_regulator_interface.h>
#include <cnr_impedance_regulator/cnr_impedance_regulator_state.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>

namespace cnr
{
namespace control
{

template<int N, int MaxN=N>
using BaseImpedanceRegulatorN = RegulatorInterfaceN<BaseRegulatorParams,
                                                  ImpedanceRegulatorState<N,MaxN>,
                                                  JointRegulatorReference<N,MaxN>,
                                                  JointRegulatorControlCommand<N,MaxN>,
                                                  JointRegulatorFeedback<N,MaxN> >;

template<int N, int MaxN=N>
class ImpedanceRegulatorN : public BaseImpedanceRegulatorN<N,MaxN>
{
public:
  typedef std::shared_ptr<ImpedanceRegulatorN> Ptr;
  typedef std::shared_ptr<ImpedanceRegulatorN const > ConstPtr;

  ImpedanceRegulatorN() = default;
  virtual ~ImpedanceRegulatorN() = default;
  ImpedanceRegulatorN(const ImpedanceRegulatorN&) = delete;
  ImpedanceRegulatorN& operator=(const ImpedanceRegulatorN&) = delete;
  ImpedanceRegulatorN(ImpedanceRegulatorN&&) = delete;
  ImpedanceRegulatorN& operator=(ImpedanceRegulatorN&&) = delete;

  virtual bool initialize(ros::NodeHandle&   root_nh,
                          ros::NodeHandle&   controller_nh,
                          BaseRegulatorParamsPtr opts) override;

  bool update(BaseRegulatorReferenceConstPtr r,
              BaseRegulatorControlCommandPtr u) override;

  virtual bool starting(BaseRegulatorStateConstPtr state0, const ros::Time& time) override;

  
private:
  using MatrixN = typename std::conditional<N==1, double, Eigen::Matrix<double,N,N,Eigen::ColMajor,MaxN,MaxN> >::type;
  MatrixN m_Jinv;
  MatrixN m_damping;
  MatrixN m_damping_dafault;
  MatrixN m_k;
  MatrixN m_k_default;
  MatrixN m_k_new;
};

template<int N, int MaxN>
using ImpedanceRegulatorNPtr = typename ImpedanceRegulatorN<N,MaxN>::Ptr;

template<int N, int MaxN>
using ImpedanceRegulatorNConstPtr = typename ImpedanceRegulatorN<N,MaxN>::ConstPtr;


using ImpedanceRegulator = ImpedanceRegulatorN<-1, 20>;
using ImpedanceRegulator1 = ImpedanceRegulatorN<1>;
using ImpedanceRegulator3 = ImpedanceRegulatorN<3>;
using ImpedanceRegulator6 = ImpedanceRegulatorN<6>;
using ImpedanceRegulator7 = ImpedanceRegulatorN<7>;

}  // namespace control
}  // namespace cnr

#include <cnr_impedance_regulator/internal/cnr_impedance_regulator_impl.h>

#endif  // CNR_IMPEDANCE_REGULATOR__CNR_IMPEDANCE_REGULATOR__H
