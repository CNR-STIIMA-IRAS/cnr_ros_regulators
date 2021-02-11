#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_STATE__H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_STATE__H

#include <memory>
#include <Eigen/Dense>
#include <rosdyn_core/spacevect_algebra.h>
#include <rosdyn_utilities/chain_state.h>

namespace cnr
{
namespace control
{

/**
 * @brief The BaseRegulatorState struct
 */
struct BaseRegulatorState
{
  BaseRegulatorState() = default;
  virtual ~BaseRegulatorState() = default;
  BaseRegulatorState(const BaseRegulatorState&) = delete;
  BaseRegulatorState& operator=(const BaseRegulatorState&) = default;
  BaseRegulatorState(BaseRegulatorState&&) = delete;
  BaseRegulatorState& operator=(BaseRegulatorState&&) = delete;
};

typedef std::shared_ptr<BaseRegulatorState> BaseRegulatorStatePtr;
typedef std::shared_ptr<BaseRegulatorState const> BaseRegulatorStateConstPtr;

/**
 * @brief JointRegulatorState
 */
template<int N, int MaxN=N>
class JointRegulatorState : public BaseRegulatorState
{
protected:
  rosdyn::ChainState<N,MaxN> robot_state;

public:
  typedef std::shared_ptr<JointRegulatorState> Ptr;
  typedef std::shared_ptr<JointRegulatorState<N,MaxN> const> ConstPtr;

  JointRegulatorState() = default;
  virtual ~JointRegulatorState() = default;
  JointRegulatorState(const JointRegulatorState& cpy)
  {
    *this = cpy;
  }

  JointRegulatorState(JointRegulatorState&&) = delete;
  JointRegulatorState& operator=(JointRegulatorState&&) = delete;

  JointRegulatorState(rosdyn::Chain& kin)
    : robot_state(kin)
  {
  }

  //! no update transform!
  JointRegulatorState& operator=(const JointRegulatorState& rhs)
  {
    robot_state.copy(rhs.robot_state, rhs.robot_state.ONLY_JOINT);
    return *this;
  }

  rosdyn::ChainState<N,MaxN>& robotState()
  {
    return robot_state;
  }

  const rosdyn::ChainState<N,MaxN>& robotState() const
  {
    return robot_state;
  }
};

template<int N, int MaxN=N>
using JointRegulatorStatePtr = typename JointRegulatorState<N,MaxN>::Ptr;

template<int N, int MaxN=N>
using JointRegulatorStateConstPtr = typename JointRegulatorState<N,MaxN>::ConstPtr;


/**
 * @brief The CartesianRegulatorState struct
 */
template<int N, int MaxN=N>
using CartesianRegulatorState = JointRegulatorState<N,MaxN>;

template<int N, int MaxN=N>
using CartesianRegulatorStatePtr = typename CartesianRegulatorState<N,MaxN>::Ptr;

template<int N, int MaxN=N>
using CartesianRegulatorStateConstPtr = typename CartesianRegulatorState<N,MaxN>::ConstPtr;


}  // namespace control
}  // namespace cnr

#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_STATE__H
