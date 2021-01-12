#ifndef CNR_FAKE_REGULATOR__CNR_FAKE_REGULATOR__H
#define CNR_FAKE_REGULATOR__CNR_FAKE_REGULATOR__H

#include <cnr_interpolator_interface/cnr_interpolator_trajectory.h>
#include <cnr_regulator_interface/cnr_regulator_interface.h>

namespace cnr
{
namespace control
{

template<int N, int MaxN=N>
class FakeRegulatorN: public BaseJointRegulator<N,MaxN>
{
public:

  FakeRegulatorN() = default;
  virtual ~FakeRegulatorN() = default;
  FakeRegulatorN(const FakeRegulatorN&) = delete;
  FakeRegulatorN& operator=(const FakeRegulatorN&) = delete;
  FakeRegulatorN(FakeRegulatorN&&) = delete;
  FakeRegulatorN& operator=(FakeRegulatorN&&) = delete;

  virtual bool initialize(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh, 
                          BaseRegulatorParamsPtr opts) override;

  virtual bool update(BaseRegulatorReferenceConstPtr r,
                      BaseRegulatorFeedbackConstPtr  y,
                      BaseRegulatorControlCommandPtr u) override
  {
    CNR_ERROR_THROTTLE(this->logger(),10.0, "The regulator does not need any feedback!");
    return update(r, u);
  }

  bool update(BaseRegulatorFeedbackConstPtr  y,
              BaseRegulatorControlCommandPtr u) override
  {
    CNR_ERROR(this->logger(), "The regulator needs the reference override, while it does not use the feedback!");
    CNR_RETURN_FALSE(this->logger());
  }

  bool update(BaseRegulatorReferenceConstPtr r,
              BaseRegulatorControlCommandPtr u) override;

  
  bool update(BaseRegulatorControlCommandPtr u) override
  {
    CNR_ERROR(this->logger(), "The regulator needs the reference override!");
    CNR_RETURN_FALSE(this->logger());
  }

  virtual bool starting(BaseRegulatorStateConstPtr state0, const ros::Time& time) override;

};


template<int N, int MaxN>
using FakeRegulatorNPtr = typename FakeRegulatorN<N,MaxN>::Ptr;

template<int N, int MaxN>
using FakeRegulatorNConstPtr = typename FakeRegulatorN<N,MaxN>::ConstPtr;


using FakeRegulator  = FakeRegulatorN<-1, 20>;
using FakeRegulator1 = FakeRegulatorN<1>;
using FakeRegulator3 = FakeRegulatorN<3>;
using FakeRegulator6 = FakeRegulatorN<6>;
using FakeRegulator7 = FakeRegulatorN<7>;

}  // namespace control
}  // namespace cnr


#include <cnr_fake_regulator/internal/cnr_fake_regulator_impl.h>

#endif  // CNR_FAKE_REGULATOR__CNR_FAKE_REGULATOR__H
