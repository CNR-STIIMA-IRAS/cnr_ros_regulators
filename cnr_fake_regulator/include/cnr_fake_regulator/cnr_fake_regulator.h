#ifndef CNR_FAKE_REGULATOR__CNR_FAKE_REGULATOR__H
#define CNR_FAKE_REGULATOR__CNR_FAKE_REGULATOR__H

#include <cnr_interpolator_interface/cnr_interpolator_trajectory.h>
#include <cnr_regulator_interface/cnr_regulator_interface.h>

namespace cnr_fake_regulator
{

class FakeRegulator: public cnr_regulator_interface::BaseJointRegulator
{
public:

  FakeRegulator() = default;
  virtual ~FakeRegulator() = default;
  FakeRegulator(const FakeRegulator&) = delete;
  FakeRegulator& operator=(const FakeRegulator&) = delete;
  FakeRegulator(FakeRegulator&&) = delete;
  FakeRegulator& operator=(FakeRegulator&&) = delete;

  virtual bool initialize(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh, 
                          cnr_regulator_interface::BaseRegulatorOptionsConstPtr opts);

  bool update(cnr_interpolator_interface::InterpolatorInterfacePtr,
              cnr_regulator_interface::BaseRegulatorInputConstPtr input,
              cnr_regulator_interface::BaseRegulatorOutputPtr output);

  virtual bool starting(cnr_regulator_interface::BaseRegulatorStatePtr state0, const ros::Time& time);

};


typedef std::shared_ptr<FakeRegulator> FakeRegulatorPtr;
typedef const std::shared_ptr<FakeRegulator const > FakeRegulatorConstPtr;



}  // namespace cnr_fake_regulator


#endif  // CNR_FAKE_REGULATOR__CNR_FAKE_REGULATOR__H
