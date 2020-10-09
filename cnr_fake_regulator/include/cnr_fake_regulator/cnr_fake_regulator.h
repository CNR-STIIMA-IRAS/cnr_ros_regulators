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
                          cnr_regulator_interface::BaseRegulatorParamsPtr opts) override;

  virtual bool update(cnr_regulator_interface::BaseRegulatorReferenceConstPtr r,
                      cnr_regulator_interface::BaseRegulatorFeedbackConstPtr  y,
                      cnr_regulator_interface::BaseRegulatorControlCommandPtr u) override
  {
    CNR_ERROR_THROTTLE(logger(),10.0, "The regulator does not need any feedback!");
    return update(r, u);
  }

  bool update(cnr_regulator_interface::BaseRegulatorFeedbackConstPtr  y,
              cnr_regulator_interface::BaseRegulatorControlCommandPtr u) override
  {
    CNR_ERROR(logger(), "The regulator needs the reference override, while it does not use the feedback!");
    CNR_RETURN_FALSE(logger());
  }

  bool update(cnr_regulator_interface::BaseRegulatorReferenceConstPtr r,
              cnr_regulator_interface::BaseRegulatorControlCommandPtr u) override;

  
  bool update(cnr_regulator_interface::BaseRegulatorControlCommandPtr u) override
  {
    CNR_ERROR(logger(), "The regulator needs the reference override!");
    CNR_RETURN_FALSE(logger());
  }

  virtual bool starting(cnr_regulator_interface::BaseRegulatorStateConstPtr state0, const ros::Time& time) override;

};


typedef std::shared_ptr<FakeRegulator> FakeRegulatorPtr;
typedef std::shared_ptr<FakeRegulator const > FakeRegulatorConstPtr;



}  // namespace cnr_fake_regulator


#endif  // CNR_FAKE_REGULATOR__CNR_FAKE_REGULATOR__H
