#ifndef CNR_FAKE_REGULATOR__CNR_FAKE_REGULATOR__H
#define CNR_FAKE_REGULATOR__CNR_FAKE_REGULATOR__H

#include <cnr_interpolator_interface/cnr_interpolator_trajectory.h>
#include <cnr_regulator_interface/cnr_regulator_base.h>

namespace cnr_fake_regulator
{

class FakeRegulator: public cnr_regulator_interface::RegulatorBase
{
public:

  FakeRegulator() = default;
  virtual ~FakeRegulator() = default;
  FakeRegulator(const FakeRegulator&) = delete;
  FakeRegulator& operator=(const FakeRegulator&) = delete;
  FakeRegulator(FakeRegulator&&) = delete;
  FakeRegulator& operator=(FakeRegulator&&) = delete;

  virtual bool initialize(ros::NodeHandle&   root_nh,
                          ros::NodeHandle&   controller_nh,
                          cnr_logger::TraceLoggerPtr logger,
                          cnr_controller_interface::KinematicsStructPtr kin,
                          cnr_controller_interface::KinematicStatusPtr  state,
                          const ros::Duration &period);

  bool update(cnr_interpolator_interface::InterpolatorInterfacePtr,
              cnr_regulator_interface::RegulatorInputBaseConstPtr input,
              cnr_regulator_interface::RegulatorOutputBasePtr output);

  virtual bool starting(const ros::Time& time);

};


typedef std::shared_ptr<FakeRegulator> FakeRegulatorPtr;
typedef const std::shared_ptr<FakeRegulator const > FakeRegulatorConstPtr;



}  // namespace cnr_fake_regulator


#endif  // CNR_FAKE_REGULATOR__CNR_FAKE_REGULATOR__H
