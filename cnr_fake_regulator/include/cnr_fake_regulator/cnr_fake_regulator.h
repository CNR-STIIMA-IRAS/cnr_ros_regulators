#ifndef CNR_FAKE_REGULATOR__CNR_FAKE_REGULATOR__H
#define CNR_FAKE_REGULATOR__CNR_FAKE_REGULATOR__H

#include <cnr_interpolator_interface/cnr_interpolator_trajectory.h>
#include <cnr_regulator_interface/cnr_regulator_base.h>

namespace cnr_fake_regulator
{

class FakeRegulator: public cnr_regulator_interface::RegulatorBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FakeRegulator();
  ~FakeRegulator();

  virtual bool initialize(ros::NodeHandle&   root_nh,
                          ros::NodeHandle&   controller_nh,
                          cnr_logger::TraceLoggerPtr logger,
                          cnr_controller_interface::KinematicsStructPtr kin,
                          cnr_controller_interface::KinematicStatusPtr  state);

  bool update(cnr_interpolator_interface::InterpolatorInterfacePtr interpolator,
              cnr_regulator_interface::RegulatorInputBaseConstPtr input,
              cnr_regulator_interface::RegulatorOutputBasePtr output);

  virtual bool starting(const ros::Time& time);

};


typedef std::shared_ptr<FakeRegulator> FakeRegulatorPtr;
typedef const std::shared_ptr<FakeRegulator const > FakeRegulatorConstPtr;



}  // namespace cnr_fake_regulator


#endif  // CNR_FAKE_REGULATOR__CNR_FAKE_REGULATOR__H
