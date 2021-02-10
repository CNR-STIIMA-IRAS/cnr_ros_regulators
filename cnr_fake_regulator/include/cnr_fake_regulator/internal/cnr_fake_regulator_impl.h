#pragma once

#ifndef CNR_FAKE_REGULATOR__CNR_FAKE_REGULATOR_IMPL__H
#define CNR_FAKE_REGULATOR__CNR_FAKE_REGULATOR_IMPL__H

#include <eigen_matrix_utils/overloads.h>
#include <cnr_fake_regulator/cnr_fake_regulator.h>
#include <cnr_interpolator_interface/cnr_interpolator_inputs.h>
#include <cnr_interpolator_interface/cnr_interpolator_outputs.h>
#include <cnr_regulator_interface/cnr_regulator_references.h>
#include <cnr_regulator_interface/cnr_regulator_control_commands.h>

namespace eu =  eigen_utils;

namespace cnr
{

namespace control
{

template<int N,int MaxN>
inline bool FakeRegulatorN<N,MaxN>::initialize(ros::NodeHandle&            root_nh,
                               ros::NodeHandle&            controller_nh, 
                               BaseRegulatorParamsPtr param)
{
  if(!BaseJointRegulator<N,MaxN>::initialize(root_nh,controller_nh,param))
  {
    return false;
  }
  CNR_RETURN_TRUE(this->logger());
}

template<int N,int MaxN>
inline bool FakeRegulatorN<N,MaxN>::starting(BaseRegulatorStateConstPtr state0, const ros::Time& time)
{
  CNR_TRACE_START(this->logger());
  if(!BaseJointRegulator<N,MaxN>::starting(state0, time))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE(this->logger());
}


template<int N,int MaxN>
inline bool FakeRegulatorN<N,MaxN>::update(BaseRegulatorReferenceConstPtr _r,
                           BaseRegulatorControlCommandPtr _u)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  if(!BaseJointRegulator<N,MaxN>::update(_r,_u))
  {
    CNR_RETURN_FALSE(this->logger());
  }

  JointInputPtr  interpolator_input(new JointInput());
  JointOutputPtr interpolator_output(new JointOutput());
  interpolator_input->override() = this->r()->target_override;
  interpolator_input->time()     = this->regulator_time_;

  if(!this->interpolator())
  {
    CNR_RETURN_FALSE(this->logger());
  }
  if(!this->interpolator()->interpolate(interpolator_input, interpolator_output))
  {
    CNR_RETURN_FALSE(this->logger());
  }

  this->regulator_time_ += this->period() * this->u()->scaling;

  eu::copy(this->u()->x, interpolator_output->pnt.positions);
  eu::copy(this->u()->xd, interpolator_output->pnt.velocities);
  eu::copy(this->u()->xdd, interpolator_output->pnt.accelerations);
  eu::copy(this->u()->eff, interpolator_output->pnt.effort);
  this->u()->scaling = this->r()->target_override;
  this->u()->in_path_tolerance = (this->regulator_time_-this->interpolator()->trjTime()).toSec()>0;
  this->u()->in_goal_tolerance = (this->regulator_time_-this->interpolator()->trjTime()).toSec()>0;
  this->u()->time_from_start = this->regulator_time_;

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}


}  // namespace control
}  // namespace cnr

#endif // CNR_FAKE_REGULATOR__CNR_FAKE_REGULATOR_IMPL__H


