/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INTERFACE__H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INTERFACE__H

#include <memory>
#include <ros/node_handle.h>
#include <cnr_logger/cnr_logger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <rosdyn_core/chain_state.h>
#include <cnr_interpolator_interface/cnr_interpolator_interface.h>
#include <cnr_regulator_interface/internal/cnr_regulator_base.h>

namespace cnr_regulator_interface
{


template< class P, class X, class R, class U, class Y>
class RegulatorInterface : public cnr_regulator_interface::BaseRegulator
{
public:
  
  RegulatorInterface() = default;
  virtual ~RegulatorInterface() = default;
  RegulatorInterface(const RegulatorInterface&) = delete;
  RegulatorInterface& operator=(const RegulatorInterface&) = delete;
  RegulatorInterface(RegulatorInterface&&) = delete;
  RegulatorInterface& operator=(RegulatorInterface&&) = delete;

  virtual bool initialize(ros::NodeHandle&  root_nh, 
                          ros::NodeHandle& controller_nh, 
                          cnr_regulator_interface::BaseRegulatorParamsPtr opts) override
  {
    if(!cnr_regulator_interface::BaseRegulator::initialize(root_nh,controller_nh, opts))
    {
      return false;
    }
    
    CNR_TRACE_START(logger());
    const std::shared_ptr<P> _p = std::dynamic_pointer_cast<P>(opts);
    if(!_p)
    {
      CNR_RETURN_TRUE(logger(), "Recast failure. Check the objects type. Abort.");
    }
    
    CNR_RETURN_TRUE(logger());
  }

  virtual bool starting(cnr_regulator_interface::BaseRegulatorStateConstPtr x0, 
                        const ros::Time& time) override
  {
    CNR_TRACE_START(logger());
    std::shared_ptr<X const> _x0 = std::dynamic_pointer_cast<X const>(x0);
    if(!_x0)
    {
      CNR_RETURN_FALSE(logger(), "Recast failure. Check the objects type. Abort.");
    }
    
    if(!cnr_regulator_interface::BaseRegulator::starting(_x0, time))
    {
      CNR_RETURN_FALSE(logger());
    }
    std::shared_ptr<X> x( new X( ) );
    this->x_ = x;
    CNR_RETURN_TRUE(logger());
  }

  virtual bool update(cnr_regulator_interface::BaseRegulatorReferenceConstPtr   r,
                      cnr_regulator_interface::BaseRegulatorFeedbackConstPtr    y,
                      cnr_regulator_interface::BaseRegulatorControlCommandPtr   u) override
  {
    CNR_TRACE_START_THROTTLE_DEFAULT(logger());
    auto _r = get(r);
    auto _y = get(y);
    auto _u = get(u);
    if(!_r || !_y || !_u )
    {
      CNR_RETURN_FALSE(logger());
    }
    if(!cnr_regulator_interface::BaseRegulator::update(_r,_y,_u))
    {
      CNR_RETURN_FALSE(logger());
    }
    CNR_RETURN_TRUE_THROTTLE_DEFAULT(logger());
  }

  virtual bool update(cnr_regulator_interface::BaseRegulatorReferenceConstPtr   r,
                      cnr_regulator_interface::BaseRegulatorControlCommandPtr   u) override
  {
    CNR_TRACE_START_THROTTLE_DEFAULT(logger());
    auto _r = get(r);
    auto _u = get(u);
    if(!_r || !_u )
    {
      CNR_RETURN_FALSE(logger());
    }
    if(!cnr_regulator_interface::BaseRegulator::update(_r,_u))
    {
      CNR_RETURN_FALSE(logger());
    }
    CNR_RETURN_TRUE_THROTTLE_DEFAULT(logger());
  }

  virtual bool update(cnr_regulator_interface::BaseRegulatorFeedbackConstPtr   y,
                      cnr_regulator_interface::BaseRegulatorControlCommandPtr  u) override
  {
    CNR_TRACE_START_THROTTLE_DEFAULT(logger());
    auto _y = get(y);
    auto _u = get(u);
    if(!_y || !_u )
    {
      CNR_RETURN_FALSE(logger());
    }
    if(!cnr_regulator_interface::BaseRegulator::update(_y,_u))
    {
      CNR_RETURN_FALSE(logger());
    }
    CNR_RETURN_TRUE_THROTTLE_DEFAULT(logger());
  }

  virtual bool update(cnr_regulator_interface::BaseRegulatorControlCommandPtr  u) override
  {
    CNR_TRACE_START_THROTTLE_DEFAULT(logger());
    auto _u = get(u);
    if(!_u )
    {
      CNR_RETURN_FALSE(logger());
    }
    if(!cnr_regulator_interface::BaseRegulator::update(_u))
    {
      CNR_RETURN_FALSE(logger());
    }
    CNR_RETURN_TRUE_THROTTLE_DEFAULT(logger());
  }

  virtual bool stopping(const ros::Time& time) 
  {
    CNR_TRACE_START(logger());
    if(!cnr_regulator_interface::BaseRegulator::stopping(time))
    {
      CNR_RETURN_FALSE(logger());
    }
    CNR_RETURN_TRUE(logger());
  }

  void setRegulatorTime(const ros::Duration& time) { this->m_regulator_time = time;}
  const ros::Duration& getRegulatorTime() const { return this->m_regulator_time; }
   
  std::shared_ptr<P const>  p ( ) const { return std::dynamic_pointer_cast<P const>(p_); }
  std::shared_ptr<X const>  x0( ) const { return std::dynamic_pointer_cast<X const>(x0_);}
  std::shared_ptr<X const>  x ( ) const { return std::dynamic_pointer_cast<X const>(x_); }
  std::shared_ptr<X      >  x ( )       { return std::dynamic_pointer_cast<X      >(x_); }
  std::shared_ptr<R const>  r ( ) const { return std::dynamic_pointer_cast<R const>(r_); }
  //std::shared_ptr<R      >  r ( )       { return std::dynamic_pointer_cast<R      >(r_); }
  std::shared_ptr<U const>  u ( ) const { return std::dynamic_pointer_cast<U const>(u_); }
  std::shared_ptr<U      >  u ( )       { return std::dynamic_pointer_cast<U      >(u_); }
  std::shared_ptr<Y const>  y ( ) const { return std::dynamic_pointer_cast<Y const>(y_); }
  std::shared_ptr<Y      >  y ( )       { return std::dynamic_pointer_cast<Y      >(y_); }

protected:

  std::shared_ptr<R const > get(cnr_regulator_interface::BaseRegulatorReferenceConstPtr r)
  {
    std::shared_ptr<R const > _r = std::dynamic_pointer_cast<R const>(r);
    if(!_r)
    {
      CNR_ERROR(logger(), "Recast failure. Check the objects type. Abort.");
    }
    return _r;
  }
  std::shared_ptr<Y const> get(cnr_regulator_interface::BaseRegulatorFeedbackConstPtr y)
  {
    std::shared_ptr<Y const > _y = std::dynamic_pointer_cast<Y const>(y);
    if(!_y)
    {
      CNR_ERROR(logger(), "Recast failure. Check the objects type. Abort.");
    }
    return _y;
  }
  std::shared_ptr<U> get(cnr_regulator_interface::BaseRegulatorControlCommandPtr u)
  {
    std::shared_ptr<U> _u = std::dynamic_pointer_cast<U>(u);
    if(!_u)
    {
      CNR_ERROR(logger(), "Recast failure. Check the objects type. Abort.");
    }
    return _u;
  }
};


using __BaseJointRegulator = RegulatorInterface<cnr_regulator_interface::JointRegulatorParams,
                                                cnr_regulator_interface::JointRegulatorState,
                                                cnr_regulator_interface::JointRegulatorReference,
                                                cnr_regulator_interface::JointRegulatorControlCommand,
                                                cnr_regulator_interface::JointRegulatorFeedback>;

class BaseJointRegulator : public __BaseJointRegulator
{
public:
  BaseJointRegulator() = default;
  virtual ~BaseJointRegulator() = default;
  BaseJointRegulator(const BaseJointRegulator&) = delete;
  BaseJointRegulator& operator=(const BaseJointRegulator&) = delete;
  BaseJointRegulator(BaseJointRegulator&&) = delete;
  BaseJointRegulator& operator=(BaseJointRegulator&&) = delete;

  bool starting(cnr_regulator_interface::BaseRegulatorStateConstPtr state0, const ros::Time& time) override
  {

    CNR_TRACE_START(logger());
    if(!cnr_regulator_interface::__BaseJointRegulator::starting(state0, time))
    {
      CNR_RETURN_FALSE(logger());
    }
    x()->setRobotState(kin());
    CNR_RETURN_TRUE(logger());
  }

};

using __BaseCartesianRegulator = RegulatorInterface<cnr_regulator_interface::CartesianRegulatorParams,
                                                    cnr_regulator_interface::CartesianRegulatorState,
                                                    cnr_regulator_interface::CartesianRegulatorReference,
                                                    cnr_regulator_interface::JointRegulatorControlCommand, 
                                                    cnr_regulator_interface::JointRegulatorFeedback >;
                           
class BaseCartesianRegulator : public  __BaseCartesianRegulator
{
public:
  BaseCartesianRegulator() = default;
  virtual ~BaseCartesianRegulator() = default;
  BaseCartesianRegulator(const BaseCartesianRegulator&) = delete;
  BaseCartesianRegulator& operator=(const BaseCartesianRegulator&) = delete;
  BaseCartesianRegulator(BaseCartesianRegulator&&) = delete;
  BaseCartesianRegulator& operator=(BaseCartesianRegulator&&) = delete;

  bool starting(cnr_regulator_interface::BaseRegulatorStateConstPtr state0, const ros::Time& time)override
  {

    CNR_TRACE_START(logger());
    if(!cnr_regulator_interface::__BaseCartesianRegulator::starting(state0, time))
    {
      CNR_RETURN_FALSE(logger());
    }
    x()->setRobotState(kin());
    CNR_RETURN_TRUE(logger());
  }

};

                        

}

#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INTERFACE__H
