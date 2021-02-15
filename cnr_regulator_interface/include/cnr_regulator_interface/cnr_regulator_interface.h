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
#include <rosdyn_utilities/chain_state.h>
#include <cnr_interpolator_interface/cnr_interpolator_interface.h>
#include <cnr_regulator_interface/internal/cnr_regulator_base.h>

#if !defined(MAX_NUM_AXES) || MAX_NUM_AXES==0
  #define MAX_NUM_AXES 20
#endif

namespace cnr
{
namespace control
{

constexpr int max_dof = MAX_NUM_AXES;


template< class P, class X, class R, class U, class Y>
class RegulatorInterfaceN : public BaseRegulator
{
public:
  RegulatorInterfaceN() = default;
  virtual ~RegulatorInterfaceN() = default;
  RegulatorInterfaceN(const RegulatorInterfaceN&) = delete;
  RegulatorInterfaceN& operator=(const RegulatorInterfaceN&) = delete;
  RegulatorInterfaceN(RegulatorInterfaceN&&) = delete;
  RegulatorInterfaceN& operator=(RegulatorInterfaceN&&) = delete;

  virtual bool initialize(ros::NodeHandle&  root_nh,
                          ros::NodeHandle& controller_nh,
                          BaseRegulatorParamsPtr opts) override;
  virtual bool starting(BaseRegulatorStateConstPtr x0,
                        const ros::Time& time) override;

  virtual bool update(BaseRegulatorReferenceConstPtr   r,
                      BaseRegulatorFeedbackConstPtr    y,
                      BaseRegulatorControlCommandPtr   u) override;

  virtual bool update(BaseRegulatorReferenceConstPtr   r,
                      BaseRegulatorControlCommandPtr   u) override;

  virtual bool update(BaseRegulatorFeedbackConstPtr   y,
                      BaseRegulatorControlCommandPtr  u) override;

  virtual bool update(BaseRegulatorControlCommandPtr  u) override;

  virtual bool stopping(const ros::Time& time);

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
  std::shared_ptr<R const > get(BaseRegulatorReferenceConstPtr r);
  std::shared_ptr<Y const> get(BaseRegulatorFeedbackConstPtr y);
  std::shared_ptr<U> get(BaseRegulatorControlCommandPtr u);
};


template<int N,int MaxN=N>
using __BaseJointRegulatorN = RegulatorInterfaceN<JointRegulatorParams, JointRegulatorState<N,MaxN>,
                                JointRegulatorReference<N,MaxN>, JointRegulatorControlCommand<N,MaxN>,
                                  JointRegulatorFeedback<N,MaxN> >;

template<int N, int MaxN=N>
class BaseJointRegulatorN : public __BaseJointRegulatorN<N,MaxN>
{
public:
  BaseJointRegulatorN() = default;
  virtual ~BaseJointRegulatorN() = default;
  BaseJointRegulatorN(const BaseJointRegulatorN&) = delete;
  BaseJointRegulatorN& operator=(const BaseJointRegulatorN&) = delete;
  BaseJointRegulatorN(BaseJointRegulatorN&&) = delete;
  BaseJointRegulatorN& operator=(BaseJointRegulatorN&&) = delete;

  bool starting(BaseRegulatorStateConstPtr state0, const ros::Time& time) override;
};

using BaseJointRegulator  = BaseJointRegulatorN<-1, cnr::control::max_dof>;
using BaseJointRegulator1 = BaseJointRegulatorN<1>;
using BaseJointRegulator3 = BaseJointRegulatorN<3>;
using BaseJointRegulator6 = BaseJointRegulatorN<6>;
using BaseJointRegulator7 = BaseJointRegulatorN<7>;

template<int N,int MaxN=N>
using __BaseCartesianRegulatorN = RegulatorInterfaceN<CartesianRegulatorParams,
                                                    CartesianRegulatorState<N,MaxN>, // depends on ChainState
                                                    CartesianRegulatorReference,
                                                    JointRegulatorControlCommand<N,MaxN>,
                                                    JointRegulatorFeedback<N,MaxN> >;

template<int N,int MaxN=N>
class BaseCartesianRegulatorN : public  __BaseCartesianRegulatorN<N,MaxN>
{
public:
  BaseCartesianRegulatorN() = default;
  virtual ~BaseCartesianRegulatorN() = default;
  BaseCartesianRegulatorN(const BaseCartesianRegulatorN&) = delete;
  BaseCartesianRegulatorN& operator=(const BaseCartesianRegulatorN&) = delete;
  BaseCartesianRegulatorN(BaseCartesianRegulatorN&&) = delete;
  BaseCartesianRegulatorN& operator=(BaseCartesianRegulatorN&&) = delete;

  bool starting(BaseRegulatorStateConstPtr state0, const ros::Time& time) override;
};

using BaseCartesianRegulator  = BaseCartesianRegulatorN<-1, cnr::control::max_dof>;
using BaseCartesianRegulator1 = BaseCartesianRegulatorN<1>;
using BaseCartesianRegulator3 = BaseCartesianRegulatorN<3>;
using BaseCartesianRegulator6 = BaseCartesianRegulatorN<6>;
using BaseCartesianRegulator7 = BaseCartesianRegulatorN<7>;

}
}

#include <cnr_regulator_interface/internal/cnr_regulator_interface_impl.h>

#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INTERFACE__H
