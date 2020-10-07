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
#include <cnr_controller_interface/utils/cnr_kinematics_utils.h>
#include <cnr_interpolator_interface/cnr_interpolator_interface.h>
#include <cnr_regulator_interface/cnr_regulator_base.h>

namespace cnr_regulator_interface
{


template< class OPTS, class STATE, class INPUT, class OUTPUT>
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
                          cnr_regulator_interface::BaseRegulatorOptionsConstPtr opts)
  {
    if(!cnr_regulator_interface::BaseRegulator::initialize(root_nh,controller_nh, opts))
    {
      return false;
    }
    
    CNR_TRACE_START(opts->logger);
    const std::shared_ptr<OPTS const > _opts = std::dynamic_pointer_cast<OPTS const>(opts);
    if(!opts)
    {
      CNR_RETURN_TRUE(opts->logger, "Recast failure. Check the objects type. Abort.");
    }
    this->m_opts << opts;
    
    CNR_RETURN_TRUE(opts->logger);
  }

  virtual bool starting(cnr_regulator_interface::BaseRegulatorStateConstPtr state0, 
                        const ros::Time& time)
  {
    CNR_TRACE_START(this->m_opts.logger);
    if(!cnr_regulator_interface::BaseRegulator::starting(state0, time))
    {
      CNR_RETURN_FALSE(this->m_opts.logger);
    }
    const std::shared_ptr<STATE const> st0 = std::dynamic_pointer_cast<STATE const>(state0);
    if(!st0)
    {
      CNR_RETURN_FALSE(this->m_opts.logger, "Recast failure. Check the objects type. Abort.");
    }
    this->m_state0.reset( new STATE( ) );
    this->m_state0->setRobotState(*st0->getRobotState());
    
    this->m_state.reset( new STATE( ) );
    this->m_state->setRobotState(*st0->getRobotState());
    CNR_RETURN_TRUE(this->m_opts.logger);
  }

  virtual bool update(cnr_interpolator_interface::InterpolatorInterfacePtr  interpolator,
                      cnr_regulator_interface::BaseRegulatorInputConstPtr   input,
                      cnr_regulator_interface::BaseRegulatorOutputPtr       output)
  {
    CNR_TRACE_START(this->m_opts.logger);
    if(!cnr_regulator_interface::BaseRegulator::update(interpolator, input, output))
    {
      CNR_RETURN_FALSE(this->m_opts.logger);
    }
    const std::shared_ptr<INPUT const > in = std::dynamic_pointer_cast<INPUT const>(input);
    if(!in)
    {
      CNR_RETURN_FALSE(this->m_opts.logger, "Recast failure. Check the objects type. Abort.");
    }
    m_input = *in;
    
    std::shared_ptr<OUTPUT> out = std::dynamic_pointer_cast<OUTPUT>(output);
    if(!out)
    {
      CNR_RETURN_FALSE(this->m_opts.logger, "Recast failure. Check the objects type. Abort.");
    }
    m_output = out;
    
    CNR_RETURN_TRUE(this->m_opts.logger);
  }

  virtual bool stopping(const ros::Time& time) 
  {
    CNR_TRACE_START(this->m_opts.logger);
    if(!cnr_regulator_interface::BaseRegulator::stopping(time))
    {
      CNR_RETURN_FALSE(this->m_opts.logger);
    }
    CNR_RETURN_TRUE(this->m_opts.logger);
  }

  void setRegulatorTime(const ros::Duration& time) { this->m_regulator_time = time;}
  const ros::Duration& getRegulatorTime() const { return this->m_regulator_time; }
  
  
  const std::shared_ptr<STATE const>  getState0() const {return m_state0;}
  const std::shared_ptr<STATE const>  getState () const {return m_state;}
  
protected:
  
  OPTS m_opts;
  std::shared_ptr<STATE>  m_state0;
  std::shared_ptr<STATE>  m_state;
  INPUT                   m_input;
  std::shared_ptr<OUTPUT> m_output;
};

typedef RegulatorInterface<cnr_regulator_interface::JointRegulatorOptions,
                           cnr_regulator_interface::JointRegulatorState,
                           cnr_regulator_interface::JointRegulatorInput,
                           cnr_regulator_interface::JointRegulatorOutput > BaseJointRegulator;
                           
typedef RegulatorInterface<cnr_regulator_interface::CartesianRegulatorOptions,
                           cnr_regulator_interface::CartesianRegulatorState,
                           cnr_regulator_interface::CartesianRegulatorInput,
                           cnr_regulator_interface::CartesianRegulatorOutput> BaseCartesianRegulator;

}

#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INTERFACE__H
