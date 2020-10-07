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

#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_BASE__H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_BASE__H

#include <ros/node_handle.h>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <cnr_logger/cnr_logger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <cnr_controller_interface/utils/cnr_kinematics_utils.h>
#include <cnr_interpolator_interface/cnr_interpolator_interface.h>
#include <cnr_regulator_interface/cnr_regulator_state.h>
#include <cnr_regulator_interface/cnr_regulator_options.h>
#include <cnr_regulator_interface/cnr_regulator_inputs.h>
#include <cnr_regulator_interface/cnr_regulator_outputs.h>

namespace cnr_regulator_interface
{


class BaseRegulator
{
public:
  BaseRegulator() = default;
  virtual ~BaseRegulator() = default;
  BaseRegulator(const BaseRegulator&) = delete;
  BaseRegulator& operator=(const BaseRegulator&) = delete;
  BaseRegulator(BaseRegulator&&) = delete;
  BaseRegulator& operator=(BaseRegulator&&) = delete;

  virtual bool initialize(ros::NodeHandle&  root_nh, 
                          ros::NodeHandle& controller_nh, 
                          cnr_regulator_interface::BaseRegulatorOptionsConstPtr opts);

  virtual bool starting(cnr_regulator_interface::BaseRegulatorStateConstPtr state0, 
                        const ros::Time& /*time*/);

  virtual bool update(cnr_interpolator_interface::InterpolatorInterfacePtr /*interpolator*/,
                      BaseRegulatorInputConstPtr   /*input*/,
                      BaseRegulatorOutputPtr       /*output*/);

  virtual bool stopping(const ros::Time& /*time*/) {return true;}

  void setRegulatorTime(const ros::Duration& time) { m_regulator_time = time;}
  const ros::Duration& getRegulatorTime() const { return m_regulator_time; }
  
protected:
  
  std::shared_ptr<cnr_logger::TraceLogger>  m_logger;
  ros::Duration                             m_regulator_time;
  ros::Duration                             m_period;
  std::vector<std::string>                  m_controlled_resources;
};

typedef std::shared_ptr<BaseRegulator> BaseRegulatorPtr;
typedef std::shared_ptr<BaseRegulator const> BaseRegulatorConstPtr;
}

#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_BASE__H
