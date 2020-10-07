#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_OPTIONS__H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_OPTIONS__H

#include <memory>
#include <ros/time.h>
#include <cnr_logger/cnr_logger.h>
#include <cnr_controller_interface/utils/cnr_kinematics_utils.h>



namespace cnr_regulator_interface
{


/**
 * @brief The BaseRegulatorOptions struct
 */
struct BaseRegulatorOptions
{
    
  typedef std::shared_ptr<BaseRegulatorOptions> Ptr;
  typedef const std::shared_ptr<BaseRegulatorOptions const> ConstPtr;
  
  BaseRegulatorOptions() = default;
  virtual ~BaseRegulatorOptions() = default;
  BaseRegulatorOptions(const BaseRegulatorOptions&) = delete;
  
  BaseRegulatorOptions(BaseRegulatorOptions&&) = delete;
  BaseRegulatorOptions& operator=(BaseRegulatorOptions&&) = delete;
  
  BaseRegulatorOptions(size_t n_dim) : dim(n_dim){}
  
  void set_dimension(const size_t& n_dim) { dim = n_dim;}
  
  cnr_logger::TraceLoggerPtr logger;
  ros::Duration              period;
  size_t                     dim;
  cnr_controller_interface::KinematicsStructPtr robot_kin;
  
  BaseRegulatorOptions& operator<<(const BaseRegulatorOptions::ConstPtr& rhs)
  {
    this->logger = rhs->logger;
    this->period = rhs->period;
    this->dim = rhs->dim;
    this->robot_kin = rhs->robot_kin;
    return *this;
  }
  
  BaseRegulatorOptions& operator=(const BaseRegulatorOptions& rhs)
  {
    this->logger = rhs.logger;
    this->period = rhs.period;
    this->dim = rhs.dim;
    this->robot_kin = rhs.robot_kin;
    return *this;
  }
  
  const size_t nAx() { return robot_kin->nAx(); }
};

typedef BaseRegulatorOptions::Ptr BaseRegulatorOptionsPtr;
typedef BaseRegulatorOptions::ConstPtr BaseRegulatorOptionsConstPtr;

struct JointRegulatorOptions : public cnr_regulator_interface::BaseRegulatorOptions
{
  JointRegulatorOptions() = default;
  virtual ~JointRegulatorOptions() = default;
  JointRegulatorOptions(const JointRegulatorOptions&) = delete;
  JointRegulatorOptions& operator=(const JointRegulatorOptions&) = delete;
  JointRegulatorOptions(JointRegulatorOptions&&) = delete;
  JointRegulatorOptions& operator=(JointRegulatorOptions&&) = delete;
  
  void set_dof(const size_t& n_dim) { set_dimension(n_dim);}
  
};

typedef std::shared_ptr<JointRegulatorOptions> JointRegulatorOptionsPtr;
typedef const std::shared_ptr<JointRegulatorOptions const> JointRegulatorOptionsConstPtr;


/**
 * @brief The CartesianRegulatorOptions struct
 */
struct CartesianRegulatorOptions : public cnr_regulator_interface::BaseRegulatorOptions
{
  CartesianRegulatorOptions() : BaseRegulatorOptions(6) {};
  virtual ~CartesianRegulatorOptions() = default;
  CartesianRegulatorOptions(const CartesianRegulatorOptions&) = delete;
  CartesianRegulatorOptions& operator=(const CartesianRegulatorOptions&) = delete;
  CartesianRegulatorOptions(CartesianRegulatorOptions&&) = delete;
  CartesianRegulatorOptions& operator=(CartesianRegulatorOptions&&) = delete;

};
typedef std::shared_ptr<CartesianRegulatorOptions> CartesianRegulatorOptionsPtr;
typedef const std::shared_ptr<CartesianRegulatorOptions const> CartesianRegulatorOptionsConstPtr;


}  // namespace cnr_regulator_interface

#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_OPTIONS__H
