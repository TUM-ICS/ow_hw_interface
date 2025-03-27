/*! \file
 *
 * \author J. Rogelio Guadarrama-Olvera
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 03.05.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#include <ow_hw_interface/hw_interface.h>

namespace ow_hw_interface
{

HwInterface::HwInterface(const ow::Scalar& freq,
             const std::vector<JointHandle>& joint_handles,
             const std::vector<FTHandle>& ft_handles,
             const std::vector<ow::Wrench> &ft_calib,
             const ImuHandle& imu_handle,
             const ow::CartesianPosition& X_imu_base) :
  Base("robot"),
  freq_(freq),
  has_imu_(false),
  joint_handles_(joint_handles),
  ft_handles_(ft_handles),
  ft_sensor_calib_(ft_calib),
  imu_handle_(imu_handle),
  X_imu_base_(X_imu_base),
  q_real_(ow::JointState::Zero()),
  q_cmd_(ow::JointState::Zero()),
  q_last_cmd_(ow::JointState::Zero()),
  imu_sensor_(ow::ImuSensor::Zero()),
  state_diff_(ow::ScalarFiniteDifference::FirstOrderAccurarcyThree(freq_)),
  Q_init_yaw_w_(ow::AngularPosition::Identity())
{
}

HwInterface::~HwInterface()
{
}

bool HwInterface::hasImuSensor() const
{
  return has_imu_;
}

bool HwInterface::init(const ow::Parameter& parameter, ros::NodeHandle& nh)
{
  // read parameters
  ow::Scalar freq_cut  = 10;

  // check joint size
  if(joint_handles_.size() != q_real_.q().size())
  {
    ROS_ERROR("%s::init: Number of joint handles=%ld doesn't match robot dofs=%ld",
              Base::name().c_str(), joint_handles_.size(), q_real_.q().size());
    return false;
  }

  // add force torque sensors
  if(ft_handles_.size() != 2)
  {
    ROS_ERROR("%s::init: Number of force torque handles=%ld is not 2",
              Base::name().c_str(), ft_handles_.size());
    return false;
  }

  // add force torque sensors
  for(size_t i=0; i < ft_handles_.size(); ++i)
  {
    // add sensor
    ft_sensors_.push_back(
      ow::FTSensor(ow::Wrench::Zero(),ft_handles_[i].getName(), ft_sensor_calib_[i]));

    // add filter
    ft_sensors_filters_.push_back( ow::make_unique<FTFilter>(
      ow::ScalarButterWorthFilter::LowPassSecondOrder(freq_, freq_cut)));
  }
  wrenches_.resize(ft_sensors_.size());

  // add imu if available
  if(!imu_handle_.getOrientation())
  {
    has_imu_ = false;
    ROS_WARN("%s::init: No IMU sensor provided", Base::name().c_str());
  }
  else
  {
    // set the data
    has_imu_ = true;
    imu_sensor_.name() = imu_handle_.getName();
    imu_sensor_.X_imu_base() = X_imu_base_;

    // read the inital orientation
    ow::AngularPosition Q_imu_w;
    Q_imu_w.x() = imu_handle_.getOrientation()[0];
    Q_imu_w.y() = imu_handle_.getOrientation()[1];
    Q_imu_w.z() = imu_handle_.getOrientation()[2];
    Q_imu_w.w() = imu_handle_.getOrientation()[3];

    // convert to inital floating base orientiaton
    ow::AngularPosition Q_base_w = Q_imu_w*imu_sensor_.X_imu_base().angular().inverse();

    // save the intial yaw orientation of the base
    Q_init_yaw_w_ = Q_base_w.yawQuaternion();
  }
  
  return true;
}

void HwInterface::update()
{
  // copy joint state
  for(size_t i = 0; i < joint_handles_.size(); ++i)
  {
    q_real_.pos()[i] = joint_handles_[i].getPosition();
    q_real_.vel()[i] = joint_handles_[i].getVelocity();
    q_real_.effort()[i] = joint_handles_[i].getEffort();
    q_last_cmd_.pos()[i] = joint_handles_[i].getCommand();
  }

  // copy and filtered force torque values
  for(size_t i = 0; i < ft_handles_.size(); ++i)
  {
    ow::Wrench& W_raw = ft_sensors_[i].W();

    W_raw.force().x() = ft_handles_[i].getForce()[0];
    W_raw.force().y() = ft_handles_[i].getForce()[1];
    W_raw.force().z() = ft_handles_[i].getForce()[2];

    W_raw.moment().x() = ft_handles_[i].getTorque()[0];
    W_raw.moment().y() = ft_handles_[i].getTorque()[1];
    W_raw.moment().z() = ft_handles_[i].getTorque()[2];

    W_raw -= ft_sensors_[i].WOffset();

    wrenches_[i] = ft_sensors_filters_[i]->update(W_raw);
  }

  // copy imu data
  if(has_imu_)
  {
    ow::AngularPosition& Q_imu_w = imu_sensor_.angularPos();
    ow::AngularVelocity& omega_imu = imu_sensor_.angularVel();
    ow::LinearAcceleration& xpp_imu = imu_sensor_.linearAcc();

    // get the data
    Q_imu_w.x() = imu_handle_.getOrientation()[0];
    Q_imu_w.y() = imu_handle_.getOrientation()[1];
    Q_imu_w.z() = imu_handle_.getOrientation()[2];
    Q_imu_w.w() = imu_handle_.getOrientation()[3];
    omega_imu.x() = imu_handle_.getAngularVelocity()[0];
    omega_imu.y() = imu_handle_.getAngularVelocity()[1];
    omega_imu.z() = imu_handle_.getAngularVelocity()[2];
    xpp_imu.x() = imu_handle_.getLinearAcceleration()[0];
    xpp_imu.y() = imu_handle_.getLinearAcceleration()[1];
    xpp_imu.z() = imu_handle_.getLinearAcceleration()[2];

    // transform imu sensor data to the floating base frame
    // by post multiplication of inv transf between imu mounting wrt base
    // And remove the inital yaw orientation by pre multipliation with inverse
    // inital yaw orientation

    // Q_b_w = (Q_init_yaw_w)^-1 * Q_imu_w * (Q_imu_base)^-1
    imu_sensor_.angularPos() = 
      Q_init_yaw_w_.inverse() * Q_imu_w * imu_sensor_.X_imu_base().angular().inverse(); 

    // rotate the angular velocity into base frame
    imu_sensor_.angularVel() = imu_sensor_.X_imu_base().angular() * omega_imu;

    // rotate the linear velocity into the base frame
    imu_sensor_.linearAcc() = imu_sensor_.X_imu_base().angular() * xpp_imu;
  }
}

void HwInterface::updateCommand(const ow::JointPosition& q_cmd)
{
  // compute the finite derivatives
  q_cmd_.pos() = q_cmd;
  q_cmd_ = state_diff_.update(q_cmd_);
}

const ow::JointState& HwInterface::lastJointStateCommand() const
{
  return q_last_cmd_;
}

const ow::JointState& HwInterface::jointStateCommand() const
{
  return q_cmd_;
}

const ow::JointState& HwInterface::jointStateReal() const
{
  return q_real_;
}

const ow::ImuSensor& HwInterface::imu() const
{
  return imu_sensor_;
}

const ow::Wrench& HwInterface::forceTorqueLeft() const
{
  return wrenches_[0];
}

const ow::Wrench& HwInterface::forceTorqueRight() const
{
  return wrenches_[1];
}

}
