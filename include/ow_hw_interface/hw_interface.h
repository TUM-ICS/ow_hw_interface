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

#ifndef OPEN_WALKER_ROBOT_H
#define OPEN_WALKER_ROBOT_H

// ow
#include <ow_core/math.h>
#include <ow_core/algorithms.h>
#include <ow_core/types.h>
#include <ow_core/common/smart_ptr.h>
#include <ow_core/interfaces/i_hw_interface.h>

// ros control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_state_interface.h>

#include <fstream>

/*!
 * \brief Open Walker hardware interface module namespace. These classes
 * implement the interface to get sensor data form the ros_control framework.
 */
namespace ow_hw_interface
{

/*!
 * \brief The Robot class
 *
 * This class implements the generic interface between the open walker
 * framework and ros_control plugin. It get recives the real robot state/
 * sensor reading from the ow_ros_control_plugin and passes the information 
 * to ow_controllers.
 * 
 */
class HwInterface :
  public ow::IHwInterface
{
public:
  typedef ow::IHwInterface Base;
  typedef ow_core::MatrixAlgorithm<ow::Wrench> FTFilter; 

  typedef hardware_interface::ForceTorqueSensorHandle FTHandle;
  typedef hardware_interface::ImuSensorHandle ImuHandle;
  typedef hardware_interface::JointHandle JointHandle;

protected:
  double freq_;               //!< Update rate [Hz].
  bool has_imu_;              //!< True when the robot has an IMU sensor.

  // robot state
  ow::JointState q_real_;                   //!<  Real robot state.
  ow::JointState q_cmd_;                    //!<  Commanded robot state.
  ow::JointState q_last_cmd_;               //!<  Last commanded joint statte

  // sensor readings
  ow::ImuSensor imu_sensor_;                //!< IMU sensor data.
  ow::AngularPosition Q_init_yaw_w_;        //!< Inital orientation around floating base z-axis
  ow::CartesianPosition X_imu_base_;        //!< IMU to robot base pose

  std::vector<ow::FTSensor> ft_sensors_;      //!< FT sensors.
  std::vector<ow::Wrench> ft_sensor_calib_;   //!< Filtered FT sensors.
  std::vector<ow::Wrench> wrenches_;          //!< Filtered FT sensors.

  // algorithms
  ow::StateDifferentiator<ow::JointState> state_diff_;          //!< Commanded state derivative.
  std::vector<std::unique_ptr<FTFilter> > ft_sensors_filters_;  //!< Force torque filters.

  // ros control plugin handles
  const std::vector<JointHandle>& joint_handles_;
  const std::vector<FTHandle>& ft_handles_;
  const ImuHandle& imu_handle_;

public:
  HwInterface(
    const ow::Scalar& freq,
    const std::vector<JointHandle>& joint_handles,
    const std::vector<FTHandle>& ft_handles,
    const std::vector<ow::Wrench>& ft_calib,
    const ImuHandle& imu_handle,
    const ow::CartesianPosition& X_imu_base);

  virtual ~HwInterface();

  /**
   * @brief updates the class
   * 
   * Reads the information from ros control and updates internal variables.
   */
  void update();

  /**
   * @brief send a new joint command to the robot
   * 
   * @param q_cmd the commanded jointposition
   */
  void updateCommand(const ow::JointPosition &q_cmd);

  /**
   * @brief checks if the robot has an imu sensor
   * 
   * @return true 
   * @return false 
   */
  virtual bool hasImuSensor() const;

  /**
   * @brief get the last commanded joint state
   * 
   * @return const ow::JointState& the commanded joint state
   */
  virtual const ow::JointState& lastJointStateCommand() const;

  /**
   * @brief get the current commanded jointstate
   * 
   * @return const ow::JointState& 
   */
  virtual const ow::JointState& jointStateCommand() const;

  /**
   * @brief get the current real robot jointstate                          
   * 
   * @return const ow::JointState& 
   */
  virtual const ow::JointState& jointStateReal() const;

  /*!
  * \brief Output port function.
  *
  * \return
  *    ImuSensor of the robot.
  */
  const ow::ImuSensor& imu() const;

  /*!
  * \brief Output port function.
  *
  * \return
  *    force torque wrench of the robot.
  */
  const ow::Wrench& forceTorqueRight() const;

  /*!
  * \brief Output port function.
  *
  * \return
  *    force torque wrench of the robot.
  */
  const ow::Wrench& forceTorqueLeft() const;

protected:
  /*!
   * \brief Initialization of COMEstimator module
   */
  virtual bool init(const ow::Parameter& parameter, ros::NodeHandle& nh);

};

} // namespace ow_hw_interface

#endif // OPEN_WALKER_HW_INTERFACE_H
