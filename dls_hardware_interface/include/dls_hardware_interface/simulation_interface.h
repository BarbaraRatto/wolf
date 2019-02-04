///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////


#ifndef HARDWARE_INTERFACE_SIMULATION_INTERFACE_H
#define HARDWARE_INTERFACE_SIMULATION_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>
#include <Eigen/Dense>


namespace hardware_interface
{

class SimulationHandle
{
public:

  SimulationHandle(): 
    name_()
  {}

  SimulationHandle(const std::string& name,int* xenomai_switch_count,bool* is_robot_real,
          bool* pause,bool* reset, bool* freeze_base, 
          Eigen::Vector3d* ext_force, Eigen::Vector3d* ext_torque):
    name_(name), 
    xenomai_switch_count_(xenomai_switch_count),
    is_robot_real_(is_robot_real),
    pause_(pause),
    reset_(reset),
    freeze_base_(freeze_base),
    ext_force_(ext_force),
    ext_torque_(ext_torque)
  {
    if (!is_robot_real_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. isRobotReal pointer is null.");
    }
    if (!pause_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Pause pointer is null.");
    }
    if (!reset_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Reset pointer is null.");
    }
    if (!freeze_base_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Freeze base pointer is null.");
    }
    if (!ext_force_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. External Force pointer is null.");
    }
    if (!ext_torque_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. External Torque pointer is null.");
    }
    *is_robot_real_ = false;
    *pause_ = true;
    *freeze_base_ = true;
    *reset = false;
    *ext_force_ = Eigen::Vector3d::Zero();
    *ext_torque_ = Eigen::Vector3d::Zero();
  }

  std::string getName() const {return name_;}

  int getXenomaiSwitchCount()  const 
  {
    return *xenomai_switch_count_;
  }

  bool isRobotReal()  const 
  {
    return *is_robot_real_;
  }

  void setPauseCommand(bool pause)
  {
    *pause_ = pause;
  }

  void setResetFlag()
  {
    *reset_ = true;
  }

  bool getFreezeBaseFlag()  const 
  {
    return *freeze_base_;
  }
  void setFreezeBaseFlag(bool freeze_base)
  {
    *freeze_base_ = freeze_base;
  }

  void setExternalDisturbance(Eigen::Vector3d ext_force,Eigen::Vector3d ext_torque)
  {
    *ext_force_ = ext_force;
    *ext_torque_ = ext_torque;
  }

private:
  std::string name_;
  int * xenomai_switch_count_;
  bool* is_robot_real_;
  bool* pause_;
  bool* reset_;
  bool* freeze_base_;
  Eigen::Vector3d* ext_force_;
  Eigen::Vector3d* ext_torque_;


};

/** \brief Hardware interface to support reading the ground truth state. */
class SimulationInterface : public HardwareResourceManager<SimulationHandle> {};

}

#endif // HARDWARE_INTERFACE_SIMULATION_INTERFACE_H