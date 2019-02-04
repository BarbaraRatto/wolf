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


#ifndef HARDWARE_MOTOR_INTERFACE_H
#define HARDWARE_MOTOR_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>
#include <Eigen/Dense>


namespace hardware_interface
{

class MotorHandle
{
public:

  MotorHandle(): 
    name_()
  {}

  MotorHandle(const std::string& name,bool* remove_torque_offsets,
              std::vector<double>* misc_sensors,
              std::vector<std::string>* misc_sensors_names):
    name_(name), 
    remove_torque_offsets_(remove_torque_offsets),
    misc_sensors_(misc_sensors),
    misc_sensors_names_(misc_sensors_names)
  {
    if (!remove_torque_offsets_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Remove Torque Offsets pointer is null.");
    }
    if (!misc_sensors_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Misc Sensors pointer is null.");
    }
    if (!misc_sensors_names_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Misc Sensors Names pointer is null.");
    }
    *remove_torque_offsets_ = false;
  }

  std::string getName() const {return name_;}

  void removeOffsets()
  {
    *remove_torque_offsets_ = true;
  }

  std::vector<std::string> getMiscSensorsNames()
  {
    return *misc_sensors_names_;
  }

  std::vector<double> getMiscSensors()
  {
    return *misc_sensors_;
  }

private:
  std::string name_;
  bool* remove_torque_offsets_;
  std::vector<double>* misc_sensors_;
  std::vector<std::string>* misc_sensors_names_;

};

/** \brief Hardware interface to support reading the ground truth state. */
class MotorInterface : public HardwareResourceManager<MotorHandle> {};

}

#endif // HARDWARE_MOTOR_INTERFACE_H
