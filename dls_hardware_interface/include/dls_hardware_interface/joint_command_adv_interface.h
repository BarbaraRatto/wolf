#ifndef HARDWARE_INTERFACE_JOINT_COMMAND_ADV_INTERFACE_H
#define HARDWARE_INTERFACE_JOINT_COMMAND_ADV_INTERFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <dls_hardware_interface/joint_state_adv_interface.h>

namespace hardware_interface
{

/** \brief A handle used to read and command a single joint. */
class JointCommandAdvHandle : public JointStateAdvHandle
{
public:
  JointCommandAdvHandle() : JointStateAdvHandle(), cmd_pos_(0), cmd_vel_(0), cmd_eff_(0),
    cmd_p_gains_(0),cmd_i_gains_(0),cmd_d_gains_(0){}

  /**
   * \param js This joint's state handle
   * \param cmd_pos A pointer to the storage for this joint's output command position
   * \param cmd_vel A pointer to the storage for this joint's output command velocity
   */
  JointCommandAdvHandle(const JointStateAdvHandle& js, double* cmd_pos, double* cmd_vel,
      double* cmd_eff, double* cmd_p_gains, double* cmd_i_gains, double* cmd_d_gains)
    : JointStateAdvHandle(js), cmd_pos_(cmd_pos), cmd_vel_(cmd_vel), cmd_eff_(cmd_eff),
        cmd_p_gains_(cmd_p_gains), cmd_i_gains_(cmd_i_gains), cmd_d_gains_(cmd_d_gains)
  {
    if (!cmd_pos)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command position pointer is null.");
    }
    if (!cmd_vel)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command velocity  pointer is null.");
    }
    if (!cmd_eff)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command effort  pointer is null.");
    }
    if (!cmd_p_gains)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command p gains  pointer is null.");
    }
    if (!cmd_i_gains)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command i gains  pointer is null.");
    }
    if (!cmd_d_gains)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command d gains  pointer is null.");
    }
  }

  void setCommand(double cmd_pos, double cmd_vel, double cmd_eff)
  {
    setCommandPosition(cmd_pos);
    setCommandVelocity(cmd_vel);
    setCommandEffort(cmd_vel);
  }

  void setCommandGains(double cmd_p_gains, double cmd_i_gains, double cmd_d_gains)
  {
    setCommandPGain(cmd_p_gains);
    setCommandIGain(cmd_i_gains);
    setCommandDGain(cmd_d_gains);
  }

  void setCommandPosition(double cmd_pos)     {assert(cmd_pos_); *cmd_pos_ = cmd_pos;}
  void setCommandVelocity(double cmd_vel)     {assert(cmd_vel_); *cmd_vel_ = cmd_vel;}
  void setCommandEffort(double cmd_eff)       {assert(cmd_eff_); *cmd_eff_ = cmd_eff;}

  void setCommandPGain(double cmd_p_gains)     {assert(cmd_p_gains_); *cmd_p_gains_ = cmd_p_gains;}
  void setCommandIGain(double cmd_i_gains)     {assert(cmd_i_gains_); *cmd_i_gains_ = cmd_i_gains;}
  void setCommandDGain(double cmd_d_gains)     {assert(cmd_d_gains_); *cmd_d_gains_ = cmd_d_gains;}


  double getCommandPosition()     const {assert(cmd_pos_); return *cmd_pos_;}
  double getCommandVelocity()     const {assert(cmd_vel_); return *cmd_vel_;}
  double getCommandEffort()     const {assert(cmd_eff_); return *cmd_eff_;}

  double getCommandPGain()     const {assert(cmd_p_gains_); return *cmd_p_gains_;}
  double getCommandIGain()     const {assert(cmd_i_gains_); return *cmd_i_gains_;}
  double getCommandDGain()     const {assert(cmd_d_gains_); return *cmd_d_gains_;}

private:
  double* cmd_pos_;
  double* cmd_vel_;
  double* cmd_eff_;
  double* cmd_p_gains_;
  double* cmd_i_gains_;
  double* cmd_d_gains_;
};

/** \brief Hardware interface to support commanding an array of joints.
 *
 * This \ref HardwareInterface supports commanding joints by position, velocity
 * together in one command.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
class JointCommandAdvInterface : public HardwareResourceManager<JointCommandAdvHandle, ClaimResources> {};

}

#endif /*HARDWARE_INTERFACE_JOINT_COMMAND_ADV_INTERFACE_H*/
