#ifndef FOOT_STATE_MACHINE_H
#define FOOT_STATE_MACHINE_H

#include <atomic>

namespace wb_controller
{

class FootStateMachine
{
public:

  FootStateMachine()
  {
    // Inputs
    duty_factor_ = 0.8; // It is defined as T_stance / T_cycle
    swing_frequency_ = 0.3;
    reset();
    cycle_ended_ = true; // We set true so that the swing can be triggered
    updatePeriods();
    state_ = prev_state_ = states::STANCE;
  }

  bool isSwing()
  {
    if(state_ == states::SWING)
      return true;
    else
      return false;
  }

  bool isStance()
  {
    if(state_ == states::STANCE)
      return true;
    else
      return false;
  }

  bool isStateChanged()
  {
    if(prev_state_ != state_)
      return true;
    else
      return false;
  }

  bool isTouchDown()
  {
    if(prev_state_ == states::SWING && state_ == states::STANCE)
      return true;
    else
      return false;
  }

  bool isLiftOff()
  {
    if(prev_state_ == states::STANCE && state_ == states::SWING)
      return true;
    else
      return false;
  }

  bool isCycleEnded()
  {
    if(cycle_ended_)
      return true;
    else
      return false;
  }

  void triggerSwing()
  {
    trigger_swing_ = true;
  }

  void setDutyFactor(double duty_factor)
  {
    assert(duty_factor >= 0 && duty_factor <1);
    duty_factor_ = duty_factor;
  }

  void setSwingFrequency(double swing_frequency)
  {
    assert(swing_frequency >= 0);
    swing_frequency_ = swing_frequency;
  }

  double getDutyFactor()
  {
    return duty_factor_;
  }

  void update(const double& period, const bool& contact)
  {

    prev_state_ = state_;

    updatePeriods();

    switch (state_)
    {

    case states::STANCE:

      stance_time_+=period;

      if(stance_time_ >= T_stance_)
        cycle_ended_ = true;

      if(trigger_swing_ && cycle_ended_)
      {
        state_ = states::SWING;
        cycle_ended_ = false;
      }

      break;

    case states::SWING:

      swing_time_+=period;

      //if(swing_frequency_>0)
      //  half_swing_time = 1/(2*swing_frequency_); // NOTE: since we use f'=2f, we should divide by 4 and not by 2.
      // If swing frequency is geq than 0
      // deactivate the contact sensing for half of the swing period
      if(contact && swing_time_>=T_swing_/2.0)
      {
        state_ = states::STANCE;
        reset();
      }

      break;

    default:
      break;

    };
  }

private:

  void reset()
  {
    stance_time_ = 0.0;
    swing_time_ = 0.0;
    trigger_swing_ = false;
  }

  void updatePeriods()
  {
    T_swing_ = 1/swing_frequency_;
    T_stance_ = duty_factor_/(1 - duty_factor_) * T_swing_;
  }

  double swing_time_;
  double stance_time_;
  double T_stance_;
  double T_swing_;
  bool trigger_swing_;
  std::atomic<double> swing_frequency_;
  std::atomic<double> duty_factor_;
  std::atomic<bool> cycle_ended_;

  enum states {SWING=0,STANCE};
  unsigned int state_;
  unsigned int prev_state_;

};

} // namespace

#endif
