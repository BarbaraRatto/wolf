/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <memory>
#include <vector>
#include <map>
#include <atomic>

namespace wolf_controller {

class Controller;
class StateMachine;

class QuadrupedRobotState {
public:
  typedef std::shared_ptr<QuadrupedRobotState> Ptr;

  virtual ~QuadrupedRobotState() = default;
  virtual void updateStateMachine(StateMachine* state_machine, const double& dt) = 0;
};

class QuadrupedRobotIdleState : public QuadrupedRobotState {
public:
  void updateStateMachine(StateMachine* state_machine, const double& dt) override;
};

class QuadrupedRobotInitState : public QuadrupedRobotState {
public:
  void updateStateMachine(StateMachine* state_machine, const double& dt) override;
};

class QuadrupedRobotStandingUpState : public QuadrupedRobotState {
public:
  void updateStateMachine(StateMachine* state_machine, const double& dt) override;
};

class QuadrupedRobotActiveState : public QuadrupedRobotState {
public:
  void updateStateMachine(StateMachine* state_machine, const double& dt) override;
};

class QuadrupedRobotStandingDownState : public QuadrupedRobotState {
public:
  void updateStateMachine(StateMachine* state_machine, const double& dt) override;
};

class QuadrupedRobotAnomalyState : public QuadrupedRobotState {
public:
  void updateStateMachine(StateMachine* state_machine, const double& dt) override;
};

class StateMachine
{
public:

  const std::string CLASS_NAME = "StateMachine";

  typedef std::shared_ptr<StateMachine> Ptr;

  typedef std::shared_ptr<const StateMachine> ConstPtr;

  enum robot_states_t {IDLE,INIT,ANOMALY,STANDING_UP,STANDING_DOWN,ACTIVE,N_STATES=7};

  StateMachine(Controller* controller);

  void updateStateMachine(const double& dt);

  void setCurrentState(robot_states_t state);

  robot_states_t getCurrentState();

  robot_states_t getPreviousState();

  std::string getStateAsString();

  std::vector<std::string> getStatesAsString();

  Controller* getController();

private:
  Controller* controller_;
  std::atomic<robot_states_t> current_state_;
  std::atomic<robot_states_t> previous_state_;
  std::string current_state_str_;
  std::map<robot_states_t,QuadrupedRobotState::Ptr> states_;
};

}

#endif
