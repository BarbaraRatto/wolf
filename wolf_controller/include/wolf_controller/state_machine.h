#include <memory>
#include <wolf_controller/controller.h>

namespace wolf_controller {

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

  enum robot_states_t {IDLE,INIT,ANOMALY,STANDING_UP,STANDING_DOWN,ACTIVE,N_STATES=7};

  StateMachine(Controller* controller);

  void updateStateMachine(const double& dt);

  void setCurrentState(robot_states_t state);

  robot_states_t getCurrentState();

  robot_states_t getPreviousState();

  Controller* getController();

private:
  Controller* controller_;
  robot_states_t current_state_;
  robot_states_t previous_state_;
  std::map<robot_states_t,QuadrupedRobotState::Ptr> states_;
};

}
