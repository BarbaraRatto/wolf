#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <atomic>
#include <wb_controller/geometry.h>
#include <wb_controller/utils.h>
#include <wb_controller/foot_state_machine.h>
#include <wb_controller/foot_trajectory.h>

namespace wb_controller
{

class Gait
{

public:

  enum gait_t {CRAWL=0,TROT,ONE_FOOT_LF,ONE_FOOT_RH,ONE_FOOT_RF,ONE_FOOT_LH};

  static inline std::string enum_to_string(const Gait::gait_t& gait_type)
  {
    switch(gait_type)
    {
       case Gait::gait_t::CRAWL:
          return "CRAWL";
       case Gait::gait_t::TROT:
          return "TROT";
    };
  }

  static inline Gait::gait_t string_to_enum(const std::string& gait_type)
  {
    if (gait_type == "CRAWL")
      return Gait::gait_t::CRAWL;
    else if (gait_type == "TROT")
      return Gait::gait_t::TROT;
  }

  Gait(const std::vector<std::string>& foot_names, const gait_t& gait_type)
  {
    assert(foot_names.size() == N_LEGS);

    auto ordered_foot_names = sortByLegPrefix(foot_names);

    if(gait_type == TROT)
    {
      schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::LF],0));
      schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::RH],0));
      schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::RF],1));
      schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::LH],1));
      next_feet_to_move_.resize(2);
      max_priority_ = 1;
    }
    else if(gait_type == CRAWL)
    {
      schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::LF],0));
      schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::RH],1));
      schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::RF],2));
      schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::LH],3));
      next_feet_to_move_.resize(1);
      max_priority_ = 3;
    }
    else if(gait_type == ONE_FOOT_LF)
    {
      schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::LF],0));
      next_feet_to_move_.resize(1);
      max_priority_ = 0;
    }
    else if(gait_type == ONE_FOOT_RH)
    {
      schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::RH],0));
      next_feet_to_move_.resize(1);
      max_priority_ = 0;
    }
    else if(gait_type == ONE_FOOT_RF)
    {
      schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::RF],0));
      next_feet_to_move_.resize(1);
      max_priority_ = 0;
    }
    else if(gait_type == ONE_FOOT_LH)
    {
      schedule_.push_back(foot_priority_t(ordered_foot_names[_leg_id::LH],0));
      next_feet_to_move_.resize(1);
      max_priority_ = 0;
    }
    else
    {
      throw std::runtime_error("Wrong gait type!");
    }

    current_priority_ = 0;
  }

  const std::vector<std::string>& getNextSchedule()
  {

    unsigned int idx = 0;
    for(unsigned int i=0;i < schedule_.size(); i++)
      if(schedule_[i].second == current_priority_)
        next_feet_to_move_[idx++] = schedule_[i].first;

    current_priority_++;
    current_priority_ %= max_priority_+1;

    return next_feet_to_move_;
  }

private:

  typedef std::pair<std::string,unsigned int> foot_priority_t;
  foot_priority_t foot_priority_;
  std::vector<foot_priority_t> schedule_;

  unsigned int current_priority_;
  unsigned int max_priority_;

  std::vector<std::string> next_feet_to_move_;


};

class GaitGenerator
{
public:

  /**
     * @brief Shared pointer to GaitGenerator
     */
  typedef std::shared_ptr<GaitGenerator> Ptr;

  /**
     * @brief Shared pointer to const GaitGenerator
     */
  typedef std::shared_ptr<const GaitGenerator> ConstPtr;

  GaitGenerator(const std::vector<std::string>& foot_names, const Gait::gait_t& gait_type, const std::string& trajectory_type)
  {
    assert(foot_names.size()==N_LEGS);// We assume we are working with a dog
    foot_names_ = foot_names;
    for(unsigned int i = 0; i<foot_names.size(); i++)
    {
      feet_[foot_names[i]].state_machine.reset(new FootStateMachine());
      feet_[foot_names[i]].trajectory.reset(selectTrajectoryType(trajectory_type));
      feet_[foot_names[i]].contact_state = false;
      feet_[foot_names[i]].trigger_stance = false;
      feet_[foot_names[i]].initial_pose = Eigen::Affine3d::Identity();
    }

    setSwingFrequency(0.0);

    gait_buffer_.resize(2);

    for(unsigned int i=0; i<gait_buffer_.size(); i++)
      gait_buffer_[i].reset(new Gait(foot_names,gait_type));

    current_gait_idx_ = 0;
    next_gait_idx_ = 1;
    scheduled_feet_ = gait_buffer_[current_gait_idx_]->getNextSchedule();

    change_gait_ = false;
    activate_swing_ = false;

    gait_type_ = gait_type;
  }

  const std::vector<std::string>& getFootNames()
  {
    return foot_names_;
  }

  void switchGait()
  {
    if(gait_type_ == Gait::gait_t::TROT)
      setGaitType(Gait::gait_t::CRAWL);
    else
      setGaitType(Gait::gait_t::TROT);
  }

  void setGaitType(const Gait::gait_t& gait_type)
  {
    gait_buffer_[next_gait_idx_].reset(new Gait(foot_names_,gait_type));
    change_gait_ = true;
    gait_type_ = gait_type;
  }

  void setGaitTypeName(const std::string& gait_type_name)
  {
    setGaitType(Gait::string_to_enum(gait_type_name));
  }

  const Eigen::Affine3d& getReference(const std::string& foot_name)
  {
    return feet_[foot_name].trajectory->getReference();
  }

  const Eigen::Vector6d& getReferenceDot(const std::string& foot_name)
  {
    return feet_[foot_name].trajectory->getReferenceDot();
  }

  bool isSwinging(const std::string& foot_name)
  {
    return feet_[foot_name].state_machine->isSwing();
  }

  bool isInStance(const std::string& foot_name)
  {
    return feet_[foot_name].state_machine->isStance();
  }

  bool isStateChanged(const std::string& foot_name)
  {
    return feet_[foot_name].state_machine->isStateChanged();
  }

  bool isTouchDown(const std::string& foot_name)
  {
    return feet_[foot_name].state_machine->isTouchDown();
  }

  bool isLiftOff(const std::string& foot_name)
  {
    return feet_[foot_name].state_machine->isLiftOff();
  }

  bool isCycleEnded(const std::string& foot_name)
  {
    return feet_[foot_name].state_machine->isCycleEnded();
  }

  bool isAnyFootInLiftOff()
  {
    bool result = false;
    for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
      result = result || it->second.state_machine->isLiftOff();
    return result;
  }

  bool isAnyFootInSwing()
  {
    bool result = false;
    for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
      result = result || it->second.state_machine->isSwing();
    return result;
  }

  bool isAnyFootInTouchDown()
  {
    bool result = false;
    for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
      result = result || it->second.state_machine->isTouchDown();
    return result;
  }

  bool isAnyFootInStance()
  {
    bool result = false;
    for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
      result = result || it->second.state_machine->isStance();
    return result;
  }

  void setContactState(const std::string& foot_name, const bool& contact)
  {
    feet_[foot_name].contact_state = contact;
  }

  const bool& getContactState(const std::string& foot_name)
  {
    return feet_[foot_name].contact_state;
  }

  void setInitialPose(const std::string& foot_name, const Eigen::Affine3d& initial_pose)
  {
    feet_[foot_name].trajectory->setInitialPose(initial_pose);
  }

  double getDutyFactor(const std::string& foot_name)
  {
    return feet_[foot_name].state_machine->getDutyFactor();
  }

  void setDutyFactor(const double& duty_factor)
  {
    for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
      it->second.state_machine->setDutyFactor(duty_factor);
  }

  void setDutyFactor(const std::string& foot_name, const double& duty_factor)
  {
    feet_[foot_name].state_machine->setDutyFactor(duty_factor);
  }

  void setSwingFrequency(const double& swing_frequency)
  {
    for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
    {
      it->second.trajectory->setSwingFrequency(swing_frequency);
      it->second.state_machine->setSwingFrequency(swing_frequency);
    }
  }

  void setSwingFrequency(const std::string& foot_name, const double& swing_frequency)
  {
    feet_[foot_name].trajectory->setSwingFrequency(swing_frequency);
    feet_[foot_name].state_machine->setSwingFrequency(swing_frequency);
  }

  double getSwingFrequency(const std::string& foot_name)
  {
    return feet_[foot_name].trajectory->getSwingFrequency();
  }

  double getAvgSwingFrequency()
  {
    double avg = 0.0;
    for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
      avg = avg + it->second.trajectory->getSwingFrequency();
    avg = avg / feet_.size();
    return avg;
  }

  double getAvgDutyFactor()
  {
    double avg = 0.0;
    for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
      avg = avg + it->second.state_machine->getDutyFactor();
    avg = avg / feet_.size();
    return avg;
  }

  const Gait::gait_t& getGaitType()
  {
    return gait_type_;
  }

  std::string getGaitTypeName()
  {
    return Gait::enum_to_string(gait_type_);
  }

  void setStepLength(const double& length)
  {
    for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
      it->second.trajectory->setStepLength(length);
  }

  void setStepHeading(const double& heading)
  {
    for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
      it->second.trajectory->setStepHeading(heading);
  }

  void setStepHeight(const double& height)
  {
    for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
      it->second.trajectory->setStepHeight(height);
  }

  void setStepHeadingRate(const double& heading_rate)
  {
    for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
      it->second.trajectory->setStepHeadingRate(heading_rate);
  }

  void setStepLength(const std::string& foot_name, const double& length)
  {
    feet_[foot_name].trajectory->setStepLength(length);
  }

  void setStepHeading(const std::string& foot_name, const double& heading)
  {
    feet_[foot_name].trajectory->setStepHeading(heading);
  }

  void setStepHeight(const std::string& foot_name, const double& height)
  {
    feet_[foot_name].trajectory->setStepHeight(height);
  }

  void setStepHeadingRate(const std::string& foot_name, const double& heading_rate)
  {
    feet_[foot_name].trajectory->setStepHeadingRate(heading_rate);
  }

  void activateSwing()
  {
    activate_swing_ = true;
  }

  void deactivateSwing()
  {
    activate_swing_ = false;
  }

  bool isTrajectoryFinished(const std::string& foot_name)
  {
    return feet_[foot_name].trajectory->isFinished();
  }

  void update(const double& period)
  {
    // 1) Check if the scheduled feet are all ready to get triggered and start the swing if this is the case.
    bool scheduled_feet_are_ready = true;
    for(unsigned int i=0; i<scheduled_feet_.size(); i++)
      if(!feet_[scheduled_feet_[i]].state_machine->isCycleEnded())
      {
        scheduled_feet_are_ready = false;
        break;
      }
    if(scheduled_feet_are_ready && activate_swing_)
      for(unsigned int i=0; i<scheduled_feet_.size(); i++)
        feet_[scheduled_feet_[i]].state_machine->triggerSwing();

    // 2) Update the trajectories for each foot depending on the state machine status
    for(feet_t::iterator it = feet_.begin(); it != feet_.end(); it++)
    {
#ifdef REACHING_MOTION
      it->second.trigger_stance = it->second.contact_state;
#else
      it->second.trigger_stance = it->second.contact_state || it->second.trajectory->isFinished(); //CloseLoop with trajectory end
#endif
      it->second.state_machine->update(period,it->second.trigger_stance);

      if (it->second.state_machine->isSwing())
      {
        if (it->second.state_machine->isLiftOff())
        {
          it->second.trajectory->start();
        }
        it->second.trajectory->update(period);

        ROS_DEBUG_STREAM("Update trajectory for foot "<< it->first);
      }
      else
      {
        if (it->second.state_machine->isTouchDown())
        {
          it->second.trajectory->stop();
        }
        it->second.trajectory->standBy();

        ROS_DEBUG_STREAM("StandBy trajectory for foot "<< it->first);
      }
    }

    // 3) If the cycle for the scheduled feet is over, change the schedule to the next one (i.e. move to the next feet)
    unsigned int cnt = 0;
    for(unsigned int i=0; i<scheduled_feet_.size(); i++)
      if(feet_[scheduled_feet_[i]].state_machine->isCycleEnded())
        cnt++;
    if(cnt == scheduled_feet_.size())
    {
      if(change_gait_)
        changeGait();
      scheduled_feet_ = gait_buffer_[current_gait_idx_]->getNextSchedule();
    }
  }

private:

  void changeGait()
  {
    current_gait_idx_ = current_gait_idx_ + 1;
    current_gait_idx_ = current_gait_idx_ % 2;
    next_gait_idx_    = next_gait_idx_    + 1;
    next_gait_idx_    = next_gait_idx_    % 2;

    change_gait_ = false;
  }

  struct feet_status_t
  {
    std::shared_ptr<FootStateMachine> state_machine;
    std::shared_ptr<TrajectoryInterface> trajectory;
    bool contact_state;
    bool trigger_stance;
    Eigen::Affine3d initial_pose;
  };

  TrajectoryInterface* selectTrajectoryType(const std::string& trajectory_type)
  {
    ROS_INFO_STREAM("Selected " << trajectory_type << " trajectory");
    if(std::strcmp(trajectory_type.c_str(),"ellipse")==0)
    {
      return new Ellipse();
    }
    else
    {
      throw std::runtime_error("Wrong trajectory type!");
    }
    return NULL;
  }

  typedef std::map<std::string,feet_status_t> feet_t;
  typedef std::shared_ptr<Gait> gait_ptr_t;

  std::vector<std::string> scheduled_feet_;
  std::vector<std::string> foot_names_;

  feet_t feet_;
  std::vector<gait_ptr_t > gait_buffer_;
  std::atomic<unsigned int> current_gait_idx_;
  std::atomic<unsigned int> next_gait_idx_;
  std::atomic<bool> change_gait_;
  std::atomic<bool> activate_swing_;

  Gait::gait_t gait_type_;

};

} // namespace


#endif
