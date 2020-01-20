#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <atomic>
#include <XBotInterface/ModelInterface.h>
#include <wb_controller/geometry.h>
#include <wb_controller/utils.h>
#include <wb_controller/foot_state_machine.h>
#include <wb_controller/foot_trajectory.h>

namespace wb_controller
{

class Gait
{

public:
  Gait(const std::vector<std::string>& feet_names, const std::string& gait_type)
  {
    ROS_INFO_STREAM("Selected " << gait_type << " gait");

    assert(feet_names.size() == N_LEGS);

    auto ordered_feet_names = sortByLegName(feet_names);

    if(std::strcmp(gait_type.c_str(),"trot")==0)
    {
      schedule_.push_back(foot_priority_t(ordered_feet_names[_leg_id::LF],0));
      schedule_.push_back(foot_priority_t(ordered_feet_names[_leg_id::RH],0));
      schedule_.push_back(foot_priority_t(ordered_feet_names[_leg_id::RF],1));
      schedule_.push_back(foot_priority_t(ordered_feet_names[_leg_id::LH],1));
      next_feet_to_move_.resize(2);
      max_priority_ = 1;
    }
    else if(std::strcmp(gait_type.c_str(),"crawl")==0)
    {
      schedule_.push_back(foot_priority_t(ordered_feet_names[_leg_id::LF],0));
      schedule_.push_back(foot_priority_t(ordered_feet_names[_leg_id::RH],1));
      schedule_.push_back(foot_priority_t(ordered_feet_names[_leg_id::RF],2));
      schedule_.push_back(foot_priority_t(ordered_feet_names[_leg_id::LH],3));
      next_feet_to_move_.resize(1);
      max_priority_ = 3;
    }
    else if(std::strcmp(gait_type.c_str(),"one_foot_lf")==0)
    {
      schedule_.push_back(foot_priority_t(ordered_feet_names[_leg_id::LF],0));
      next_feet_to_move_.resize(1);
      max_priority_ = 0;
    }
    else if(std::strcmp(gait_type.c_str(),"one_foot_rh")==0)
    {
      schedule_.push_back(foot_priority_t(ordered_feet_names[_leg_id::RH],0));
      next_feet_to_move_.resize(1);
      max_priority_ = 0;
    }
    else if(std::strcmp(gait_type.c_str(),"one_foot_rf")==0)
    {
      schedule_.push_back(foot_priority_t(ordered_feet_names[_leg_id::RF],0));
      next_feet_to_move_.resize(1);
      max_priority_ = 0;
    }
    else if(std::strcmp(gait_type.c_str(),"one_foot_lh")==0)
    {
      schedule_.push_back(foot_priority_t(ordered_feet_names[_leg_id::LH],0));
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

  GaitGenerator(const std::vector<std::string>& feet_names, const std::vector<std::string>& hips_names, const std::string& gait_type, const std::string& trajectory_type)
  {
    assert(feet_names.size()==N_LEGS);// We assume we are working with a dog
    assert(hips_names.size()==N_LEGS);
    feet_names_ = feet_names;
    hips_names_ = hips_names;
    for(unsigned int i = 0; i<feet_names.size(); i++)
    {
      feet_[feet_names[i]].state_machine.reset(new FootStateMachine());
      feet_[feet_names[i]].trajectory.reset(selectTrajectoryType(trajectory_type));
      feet_[feet_names[i]].contact_state = false;
      feet_[feet_names[i]].trigger_stance = false;
      feet_[feet_names[i]].initial_pose = Eigen::Affine3d::Identity();
    }

    setSwingFrequency(0.0);

    gait_buffer_.resize(2);

    for(unsigned int i=0; i<gait_buffer_.size(); i++)
      gait_buffer_[i].reset(new Gait(feet_names,gait_type));

    current_gait_idx_ = 0;
    next_gait_idx_ = 1;
    scheduled_feet_ = gait_buffer_[current_gait_idx_]->getNextSchedule();

    change_gait_ = false;
    activate_swing_ = false;

    gait_type_ = gait_type;
  }

  void switchGait()
  {
    if(gait_type_ == "trot")
      setGaitType("crawl");
    else
      setGaitType("trot");
  }

  void setGaitType(const std::string& gait_type)
  {
    gait_buffer_[next_gait_idx_].reset(new Gait(feet_names_,gait_type));
    change_gait_ = true;
    gait_type_ = gait_type;
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

  bool isAnyFootInTouchDown()
  {
    bool result = false;
    for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
      result = result || it->second.state_machine->isTouchDown();
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

  const std::vector<std::string>& getFeetNames()
  {
    return feet_names_;
  }

  const std::vector<std::string>& getHipsNames()
  {
    return hips_names_;
  }

  const std::string& getGaitType()
  {
    return gait_type_;
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

      it->second.trigger_stance = it->second.contact_state || it->second.trajectory->isFinished(); //CloseLoop with trajectory end

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

  feet_t feet_;
  std::vector<gait_ptr_t > gait_buffer_;
  std::atomic<unsigned int> current_gait_idx_;
  std::atomic<unsigned int> next_gait_idx_;
  std::atomic<bool> change_gait_;
  std::atomic<bool> activate_swing_;

  std::vector<std::string> feet_names_;
  std::vector<std::string> hips_names_;

  std::string gait_type_;

};

} // namespace


#endif
