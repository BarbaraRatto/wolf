#include <wb_controller/terrain_estimator.h>
#include <wb_controller/utils.h>

using namespace rt_logger;

namespace wb_controller {

TerrainEstimator::TerrainEstimator(GaitGenerator::Ptr gait_generator, StateEstimator::Ptr state_estimator)
{

  assert(gait_generator);
  gait_generator_ = gait_generator;

  assert(state_estimator);
  state_estimator_ = state_estimator;

  update_ = false;

  A_.resize(N_LEGS,3);
  Ai_.resize(3,N_LEGS);
  b_.resize(N_LEGS);

  roll_ = roll_filt_ = estimated_roll_ = 0.0;
  pitch_ = pitch_filt_ = estimated_pitch_ = 0.0;


  terrain_normal_ << 0.0, 0.0, 1.0;
  pos_.setZero();
  R_.setIdentity();
  T_.setIdentity();

  base_adjustment_ = base_adjustment_prev_ = base_adjustment_dot_ = 0.0;

  RtLogger::getLogger().addPublisher(CLASS_NAME+"/terrain_normal",terrain_normal_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/roll",roll_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/pitch",pitch_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/base_adjustment",base_adjustment_);
  RtLogger::getLogger().addPublisher(CLASS_NAME+"/base_adjustment_dot",base_adjustment_dot_);


}


void TerrainEstimator::setEstimationType() // TODO
{


}

bool TerrainEstimator::computeTerrainEstimation(const double& dt)
{

  // 0 - Update the terrain estimation everytime there is a touchdown
  if(gait_generator_->isAnyFootInTouchDown())
    update_ = true;

  // 1 - Check if the feet are all in stance
  if(gait_generator_->isAllFeetInStance())
  {

    if(update_ == true)
    {

      // 2 - Update A and b with the feet position
      update();

      tmp_matrix3d_.noalias() = A_.transpose()*A_;

      if(tmp_matrix3d_.determinant()!=0.0)
      {
        Ai_.noalias() = tmp_matrix3d_.inverse() * A_.transpose();
        terrain_normal_ = Ai_ * b_;
      }
      else
      {
        ROS_WARN_STREAM_NAMED(CLASS_NAME,"Can not solve the problem!");
        return false;
      }

      // 3 - Normalize the terrain normal
      terrain_normal_(2) = 1.0;
      terrain_normal_ = terrain_normal_ / terrain_normal_.norm();

      // 4 - Extract the estimated values for roll and pitch
      estimated_pitch_  = std::atan(terrain_normal_(0)/terrain_normal_(2));
      estimated_roll_ = std::atan(-terrain_normal_(1)*std::sin(estimated_pitch_)/terrain_normal_(0));

      // Perform only one update per touch down
      update_ = false;
    }
  }

  // 5 - Filter
  roll_  = secondOrderFilter(roll_,roll_filt_,estimated_roll_,1.0);
  pitch_ = secondOrderFilter(pitch_,pitch_filt_,estimated_pitch_,1.0);

  // 6 - Check output limits
  if((roll_  > min_roll_ ) && (roll_  < max_roll_) &&
     (pitch_ > min_pitch_) && (pitch_ < max_pitch_))
  {
      // 7 - Update the roll and pitch output values
      roll_rate_ = (roll_out_ - roll_)/dt;
      pitch_rate_ = (pitch_out_ - pitch_)/dt;
      roll_out_  = roll_;
      pitch_out_ = pitch_;
  }
  else
  {
      ROS_WARN_STREAM_NAMED(CLASS_NAME,"Angles beyond limits!");
      return false;
  }

  // Update the resulting Transformation
  rpyToRot(roll_out_,pitch_out_,0.0,R_);
  T_.translation() = pos_;
  T_.linear() = R_; // world_T_terrain

  // 8- Update the swing trajectories
  gait_generator_->setStepRoll(roll_out_);
  gait_generator_->setStepPitch(pitch_out_);
  gait_generator_->setStepRollRate(roll_rate_);
  gait_generator_->setStepPitchRate(pitch_rate_);

  // 9 - Update the state estimator (to align the contact forces with the
  // terrain)
  state_estimator_->setTerrainNormal(terrain_normal_);

  // 10 - Base adjustment, compute the offsets to help adapting the posture based
  // on the terrain, for now, we compute only an adjustment along the x axis.
  double terrain_h_base = state_estimator_->getFloatingBasePosition()(2);
  base_adjustment_ = terrain_h_base * std::tan(pitch_out_);
  base_adjustment_dot_ = (base_adjustment_ - base_adjustment_prev_)/dt;
  base_adjustment_prev_ = base_adjustment_;


  return true;
}

double TerrainEstimator::getBaseAdjustment() const
{
  return base_adjustment_;
}

double TerrainEstimator::getBaseAdjustmentDot() const
{
  return base_adjustment_dot_;
}

void TerrainEstimator::update()
{

  auto foot_names = gait_generator_->getFootNames();
  auto foot_positions = state_estimator_->getFeetPositionInWorld();

  A_(0,0) = foot_positions[foot_names[0]](0);
  A_(0,1) = foot_positions[foot_names[0]](1);
  A_(0,2) = 1.0;
  b_(0)   = -foot_positions[foot_names[0]](2);

  A_(1,0) = foot_positions[foot_names[1]](0);
  A_(1,1) = foot_positions[foot_names[1]](1);
  A_(1,2) = 1.0;
  b_(1)   = -foot_positions[foot_names[1]](2);

  A_(2,0) = foot_positions[foot_names[2]](0);
  A_(2,1) = foot_positions[foot_names[2]](1);
  A_(2,2) = 1.0;
  b_(2)   = -foot_positions[foot_names[2]](2);

  A_(3,0) = foot_positions[foot_names[3]](0);
  A_(3,1) = foot_positions[foot_names[3]](1);
  A_(3,2) = 1.0;
  b_(3)   = -foot_positions[foot_names[3]](2);

  double avg_x = 0.0;
  double avg_y = 0.0;
  double avg_z = 0.0;
  for(unsigned int i = 0; i<foot_names.size(); i++)
  {
    avg_x = avg_x + foot_positions[foot_names[i]](0);
    avg_y = avg_y + foot_positions[foot_names[i]](1);
    avg_z = avg_z + foot_positions[foot_names[i]](2);
  }

  avg_x = avg_x/N_LEGS;
  avg_y = avg_y/N_LEGS;
  avg_z = avg_z/N_LEGS;

  pos_ << avg_x, avg_y, avg_z;

}

void TerrainEstimator::setMinRoll(const double min)
{
    min_roll_ = min;
}

void TerrainEstimator::setMinPitch(const double min)
{
    min_pitch_ = min;
}

void TerrainEstimator::setMaxRoll(const double max)
{
    max_roll_ = max;
}

void TerrainEstimator::setMaxPitch(const double max)
{
    max_pitch_ = max;
}

double TerrainEstimator::getRoll() const
{
    return roll_out_;
}

double TerrainEstimator::getPitch() const
{
    return pitch_out_;
}

double TerrainEstimator::getRollRate() const
{
    return roll_rate_;
}

double TerrainEstimator::getPitchRate() const
{
    return pitch_rate_;
}

const Eigen::Matrix3d& TerrainEstimator::getOrientation() const
{
    return R_;
}

const Eigen::Affine3d& TerrainEstimator::getPose() const
{
    return T_;
}

const Eigen::Vector3d& TerrainEstimator::getPosition() const
{
    return pos_;
}

const Eigen::Vector3d& TerrainEstimator::getTerrainNormal() const
{
    return terrain_normal_;
}

} // namespace

