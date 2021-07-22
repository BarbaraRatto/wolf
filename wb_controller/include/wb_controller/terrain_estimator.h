#ifndef TERRAIN_ESTIMATOR_H
#define TERRAIN_ESTIMATOR_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <atomic>
#include <wb_controller/state_estimator.h>
#include <wb_controller/gait_generator.h>

namespace wb_controller
{

class TerrainEstimator {

public:

    const std::string CLASS_NAME = "TerrainEstimator";

    /**
     * @brief Shared pointer to TerrainEstimator
     */
    typedef std::shared_ptr<TerrainEstimator> Ptr;

    /**
     * @brief Shared pointer to const TerrainEstimator
     */
    typedef std::shared_ptr<const TerrainEstimator> ConstPtr;

    enum estimation_t {NONE=0,FLAT_TERRAIN,ROUGH_TERRAIN};

    TerrainEstimator(GaitGenerator::Ptr gait_generator, StateEstimator::Ptr state_estimator);

    void setEstimationType();

    bool computeTerrainEstimation(const double& dt);

    void setMinRoll(const double min);
    void setMinPitch(const double min);

    void setMaxRoll(const double max);
    void setMaxPitch(const double max);

    double getRoll() const;
    double getPitch() const;

    double getRollRate() const;
    double getPitchRate() const;

    double getBaseAdjustment() const;

    double getBaseAdjustmentDot() const;

    const Eigen::Vector3d& getTerrainNormal() const;

    const Eigen::Vector3d& getPosition() const;

    const Eigen::Matrix3d& getOrientation() const;

    const Eigen::Affine3d& getPose() const;

    const std::vector<Eigen::Vector3d>& getFeetPosition() const;

private:

    void update();

    Eigen::MatrixXd A_;
    Eigen::MatrixXd Ai_;
    Eigen::VectorXd b_;
    Eigen::Vector3d terrain_normal_;
    Eigen::Vector3d pos_;
    Eigen::Matrix3d R_;
    Eigen::Affine3d T_;

    GaitGenerator::Ptr gait_generator_;
    StateEstimator::Ptr state_estimator_;

    double roll_;
    double pitch_;

    double estimated_roll_;
    double estimated_pitch_;

    double roll_filt_;
    double pitch_filt_;

    std::atomic<double> max_roll_;
    std::atomic<double> max_pitch_;

    std::atomic<double> min_roll_;
    std::atomic<double> min_pitch_;

    std::atomic<double> roll_out_;
    std::atomic<double> pitch_out_;

    std::atomic<double> roll_rate_;
    std::atomic<double> pitch_rate_;

    double base_adjustment_; // FIXME: atomic
    double base_adjustment_filt_;
    double base_adjustment_dot_;
    double base_adjustment_dot_filt_;
    double base_adjustment_prev_;

    /** @brief Trigger the update of the terrain estimator */
    bool update_;

    Eigen::Matrix3d tmp_matrix3d_;


};


} // namespace


#endif

