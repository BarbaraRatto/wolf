#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <Eigen/Core>
#include <XBotInterface/TypedefAndEnums.h>

namespace wolf_controller
{

/**
 * @brief converts a quaternion \f$\mathbf{q}\f$ into a rotation matrix
 * \f$R_q(\mathbf{q})\f$. The rotation matrix maps a vector
 * \f$\mathbf{z}\in\mathbb{R}^{3\times1}\f$ (expressed in global coordinates)
 * into a vector \f$ \mathbf{z}' \in \mathbb{R}^{3\times1}\f$ (expressed in
 * local coordinates), such that \f$ \mathbf{z}' = R_q(\mathbf{q})\mathbf{z}\f$.
 *
 * @param[in] q structure containing the quaternion (it has to be normalized)
 * @return the 3 by 3 rotation matrix \f$R_q(\mathbf{q})\f$
 * @remark the function uses the formula (125) from <a href="https://www.astro.rug.nl/software/kapteyn/_downloads/attitude.pdf">"Representing Attitude: Euler
 *  Angles, Unit Quaternions, and Rotation Vectors"</a> by James Diebel.
 * @date July 2005
 */
inline void quatToRotMat(const Eigen::Quaterniond& q, Eigen::Matrix3d& R)
{
    R.setZero();
    R(0, 0) = -1.0 + 2.0 * (q.w() * q.w()) + 2.0 * (q.x() * q.x());
    R(1, 1) = -1.0 + 2.0 * (q.w() * q.w()) + 2.0 * (q.y() * q.y());
    R(2, 2) = -1.0 + 2.0 * (q.w() * q.w()) + 2.0 * (q.z() * q.z());
    R(0, 1) = 2.0 * (q.x() * q.y() + q.w() * q.z());
    R(0, 2) = 2.0 * (q.x() * q.z() - q.w() * q.y());
    R(1, 0) = 2.0 * (q.x() * q.y() - q.w() * q.z());
    R(1, 2) = 2.0 * (q.y() * q.z() + q.w() * q.x());
    R(2, 0) = 2.0 * (q.x() * q.z() + q.w() * q.y());
    R(2, 1) = 2.0 * (q.y() * q.z() - q.w() * q.x());
}


inline void rotTorpy(const Eigen::Matrix3d& R, Eigen::Vector3d& rpy)
{
    rpy(0) = std::atan2(R(1,2),R(2,2));
    rpy(1) = -std::asin(R(0,2));
    rpy(2) = std::atan2(R(0,1),R(0,0));
}

inline void rpyToRot(const Eigen::Vector3d& rpy, Eigen::Matrix3d& R)
{
  R.setZero();

  double c_y = std::cos(rpy(2));
  double s_y = std::sin(rpy(2));

  double c_r = std::cos(rpy(0));
  double s_r = std::sin(rpy(0));

  double c_p = std::cos(rpy(1));
  double s_p = std::sin(rpy(1));

  R << c_p*c_y ,  s_r*s_p*c_y - c_r*s_y                 ,  c_r*s_p*c_y + s_r*s_y  ,
       c_p*s_y ,  s_r*s_p*s_y + c_r*c_y                 ,  s_y*s_p*c_r - c_y*s_r,
       -s_p    ,  c_p*s_r                               ,  c_r*c_p;
}

inline void rpyToRotTranspose(const Eigen::Vector3d& rpy, Eigen::Matrix3d& R)
{
  R.setZero();

  double c_y = std::cos(rpy(2));
  double s_y = std::sin(rpy(2));

  double c_r = std::cos(rpy(0));
  double s_r = std::sin(rpy(0));

  double c_p = std::cos(rpy(1));
  double s_p = std::sin(rpy(1));

  R << c_p*c_y               ,  c_p*s_y                ,  -s_p,
       s_r*s_p*c_y - c_r*s_y ,  s_r*s_p*s_y + c_r*c_y  ,  s_r*c_p,
       c_r*s_p*c_y + s_r*s_y ,  c_r*s_p*s_y - s_r*c_y  ,  c_r*c_p;
}

inline void yawToRot(const double& yaw, Eigen::Matrix3d& R)
{
    R.setZero();
    double c_y = std::cos(yaw);
    double s_y = std::sin(yaw);

    R << c_y  ,  s_y ,		0,
         -s_y ,  c_y ,  		0,
         0    ,     0     ,       1;
}

inline void rollToRot(const double& roll, Eigen::Matrix3d& R)
{
    R.setZero();
    double c_r = std::cos(roll);
    double s_r = std::sin(roll);

    R <<    1   ,    0     	  ,  	  0,
            0   ,    c_r ,  s_r,
            0   ,    -s_r,  c_r;

}

inline void pitchToRot(const double& pitch, Eigen::Matrix3d& R)
{
    R.setZero();
    double c_p = std::cos(pitch);
    double s_p = std::sin(pitch);


    R << c_p 	,	 0  ,   -s_p,
            0       ,    1  ,   0,
           s_p 	,	0   ,  c_p;

}

/**
 * @brief Function to compute the linear tranformation matrix between euler
 * rates (in ZYX convention) and omega vector, where omega is expressed in world
 * coordinates to get the component expressed in the world ortogonal frame.
 * @param rpy
 * @return Ear
*/
inline void rpyToEarWorld(const Eigen::Vector3d& rpy, Eigen::Matrix3d& Ear){

    const double& pitch = rpy(1);
    const double& yaw = rpy(2);

    double c_y = std::cos(yaw);
    double s_y = std::sin(yaw);

    double c_p = std::cos(pitch);
    double s_p = std::sin(pitch);

    Ear <<  c_p*c_y, -s_y,    0,
            c_p*s_y,  c_y,    0,
            -s_p,     0,      1;
}

/**
 * @brief rpyToEar Function to compute the linear tranformation matrix between
 * euler rates (in ZYX convention) and omega vector where omega is expressed
 * in base coordinates (is R*EarWorld)
 * @param rpy
 * @return Ear
 */
inline void rpyToEarBase(const Eigen::Vector3d & rpy, Eigen::Matrix3d& Ear){

    const double& roll = rpy(0);
    const double& pitch = rpy(1);

    double c_r = std::cos(roll);
    double s_r = std::sin(roll);

    double c_p = std::cos(pitch);
    double s_p = std::sin(pitch);

    Ear<< 1,   0,    -s_p,
            0,   c_r,  c_p*s_r,
            0,  -s_r,  c_p*c_r;
    /*Ear<< 1,   0,    s_p,
          0,   c_r,  -c_p*s_r,
          0,   s_r,  c_p*c_r;*/
}

/**
 * @brief rpyToInvEar Function to compute the linear tranformation matrix between
 * omega vector and euler rates (this computes the inverse matrix of rpyToEarBase
 * @param rpy
 * @return EarInv
 */
inline void rpyToEarBaseInv(const Eigen::Vector3d & rpy, Eigen::Matrix3d& EarInv){

    const double& roll = rpy(0);
    const double& pitch = rpy(1);

    double c_r = std::cos(roll);
    double s_r = std::sin(roll);

    double c_p = std::cos(pitch);
    double s_p = std::sin(pitch);

    EarInv <<1, (s_p*s_r)/c_p,   (c_r*s_p)/c_p,
             0,          c_r,         -s_r,
             0,          s_r/c_p,   c_r/c_p;

}

/**
 * @brief computeCartesianInertiaInverse
 * @param J Jacobian
 * @param Mi joint inertia inverse
 * @param Lambdai Cartesian inertia matrix inverse
 */
inline void computeCartesianInertiaInverse(const Eigen::MatrixXd& J, const Eigen::MatrixXd& Mi, Eigen::Matrix6d& Lambdai)
{
    Lambdai = J*Mi*J.transpose();
}

}; // namespace

#endif
