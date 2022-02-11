#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <Eigen/Core>
#include <XBotInterface/TypedefAndEnums.h>

namespace wolf_controller
{

/**
 * @brief constrainAngle Normalize angle to [-M_PI,M_PI):
 * @param x
 * @return
 */
inline double constrainAngle(double x){
  x = fmod(x + M_PI,2*M_PI);
  if (x < 0)
    x += 2*M_PI;
  return x - M_PI;
}

/**
 * @brief angleConv Convert angle to [-2*M_PI,2*M_PI]
 * @param angle
 * @return
 */
inline double angleConv(double angle){
  return fmod(constrainAngle(angle),2*M_PI);
}

/**
 * @brief angleDiff Compute the wrapped difference between two angles
 * @param a
 * @param b
 * @return
 */
inline double angleDiff(double a,double b){
  double dif = fmod(b - a + M_PI,2*M_PI);
  if (dif < 0)
    dif += 2*M_PI;
  return dif - M_PI;
}

/**
 * @brief unwrap Unwrap angle to avoid jumps of pi
 * @param previousAngle
 * @param newAngle
 * @return
 */
inline double unwrap(double previousAngle,double newAngle){
  return previousAngle - angleDiff(newAngle,angleConv(previousAngle));
}

inline Eigen::Vector3d unwrap(Eigen::Vector3d& previousAngles, Eigen::Vector3d& newAngles)
{
  Eigen::Vector3d outAngles;
  for (unsigned int i=0; i<3; i++)
    outAngles(i) = previousAngles(i) - angleDiff(newAngles(i),angleConv(previousAngles(i)));
  return outAngles;
}

/* wrap x -> [0,max) */
inline double wrapMax(double x, double max)
{
  /* integer math: `(max + x % max) % max` */
  return fmod(max + fmod(x, max), max);
}
/* wrap x -> [min,max) */
inline double wrapMinMax(double x, double min, double max)
{
  return min + wrapMax(x - min, max - min);
}

inline void quatToRot(const Eigen::Quaterniond& q, Eigen::Matrix3d& R)
{
  double tmp1, tmp2;
  double squ, sqx, sqy, sqz;
  squ = q.w()*q.w();
  sqx = q.x()*q.x();
  sqy = q.y()*q.y();
  sqz = q.z()*q.z();
  R(0,0) =  sqx - sqy - sqz + squ;
  R(1,1) = -sqx + sqy - sqz + squ;
  R(2,2) = -sqx - sqy + sqz + squ;
  tmp1 = q.x()*q.y();
  tmp2 = q.z()*q.w();
  R(1,0) = 2.0 * (tmp1 + tmp2);
  R(0,1) = 2.0 * (tmp1 - tmp2);
  tmp1 = q.x()*q.z();
  tmp2 = q.y()*q.w();
  R(2,0) = 2.0 * (tmp1 - tmp2);
  R(0,2) = 2.0 * (tmp1 + tmp2);
  tmp1 = q.y()*q.z();
  tmp2 = q.x()*q.w();
  R(2,1) = 2.0 * (tmp1 + tmp2);
  R(1,2) = 2.0 * (tmp1 - tmp2);
}

inline void rotToQuat(const Eigen::Matrix3d R, Eigen::Quaterniond& q)
{
  double t, s;
  t = 1 + R(0,0) + R(1,1) + R(2,2);
  if (t > 1e-8)
  {
    s = 0.5 / std::sqrt(t);
    q.w() = 0.25 / s;
    q.x() = (R(2,1) - R(1,2)) * s;
    q.y() = (R(0,2) - R(2,0)) * s;
    q.z() = (R(1,0) - R(0,1)) * s;
  }
  else if (R(0,0) > R(1,1) && R(0,0) > R(2,2))
  {
    s = std::sqrt(1 + R(0,0) - R(1,1) - R(2,2)) * 2;
    q.x() = 0.25 * s;
    q.y() = (R(0,1) + R(1,0)) / s;
    q.z() = (R(0,2) + R(2,0)) / s;
    q.w() = (R(2,1) - R(1,2)) / s;
  }
  else if (R(1,1) > R(2,2))
  {
    s = std::sqrt(1 + R(1,1) - R(0,0) - R(2,2)) * 2;
    q.x() = (R(0,1) + R(1,0)) / s;
    q.y() = 0.25 * s;
    q.z() = (R(1,2) + R(2,1)) / s;
    q.w() = (R(0,2) - R(2,0)) / s;
  }
  else
  {
    s = std::sqrt(1 + R(2,2) - R(0,0) - R(1,1)) * 2;
    q.x() = (R(0,2) + R(2,0)) / s;
    q.y() = (R(1,2) + R(2,1)) / s;
    q.z() = 0.25 * s;
    q.w() = (R(1,0) - R(0,1)) / s;
  }
  q.normalize();
}

inline void quatToRpy(const Eigen::Quaterniond& q, Eigen::Vector3d& rpy)
{
  if(-2 * (q.x()*q.z() - q.w()*q.y()) > 0.9999)
  {
    // alternate solution
    rpy(0) = 0;                 // 2*atan2(q.x(), q.w());
    rpy(1) = M_PI / 2;
    rpy(2) = 2*std::atan2(q.z(), q.w());  // 0;
  }
  else if(-2 * (q.x()*q.z() - q.w()*q.y()) < -0.9999)
  {
    // alternate solution
    rpy(0) =  0;                 // 2*atan2(q.x(), q.w());
    rpy(1) = -M_PI / 2;
    rpy(2) =  2*std::atan2(q.z(), q.w());  // 0;
  }
  else
  {
    rpy(0) = std::atan2(2 * (q.y()*q.z() + q.w()*q.x()), (q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z()));
    rpy(1) = std::asin(-2 * (q.x()*q.z() - q.w()*q.y()));
    rpy(2) = std::atan2(2 * (q.x()*q.y() + q.w()*q.z()), (q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z()));
  }
}

inline void rpyToQuat(const double& roll, const double& pitch, const double& yaw, Eigen::Quaterniond& q)
{
  double phi, the, psi;
  phi = roll / 2;
  the = pitch / 2;
  psi = yaw / 2;
  q.w() = std::cos(phi) * std::cos(the) * std::cos(psi) + std::sin(phi) * std::sin(the) * std::sin(psi);
  q.x() = std::sin(phi) * std::cos(the) * std::cos(psi) - std::cos(phi) * std::sin(the) * std::sin(psi);
  q.y() = std::cos(phi) * std::sin(the) * std::cos(psi) + std::sin(phi) * std::cos(the) * std::sin(psi);
  q.z() = std::cos(phi) * std::cos(the) * std::sin(psi) - std::sin(phi) * std::sin(the) * std::cos(psi);
  q.normalize();
}

inline void rpyToQuat(const Eigen::Vector3d& rpy, Eigen::Quaterniond& q)
{
  rpyToQuat(rpy(0),rpy(1),rpy(2),q);
}


inline void rotToRpy(const Eigen::Matrix3d& R, Eigen::Vector3d& rpy)
{
  rpy(0) = std::atan2(R(2,1),R(2,2));
  rpy(1) = std::atan2(-R(2,0),std::sqrt(R(2,1)*R(2,1)+R(2,2)*R(2,2)));
  rpy(2) = std::atan2(R(1,0),R(0,0));
}

inline void rotTransposeToRpy(const Eigen::Matrix3d& R, Eigen::Vector3d& rpy)
{
  rpy(0) = std::atan2(R(1,2),R(2,2));
  rpy(1) = -std::asin(R(0,2));
  rpy(2) = std::atan2(R(0,1),R(0,0));
}

inline void rpyToRot(const double& roll, const double& pitch, const double& yaw, Eigen::Matrix3d& R)
{
  R.setZero();

  double c_y = std::cos(yaw);
  double s_y = std::sin(yaw);

  double c_r = std::cos(roll);
  double s_r = std::sin(roll);

  double c_p = std::cos(pitch);
  double s_p = std::sin(pitch);

  R << c_p*c_y ,  s_r*s_p*c_y - c_r*s_y                 ,  c_r*s_p*c_y + s_r*s_y  ,
      c_p*s_y ,  s_r*s_p*s_y + c_r*c_y                 ,  s_y*s_p*c_r - c_y*s_r,
      -s_p    ,  c_p*s_r                               ,  c_r*c_p;
}

inline void rpyToRot(const Eigen::Vector3d& rpy, Eigen::Matrix3d& R)
{
  rpyToRot(rpy(0),rpy(1),rpy(2),R);
}

inline void rpyToRotTranspose(const double& roll, const double& pitch, const double& yaw, Eigen::Matrix3d& R)
{
  R.setZero();

  double c_y = std::cos(yaw);
  double s_y = std::sin(yaw);

  double c_r = std::cos(roll);
  double s_r = std::sin(roll);

  double c_p = std::cos(pitch);
  double s_p = std::sin(pitch);

  R << c_p*c_y               ,  c_p*s_y                ,  -s_p,
      s_r*s_p*c_y - c_r*s_y ,  s_r*s_p*s_y + c_r*c_y  ,  s_r*c_p,
      c_r*s_p*c_y + s_r*s_y ,  c_r*s_p*s_y - s_r*c_y  ,  c_r*c_p;
}

inline void rpyToRotTranspose(const Eigen::Vector3d& rpy, Eigen::Matrix3d& R)
{
  rpyToRotTranspose(rpy(0),rpy(1),rpy(2),R);
}

inline void rollToRot(const double& roll, Eigen::Matrix3d& R)
{
  R.setZero();
  double c_r = std::cos(roll);
  double s_r = std::sin(roll);
  R <<    1   ,    0     	  ,  	  0,
      0   ,    c_r ,  -s_r,
      0   ,    s_r,  c_r;
}

inline void pitchToRot(const double& pitch, Eigen::Matrix3d& R)
{
  R.setZero();
  double c_p = std::cos(pitch);
  double s_p = std::sin(pitch);
  R << c_p 	,	 0  ,   s_p,
      0       ,    1  ,   0,
      -s_p 	,	0   ,  c_p;
}

inline void yawToRot(const double& yaw, Eigen::Matrix3d& R)
{
  R.setZero();
  double c_y = std::cos(yaw);
  double s_y = std::sin(yaw);
  R << c_y,  -s_y ,      0,
      s_y,  c_y ,      0,
      0  ,     0,      1;
}

inline void rollToRotTranspose(const double& roll, Eigen::Matrix3d& R)
{
  R.setZero();
  double c_r = std::cos(roll);
  double s_r = std::sin(roll);
  R <<    1   ,    0     	  ,  	  0,
      0   ,    c_r ,  s_r,
      0   ,    -s_r,  c_r;
}

inline void pitchToRotTranspose(const double& pitch, Eigen::Matrix3d& R)
{
  R.setZero();
  double c_p = std::cos(pitch);
  double s_p = std::sin(pitch);
  R << c_p 	,	 0  ,   -s_p,
      0       ,    1  ,   0,
      s_p 	,	0   ,  c_p;
}

inline void yawToRotTranspose(const double& yaw, Eigen::Matrix3d& R)
{
  R.setZero();
  double c_y = std::cos(yaw);
  double s_y = std::sin(yaw);
  R << c_y,  s_y ,      0,
      -s_y,  c_y ,      0,
      0  ,     0,      1;
}

/**
 * @brief rpyToEarWorld Function to compute the linear tranformation matrix between euler
 * rates (in ZYX convention) and omega vector, where omega is expressed in world
 * coordinates to get the component expressed in the world ortogonal frame.
 * Note: this function was previously called rpyToEarInv
 * @param rpy
 * @return EarWorld
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
 * @brief rpyToEarBase Function to compute the linear tranformation matrix between
 * euler rates (in ZYX convention) and omega vector where omega is expressed
 * in base coordinates (EarBase = base_R_world * EarWorld)
 * Note: this function was previously called rpyToEar
 * @param rpy
 * @return EarBase
 */
inline void rpyToEarBase(const Eigen::Vector3d & rpy, Eigen::Matrix3d& Ear){

  const double& roll  = rpy(0);
  const double& pitch = rpy(1);

  double c_r = std::cos(roll);
  double s_r = std::sin(roll);

  double c_p = std::cos(pitch);
  double s_p = std::sin(pitch);

  Ear<<   1,   0,    -s_p,
      0,   c_r,  c_p*s_r,
      0,  -s_r,  c_p*c_r;
  // XYZ convention:
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
