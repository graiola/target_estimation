#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include "target_estimation/utils.hpp"
#include <Eigen/Geometry>

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

inline void rpyToQuat(const Eigen::Vector3d rpy, Eigen::Quaterniond& q)
{
  double phi, the, psi;
  phi = rpy(0) / 2;
  the = rpy(1) / 2;
  psi = rpy(2) / 2;
  q.w() = std::cos(phi) * std::cos(the) * std::cos(psi) + std::sin(phi) * std::sin(the) * std::sin(psi);
  q.x() = std::sin(phi) * std::cos(the) * std::cos(psi) - std::cos(phi) * std::sin(the) * std::sin(psi);
  q.y() = std::cos(phi) * std::sin(the) * std::cos(psi) + std::sin(phi) * std::cos(the) * std::sin(psi);
  q.z() = std::cos(phi) * std::cos(the) * std::sin(psi) - std::sin(phi) * std::sin(the) * std::cos(psi);
  q.normalize();
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
    const double& yaw   = rpy(2);

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
    // XYZ convetion:
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

inline Eigen::Matrix3x6d EarBaseInvJacobian(const Eigen::Vector3d& rpy, const Eigen::Vector3d& omega, const double& dt){

    Eigen::Matrix3x6d out = Eigen::Matrix3x6d::Zero();
    double wy = omega(1);
    double wz = omega(2);
    double c_r  = std::cos(rpy(0));
    double c_p  = std::cos(rpy(1));
    double s_r  = std::sin(rpy(0));
    double s_p  = std::sin(rpy(1));

    out<<(dt*(wy*c_r*s_p - wz*s_p*s_r))/c_p + 1,        (dt*(wz*c_r + wy*s_r))/(c_p*c_p), 0, dt, (dt*s_p*s_r)/c_p, (dt*c_r*s_p)/c_p,
                     -dt*(wz*c_r + wy*s_r),                                            1, 0,  0,                 dt*c_r,                -dt*s_r,
                     (dt*(wy*c_r - wz*s_r))/c_p, (dt*s_p*(wz*c_r + wy*s_r))/(c_p*c_p), 1,  0,        (dt*s_r)/c_p,        (dt*c_r)/c_p;

    return out;
}


inline Eigen::Matrix3d EarBaseInvJacobianRpy(const Eigen::Vector3d& rpy, const Eigen::Vector3d& omega, const double& dt){

    Eigen::Matrix3d out = Eigen::Matrix3d::Zero();
    double wy = omega(1);
    double wz = omega(2);
    double c_r  = std::cos(rpy(0));
    double c_p  = std::cos(rpy(1));
    double s_r  = std::sin(rpy(0));
    double s_p  = std::sin(rpy(1));


    out<<   (dt*(wy*c_r*s_p - wz*s_p*s_r))/c_p + 1,        (dt*(wz*c_r + wy*s_r))/(c_p*c_p), 0,
                                -dt*(wz*c_r + wy*s_r),                                            1, 0,
                        (dt*(wy*c_r - wz*s_r))/c_p, (dt*s_p*(wz*c_r + wy*s_r))/(c_p*c_p), 1;

    return out;
}

inline Eigen::Matrix3d EarBaseInvJacobianOmega(const Eigen::Vector3d& rpy, const double& dt){

    Eigen::Matrix3d out = Eigen::Matrix3d::Zero();
    double c_r  = std::cos(rpy(0));
    double c_p  = std::cos(rpy(1));
    double s_r  = std::sin(rpy(0));
    double s_p  = std::sin(rpy(1));


    out <<    dt, (dt*s_p*s_r)/c_p, (dt*c_r*s_p)/c_p,
                         0,                 dt*c_r,                -dt*s_r,
                         0,        (dt*s_r)/c_p,        (dt*c_r)/c_p;

    return out;
}

inline Eigen::Matrix3d selfCross(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d cross;

    cross << 0, -v(2), v(1),
             v(2), 0,  -v(0),
             -v(1), v(0), 0;

    return cross;
}

/**
 * @brief omegaToMatrix Function to compute the omega matrix S(omega) from the angular
 * velocities [omega_x omega_y omega_z], such that:
 * \dot{q} = 0.5 * S(omega) q
 * Note1: we assume that the quaternion has the following convention: [qx qy qz qw]
 * which is the same as Eigen::Quaterniond. If you need the matrix for [qw qx qy qz] check the comment below.
 * @param omega
 * @return 0.5 * S(omega) 4x4 matrix
 */
inline Eigen::Matrix4d omegaToMatrix(const Eigen::Vector3d& omega)
{
    Eigen::Matrix4d S;
    S.setZero();
    S <<  0,        -omega(2), omega(1),   omega(0),
            omega(2), 0,         -omega(0),  omega(1),
            -omega(1), omega(0),  0,          omega(2),
            -omega(0), -omega(1), -omega(2),  0;

    //S <<   0,          -omega(0),  -omega(1), -omega(2),
    //       omega(0),  0,           -omega(2),  omega(1),
    //       omega(1),  omega(2),   0,           -omega(0),
    //       omega(2),  -omega(1),  omega(0),   0;

    S = 0.5 * S;

    return S;
}

inline Eigen::Matrix4x3d quaternionToMatrix(const Eigen::Quaterniond& q)
{
    Eigen::Matrix4x3d S;
    S.setZero();
    S <<  q.w(),  -q.z(),  q.y(),
          q.z(),   q.w(), -q.x(),
         -q.y(),   q.x(),  q.w(),
         -q.x(),  -q.y(), -q.z();

    return S;
}

/**
 * @brief Qtran Function to compute the transition matrix for quaternions such that:
 * \dot{q} = S(omega) q
 * If we discretize the expression:
 * q_{k+1} = exp(S(omega)*dt) q_{k}
 * By using the taylor series on the exponential, we obtain the transition matrix:
 * q_{k+1} = Qtran(omega,dt) q_{k}
 * Note: we assume that the quaternion has the following convention: [qx qy qz qw]
 * which is the same as Eigen::Quaterniond.
 * Note2: Check the comment for the linear approximation using only the first term of the taylor series.
 * @param dt
 * @param omega
 * @return Qtran(omega,dt) 4x4 matrix
 */
inline Eigen::Matrix4d Qtran(const double& dt, const Eigen::Vector3d& omega)
{
    double omega_norm = omega.norm();
    double tmp = omega_norm*dt/2.0;
    auto S = omegaToMatrix(omega);
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    if (omega_norm > 0.0)
        //return (I + S * dt); // First term approximation
        return (std::cos(tmp) * I + 2.0/omega_norm * std::sin(tmp) * S);
    else
        return I;
}

//inline Eigen::Matrix4x3d QtranOmega(const Eigen::Quaterniond q)
//{
//    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
//    Eigen::Matrix4x3d E;
//    E.setZero();
//    Eigen::Matrix3d cross;
//
//    cross << 0, -q.vec()(2), q.vec()(1),
//             q.vec()(2), 0,  -q.vec()(0),
//             -q.vec()(1), q.vec()(0), 0;
//
//    E.block<3,3>(0,0) = q.w() * I + cross; // FIXME
//    E.block<1,3>(3,0) = -q.vec().transpose();
//
//    return E;
//}

/**
 * @brief Qtrandot Function to compute the time derivative of Qtran
 * @param dt
 * @param omega
 * @return Qtrandot(omega,dt) 4x4 matrix
 */
inline Eigen::Matrix4d Qtrandot(const double& dt, const Eigen::Vector3d& omega)
{
    double omega_norm = omega.norm();
    double tmp = omega_norm*dt/2.0;
    auto S = omegaToMatrix(omega);
    if (omega_norm > 0.0)
        return (std::cos(tmp) * S - omega_norm/2.0 * std::sin(tmp) * Eigen::Matrix4d::Identity());
    else
        return Eigen::Matrix4d::Zero();
}

/**
 * @brief Qomega Function to compute the partial derivative of Qtran * q with respect to omega
 * Note: we assume that the quaternion has the following convention: [qx qy qz qw]
 * which is the same as Eigen::Quaterniond.
 * It is possible to use the first term of the taylor series approximation, in that case check the commented
 * matrices.
 * @param dt
 * @param omega
 * @return Qomega(omega,dt) 4x3 matrix
 */
inline Eigen::Matrix4x3d Qomega(const double& dt, const Eigen::Vector3d& omega, const Eigen::Vector4d& q)
{
    Eigen::Matrix4x3d Qomega;
    Qomega.setZero();
    double omega_norm = omega.norm();
    double tmp = omega_norm*dt/2.0;
    auto S  = omegaToMatrix(omega);
    Eigen::Vector3d omega_dot;

    if (omega_norm > 0.0)
        for(unsigned int col = 0; col < 3; col++)
        {
            omega_dot.setZero();
            omega_dot(col) = 1.0;
            Qomega.col(col) = ( ((-omega(col)*dt)/(2*omega_norm))*std::sin(tmp) *  Eigen::Matrix4d::Identity()  +  ( ((omega(col)*dt)/(omega_norm*omega_norm))*std::cos(tmp) - (omega(col)/(omega_norm*omega_norm*omega_norm))*std::sin(tmp) ) * S
                                + ((2/omega_norm)*std::sin(tmp))*omegaToMatrix(omega_dot) ) * q;
        }

    // For [qx qy qz qw]
    //Qomega << q(3),  q(2), -q(1),
    //         -q(2),  q(3),  q(0),
    //          q(1), -q(0),  q(3),
    //         -q(0), -q(1), -q(2);
    //
    // For [qw qx qy qz]
    //Qomega << q(1), -q(2), -q(3),
    //          q(0),  q(3), -q(2),
    //         -q(3),  q(0),  q(1),
    //          q(2),  q(1),  q(0);
    //Qomega = 0.5 * dt * Qomega;

    return Qomega;
}

/**
 * @brief isometry2vector Convert an isometry transform into a vector composed by
 * [x y z quat_x quat_y quat_z quat_w]
 * @param T
 * @param v
 */
inline void isometryToPose7d(const Eigen::Isometry3d& T, Eigen::Vector7d& p)
{
    POSE_pos(p)  = T.translation();
    POSE_quat(p) = ((Eigen::Quaterniond)T.linear()).coeffs();
}

/**
 * @brief isometry2vector Convert an isometry transform into a vector composed by
 * [x y z roll pitch yaw]
 * @param T
 * @param v
 */
inline void isometryToPose6d(const Eigen::Isometry3d& T, Eigen::Vector6d& p)
{
    POSE_pos(p) = T.translation();
    Eigen::Vector3d rpy;
    rotToRpy(T.rotation(),rpy);
    POSE_rpy(p) = rpy;
}

inline void pose7dToIsometry(const Eigen::Vector7d& p, Eigen::Isometry3d& T)
{
    T.translation() = POSE_pos(p);
    Eigen::Quaterniond q;
    q.coeffs() = POSE_quat(p);
    q.normalize();
    T.linear() = q.toRotationMatrix();
}

inline void pose7dToPose6d(const Eigen::Vector7d& p7d, Eigen::Vector6d& p6d)
{
    POSE_pos(p6d) = POSE_pos(p7d);
    Eigen::Quaterniond q;
    Eigen::Vector3d rpy;
    q.coeffs() = POSE_quat(p7d);
    q.normalize();
    quatToRpy(q,rpy);
    POSE_rpy(p6d) = rpy;
}

inline Eigen::Quaterniond computeQuaternionError(const Eigen::Quaterniond& q_des,
                                                 const Eigen::Quaterniond& q)
{
    Eigen::Quaterniond q_e;
    //int sign_flip = 1;
    //
    //// to the test to take the shortest path in the quaternion
    //if (q.dot(q_des) < 0) {
    //    sign_flip = -1;
    //}
    ////compute angular error qe = qd x q1_inv
    //q_e.w() = (sign_flip*q.w())*q_des.w() + (sign_flip*q.vec()).transpose()*q_des.vec();
    //
    ////compute rotation axis
    //q_e.vec() = (sign_flip*q.w())*q_des.vec() - q_des.w()*q.vec() - (sign_flip*q.vec()).cross(q_des.vec());

    q_e = q_des * q.inverse();

    q_e.normalize();

    return q_e;
}

inline double computeQuaternionErrorAngle(const Eigen::Quaterniond& q_des,
                                          const Eigen::Quaterniond& q)
{
    return 2*std::acos(computeQuaternionError(q_des,q).w());
}

inline Eigen::Vector7d computePoseError(const Eigen::Vector7d& p_des,
                                        const Eigen::Vector7d& p)
{
    Eigen::Quaterniond q_e;
    Eigen::Vector7d p_e;
    Eigen::Quaterniond q_des;
    q_des.coeffs() = POSE_quat(p_des);
    Eigen::Quaterniond q;
    q.coeffs() = POSE_quat(p);

    q_e = computeQuaternionError(q_des,q);

    POSE_pos(p_e) = POSE_pos(p_des) - POSE_pos(p);
    POSE_quat(p_e) = q_e.coeffs();

    return p_e;
}


#endif
