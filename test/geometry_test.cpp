#include <gtest/gtest.h>
#include "target_estimation/geometry.hpp"

Eigen::Affine3d generateRandomPose()
{
  Eigen::AngleAxisd rot;
  rot.axis().setRandom(); // create a random rotation axis
  rot.axis() /= rot.axis().norm(); // normalize it
  Eigen::Matrix2d random;
  random.setRandom();
  rot.angle() = random(0); // set a random angle
  Eigen::Affine3d pose, actual_pose;
  pose.linear() = rot.toRotationMatrix();
  pose.translation().setRandom(); // set a random position
  return pose;
}

double generateRandomAngle()
{
  Eigen::VectorXd random(1);
  random.setRandom();
  return random(0);
}

TEST(test_orientation, rotToQuatToRot )
{
  for(int i = 0; i < 100; i++){
    auto pose = generateRandomPose();
    Eigen::Quaterniond q;
    Eigen::Matrix3d R;

    rotToQuat(pose.linear(),q);
    quatToRot(q,R);
    EXPECT_TRUE( pose.linear().isApprox(R, 0.0001) );
  }
}

TEST(test_orientation, compareToEigenQuat )
{
  for(int i = 0; i < 100; i++){
    auto pose = generateRandomPose();
    Eigen::Quaterniond q, q_actual;
    Eigen::Matrix3d R, R_actual;

    rotToQuat(pose.linear(),q);
    q_actual = pose.linear();
    EXPECT_TRUE( q.isApprox(q_actual, 0.0001) );

    quatToRot(q,R);
    R_actual = q_actual.toRotationMatrix();
    EXPECT_TRUE( R.isApprox(R_actual, 0.0001) );
  }
}

TEST(test_orientation, quatToRpyToQuat )
{
  for(int i = 0; i < 100; i++){
    auto pose = generateRandomPose();
    Eigen::Vector3d rpy;
    Eigen::Quaterniond q, q_actual;
    q = pose.linear();
    quatToRpy(q,rpy);
    rpyToQuat(rpy,q_actual);
    EXPECT_TRUE( q.isApprox(q_actual, 0.0001) );
  }
}

TEST(test_orientation, singleRotationsRPY )
{
  for(int i = 0; i < 100; i++){
    auto angle = generateRandomAngle();
    Eigen::Matrix3d R;
    Eigen::Vector3d rpy, rpy_actual;

    rpy_actual << angle, 0.0, 0.0; // ROLL
    rollToRotTranspose(angle,R);
    rotTransposeToRpy(R,rpy);
    EXPECT_TRUE( rpy.isApprox(rpy_actual, 0.0001) );

    rpy_actual << 0.0, angle, 0.0; // PITCH
    pitchToRotTranspose(angle,R);
    rotTransposeToRpy(R,rpy);
    EXPECT_TRUE( rpy.isApprox(rpy_actual, 0.0001) );

    rpy_actual << 0.0, 0.0, angle; // YAW
    yawToRotTranspose(angle,R);
    rotTransposeToRpy(R,rpy);
    EXPECT_TRUE( rpy.isApprox(rpy_actual, 0.0001) );

    rpy_actual << angle, 0.0, 0.0; // ROLL
    rollToRot(angle,R);
    rotToRpy(R,rpy);
    EXPECT_TRUE( rpy.isApprox(rpy_actual, 0.0001) );

    rpy_actual << 0.0, angle, 0.0; // PITCH
    pitchToRot(angle,R);
    rotToRpy(R,rpy);
    EXPECT_TRUE( rpy.isApprox(rpy_actual, 0.0001) );

    rpy_actual << 0.0, 0.0, angle; // YAW
    yawToRot(angle,R);
    rotToRpy(R,rpy);
    EXPECT_TRUE( rpy.isApprox(rpy_actual, 0.0001) );
  }
}

TEST(test_orientation, rotationsRPY )
{
  for(int i = 0; i < 100; i++){
    auto angle = generateRandomAngle();
    Eigen::Matrix3d R, R_actual;
    Eigen::Vector3d rpy, rpy_actual;

    rpy_actual << angle, angle, angle;
    Eigen::AngleAxisd rollAngle(rpy_actual(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rpy_actual(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rpy_actual(2), Eigen::Vector3d::UnitZ());
    R_actual = yawAngle * pitchAngle * rollAngle;

    rpy << angle, angle, angle;

    rpyToRotTranspose(rpy,R);
    R.transposeInPlace();
    EXPECT_TRUE( R.isApprox(R_actual, 0.0001) );

    rpyToRot(rpy,R);
    EXPECT_TRUE( R.isApprox(R_actual, 0.0001) );
  }
}

TEST(test_orientation, rpyToQuatToRpy )
{
  for(int i = 0; i < 100; i++){
    auto angle = generateRandomAngle();
    Eigen::Matrix3d R, R_actual, Rx, Ry, Rz;
    Eigen::Vector3d rpy, rpy_actual;
    Eigen::Quaterniond q_actual;

    rollToRot(angle,Rx);
    pitchToRot(angle,Ry);
    yawToRot(angle,Rz);

    R_actual = Ry * Ry * Rx;
    rpy_actual << angle, angle, angle;

    rpyToQuat(rpy_actual,q_actual);
    quatToRpy(q_actual,rpy);

    EXPECT_TRUE( rpy_actual.isApprox(rpy, 0.0001) );
  }
}

TEST(test_orientation, rpyToQuat )
{
  for(int i = 0; i < 100; i++){
    auto angle = generateRandomAngle();
    Eigen::Matrix3d  R_actual;
    Eigen::Vector3d rpy;
    Eigen::Quaterniond q, q_actual;

    rpy << angle, angle, angle;

    rpyToRotTranspose(rpy,R_actual);
    R_actual.transposeInPlace();
    rotToQuat(R_actual,q_actual);

    rpyToQuat(rpy,q);

    EXPECT_TRUE( q_actual.isApprox(q, 0.0001) );
  }
}

TEST(test_orientation, quatToRpy )
{
  for(int i = 0; i < 100; i++){
    auto pose = generateRandomPose();
    Eigen::Quaterniond q, q_actual;
    Eigen::Vector3d rpy, rpy_actual;

    q_actual = pose.linear();

    rotTransposeToRpy(pose.linear().transpose(),rpy_actual);

    quatToRpy(q_actual,rpy);

    EXPECT_TRUE( rpy_actual.isApprox(rpy, 0.0001) );
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  return ret;
}
