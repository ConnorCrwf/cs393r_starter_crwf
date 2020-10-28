#include <stdio.h>
#include <iostream>
#include <cmath>
#include "../shared/math/math_util.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"


using std::cout;
using std::endl;
using std::sqrt;
using std::sin;
using std::cos;
using math_util::AngleDiff;

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Affine2f;

// using Eigen::Transform;
// using Eigen::Affine;

struct Pose{
  Eigen::Vector2f loc;
  float angle;
};

Affine2f GetTransform(Pose pose);

void TransformCoordTest_SplitOperations() {
  cout << "Initialize Scan Vector" << endl;
  Vector2f Scan2Lidar(1.5, 1.5) ;
  cout << "Print the vector to stdout:\n" << Scan2Lidar << endl;

  cout << "Initialize Rotation Matrix from Lidar Frame into Base Link Frame." << endl;
  Matrix2f R_Lidar2Base;
  R_Lidar2Base  << 1,0,
                    0,1;
  cout << "R_Lidar2Base = " << endl << R_Lidar2Base << endl;

  cout << "Multiply R_Lidar2Base * Scan2Lidar." << endl;
  Vector2f Scan2Base_Aligned = R_Lidar2Base * Scan2Lidar;

  cout << "Add in Lidar origin translation relative to base link frame" << endl;
  const Vector2f Tr_Lidar2Base(0.2, 0);
  Vector2f Scan2Base = Scan2Base_Aligned + Tr_Lidar2Base;

  cout << "Resulting vector:\n" << Scan2Base << endl;

}

void TransformCoordTest_HgManual() {
  cout << "Now using 3x3 Homogeneous Transform" << endl;
  Matrix3f H_Lidar2Base;
  H_Lidar2Base << 1,0,0.2,
                    0,1,0,
                    0,0,1;
  cout << "H_Lidar2Base = " << endl << H_Lidar2Base << endl;
  Vector3f Scan2Lidar(1.5, 1.5,1) ;
  Vector3f Scan2Base = H_Lidar2Base*Scan2Lidar;
  cout << "Resulting vector:\n" << Scan2Base << endl;
}

void TransformCoordTest_HgAffine() {
  cout << "Now building 3x3 Homogeneous Transform with Affine matrix that takes in Rotation and Translation matrices" << endl;
//   if want to rotate it 45deg set equal to M_PI / 4.0 
  float angle = 0;
  cout << "angle = " << angle << " radians = "
       << angle / M_PI * 180.0 << " degrees." << endl;
  cout << "Create a rotation" << endl;
  Rotation2Df R_Lidar2Base(angle);
// or can just use .matrix()
  cout << "R_Lidar2Base = " << endl << R_Lidar2Base.toRotationMatrix() << endl;


  Vector2f Tr_Lidar2Base(0.2 , 0);
  //   or use, but must add .vector() in print statement
  //   Translation2f Translation_Lidar2Base(Vector2f (0.2,0));
  cout << "Tr_Lidar2Base = " << endl << Tr_Lidar2Base << endl;

  Affine2f H_Lidar2Base = Eigen::Affine2f::Identity();
//   or use
//   Transform <float , 2 , Affine > t = Transform <float , 2 , Affine >::Identity () ;
  H_Lidar2Base . scale ( 1.0f ) ;
  H_Lidar2Base . rotate ( R_Lidar2Base ) ;
// Euler can also be used if want to rotate about a specific access
// it's Euler even though it says AngleAxisf
// H_Lidar2Base . rotate ( AngleAxisf (0.25 f * M_PI , Vector3f :: UnitX () ) ) ;
  H_Lidar2Base . translate ( Tr_Lidar2Base ) ;
  cout << "H_Lidar2Base = " << endl << H_Lidar2Base.matrix() << endl;
  Vector3f Scan2Lidar(1.5, 1.5,1) ;
  Vector3f Scan2Base = H_Lidar2Base*Scan2Lidar;
  cout << "Resulting vector:\n" << Scan2Base << endl;
}

void TransformCoordTest_HgAffineCombined()
{
  cout << "Now combining Transforms to go from current Base to previous Base" << endl;

// 1st Transform
  float angle_Lidar2Base = 0;
  Rotation2Df R_Lidar2BaseCur(angle_Lidar2Base);
  Vector2f Tr_Lidar2BaseCur(0.2 , 0);
  Affine2f H_Lidar2BaseCur = Eigen::Affine2f::Identity();
  H_Lidar2BaseCur . rotate ( R_Lidar2BaseCur ) ;
  H_Lidar2BaseCur . translate ( Tr_Lidar2BaseCur ) ;
  cout << "H_Lidar2Base = " << endl << H_Lidar2BaseCur.matrix() << endl; 


// 2nd Transform
  float angleBaseCur2BasePrev = M_PI / 4.0;
  Rotation2Df R_BaseCur2BasePrev(angleBaseCur2BasePrev);
  Vector2f Tr_BaseCur2BasePrev(0.4 , 0);
  Affine2f H_BaseCur2BasePrev = Eigen::Affine2f::Identity();
  H_BaseCur2BasePrev . rotate ( R_BaseCur2BasePrev ) ;
  H_BaseCur2BasePrev . translate ( Tr_BaseCur2BasePrev ) ;
  cout << "H_BaseCur2BasePrev = " << endl << H_BaseCur2BasePrev.matrix() << endl;
  
//   Perform Transfomration
  Vector3f Scan2Lidar(1.5, 1.5,1) ;
  Vector3f Scan2BasePrev = H_BaseCur2BasePrev*H_Lidar2BaseCur*Scan2Lidar;
  cout << "Resulting vector:\n" << Scan2BasePrev << endl;
}

Eigen::Vector2f TransformCoordTest_HgAffineCombined_Param(const Eigen::Vector2f scan_loc, Pose odom_pose_cur)

{
    // 1st Transform
    Affine2f H_Lidar2BaseCur = GetTransform({{0.2,0.0},0});

    // 2nd Transform
    Vector2f prev_odom_loc_ = {0,0};
    float prev_odom_angle_ = M_PI / 4;
    Vector2f odom_trans_diff = odom_pose_cur.loc - prev_odom_loc_;
    float odom_angle_diff = AngleDiff(odom_pose_cur.angle, prev_odom_angle_);
    Pose diff_pose = {odom_trans_diff,odom_angle_diff};
    Affine2f H_BaseCur2BasePrev = GetTransform(diff_pose);

    // Apply transformations to Scan Point
    Vector3f Scan2Lidar_xyz(scan_loc.x(),scan_loc.y(),1) ;
    Vector3f Scan2BasePrev_xyz = H_BaseCur2BasePrev*H_Lidar2BaseCur*Scan2Lidar_xyz;
    Vector2f Scan2BasePrev(Scan2BasePrev_xyz.x(),Scan2BasePrev_xyz.y());
    return Scan2BasePrev;
}

Eigen::Affine2f GetTransform(Pose pose) {
    // in radians
    Rotation2Df R(pose.angle);
    Vector2f Tr = pose.loc;
    Affine2f H = Eigen::Affine2f::Identity();
    H.rotate ( R ) ;
    H.translate ( Tr ) ;
    // cout << "H = " << endl << H.matrix() << endl;
	return H;
}

int main() {
  cout << "Convert Scan2Lidar to Scan2BasePrev\n";
//   TransformCoordTest_SplitOperations();
//   TransformCoordTest_HgManual();
//   TransformCoordTest_HgAffine();
//   TransformCoordTest_HgAffineCombined();
//   Affine2f Test = GetTransform({{0.2,0.0},0});
  Vector2f test_scan(1.5, 1.5) ;
  Pose test_pose = {{0.4,0.0},M_PI / 4.0};
  Vector2f Test = TransformCoordTest_HgAffineCombined_Param(test_scan,test_pose);
  cout << "H_BaseCur2BasePrev = " << endl << Test << endl;
  return 0;
}
