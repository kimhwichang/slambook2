#include <iostream>
#include <fstream>  
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;


// int main(int argc, char **argv) {

//   //SO3 정의 -> 회전벡터로 부터 , Quaternion으로 부터

//   Matrix3d R = AngleAxisd(M_PI/2,Vector3d(0,0,1)).toRotationMatrix();
//   Quaterniond q(R);

//   Sophus::SO3d SO3_R(R);
//   Sophus::SO3d SO3_q(q);
  
//   // cout << "SO(3) from matrix:\n" << SO3_R.matrix() << endl;
//   // cout << "SO(3) from quaternion:\n" << SO3_q.matrix() << endl;
//   // cout << "they are equal" << endl;

//   //log mapping을 통한 리 대수

//   Vector3d so3 = SO3_R.log();
//   cout << "so3 = " << so3.transpose() << endl;
//   // // hat :  vector -> skew-symmetric
//   // cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) << endl;
//   // // vee : skew-symmetric -> vector 
//   // cout << "so3 hat vee= " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

//   // 증분 섭동 모델 업데이트 -> 극소량만큼 업데이트
//   Vector3d update_so3(1e-4, 0, 0); 
//   Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R; // 변화된 회전행렬
//   //cout << "SO3 updated = \n" << SO3_updated.matrix() << endl;
//   //cout << "*******************************" << endl;
//   // SE(3) 리대수
//   Vector3d t(1, 0, 0);           
//   Sophus::SE3d SE3_Rt(R, t);           // 회전행렬을 통한 SE(3)정의 
//   Sophus::SE3d SE3_qt(q, t);            // 쿼터니언을 통한 SE(3) 정의
//   cout << "SE3 from R,t= \n" << SE3_Rt.matrix() << endl;
//   cout << "SE3 from q,t= \n" << SE3_qt.matrix() << endl;

//   typedef Eigen::Matrix<double, 6, 1> Vector6d;
//   Vector6d se3 = SE3_Rt.log();  // rho , phi -> 6d vector
//   cout << "se3 = " << se3.transpose() << endl;
//   cout << "se3 hat = \n" << Sophus::SE3d::hat(se3) << endl; // hat : 리 대수 -> skew - symmetric
//   cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl; // vee : skew-symmetric -> 벡터

//   Vector6d update_se3; 
//   update_se3.setZero();
//   update_se3(0, 0) = 1e-4;
//   Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
//   cout << "SE3 updated = " << endl << SE3_updated.matrix() << endl;

//   return 0;
// }

int main(int argc , char** argv){

  Matrix3d R = AngleAxisd(M_PI/2 , Vector3d(0,0,1)).toRotationMatrix();
  Quaterniond q(R);

  Sophus::SO3d SO3_r(R);
  Sophus::SO3d SO3_q(q);

  cout << "SO3 from Rotation matrix : " << SO3_r.matrix() <<endl;
  cout<< "SO3 from Quaternion : " << SO3_q.matrix() <<endl;

  Vector3d so3 = SO3_r.log();
  cout<< "so3 = " <<so3.transpose()<<endl;
  cout<< " so3 skew = " <<Sophus::SO3d::hat(so3)<<endl;
  cout<< " so3 vector = " <<Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose()<<endl;

  Vector3d update_so3(1e-4,0,0);
  Sophus::SO3d updated_so3 = Sophus::SO3d::exp(update_so3)*SO3_r;  // log, exp 둘다 vector 단으로 바로 적용
  cout<<"updated so3 = " <<updated_so3.matrix() <<endl;

  Vector3d t(0,0,1);
  Sophus::SE3d SE3_r(R,t);
  Sophus::SE3d SE3_q(q,t);
  typedef Eigen::Matrix<double,6,1> Vector6d;
  Vector6d se3 = SE3_r.log();
  cout<<"se3 = "<<se3.transpose()<<endl;
  cout<<"se3 hat "<<Sophus::SE3d::hat(se3)<<endl;
  cout<<"se3 vee " <<Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose()<<endl;

  Vector6d update_se3;
  update_se3.setZero();
  update_se3(0,0)= 1e-4;
  Sophus::SE3d updated_se3 = Sophus::SE3d::exp(update_se3)*SE3_r;
  cout << "SE3 updated = " << endl << updated_se3.matrix() << endl;
  return 0;



}





