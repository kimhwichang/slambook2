#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

Matrix3d quater_to_rot_mat (Quaterniond q);
AngleAxisd quater_to_rot_vec(Quaterniond q);
float quaternion_size(Quaterniond q);
Matrix3d skew_symmetric(Vector3d v);
int main(int argc, char** argv){

Quaterniond q_1 (0.35,0.2,0.3,0.1);
Quaterniond q_2 (-0.5,0.4,-0.1,0.2);

Vector3d v_t_1 (0.3,0.1,0.1);
Vector3d v_t_2 (-0.1,0.5,0.3);
Vector3d coord (0.5,0,0.2);
Isometry3d T_r1w = Isometry3d::Identity();

cout<<"quater to rot" <<quater_to_rot_vec(q_1).matrix()<<endl;
cout<<endl;
cout<<"quater to mat" <<quater_to_rot_mat(q_1)<<endl;
cout<<endl;

T_r1w.rotate(quater_to_rot_vec(q_1));
T_r1w.pretranslate(v_t_1);

q_2.normalize();
Isometry3d T_r2w(q_2);
T_r2w.pretranslate(v_t_2);

Vector3d transformed_coord = T_r2w*T_r1w.inverse()*coord;
cout<<transformed_coord <<endl;

}

AngleAxisd quater_to_rot_vec(Quaterniond q){
  q.normalize();
  //cout<<q.coeffs()<<endl;
  float theta = 2*acos(q.coeffs()[3]);
  
  Vector3d rot_axis(q.coeffs()[0],q.coeffs()[1],q.coeffs()[2]);
  rot_axis = rot_axis / sqrt( rot_axis[0]*rot_axis[0] +   rot_axis[1]*rot_axis[1]+  rot_axis[2]*rot_axis[2]);
  AngleAxisd rot_axis_vec(theta, rot_axis);
  //cout<< rot_axis_vec.toRotationMatrix()<<endl;

  return rot_axis_vec;

}

Matrix3d quater_to_rot_mat (Quaterniond q){
  q.normalize();
  Vector4d normalized_q =q.coeffs();
  Matrix3d r_m = Matrix3d::Identity();
  double q_s  = normalized_q[3];
  Vector3d q_i (normalized_q[0],normalized_q[1],normalized_q[2]); 
  Matrix3d skew_m = skew_symmetric(q_i); 
  r_m = q_i*q_i.transpose() + pow(q_s,2)*Matrix3d::Identity() + 2*q_s*skew_m +skew_m*skew_m;
  return r_m;

}

float quaternion_size(Quaterniond q){
  return sqrt(pow(q.coeffs()[0],2)+pow(q.coeffs()[1],2)+pow(q.coeffs()[2],2)+pow(q.coeffs()[3],2));

}

Matrix3d skew_symmetric(Vector3d v){
    Matrix3d skew_m;
    skew_m<< 0 , -1*v[2], v[1] ,v[2],0,-1*v[0],-1*v[1],v[0],0 ;
    return skew_m;
}


// int main(int argc, char** argv) {
//   Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5, 0.4, -0.1, 0.2);
//   q1.normalize();
//   q2.normalize();
//   Vector3d t1(0.3, 0.1, 0.1), t2(-0.1, 0.5, 0.3);
//   Vector3d p1(0.5, 0, 0.2);

//   Isometry3d T1w(q1), T2w(q2);
//   T1w.pretranslate(t1);
//   T2w.pretranslate(t2);

//   Vector3d p2 = T2w * T1w.inverse() * p1;
//   cout << endl << p2.transpose() << endl;
//   return 0;
// }