#include <iostream>
#include <cmath>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;


int main(int argc, char **argv) {

  // 회전 행렬로는 Matrix3d or Matrix3f 사용
  Matrix3d rotation_matrix = Matrix3d::Identity();
  AngleAxisd rotation_vector(M_PI/4, Vector3d(0,0,1));
  rotation_matrix = rotation_vector.toRotationMatrix();
  cout <<rotation_matrix <<endl;
  Vector3d v(1,0,0) ;
  Vector3d af_v = rotation_matrix*v;

  Eigen::Isometry3d T= Isometry3d::Identity();
  T.pretranslate(Vector3d(1,3,4));
  // T.rotate (rotation_matrix);  
  // cout << "Transform matrix = \n" << T.matrix() << endl;
  T.rotate (rotation_vector);  
  cout << "Transform matrix = \n" << T.matrix() << endl;

  Vector3d vv = T*v;
  cout << "v tranformed = " << vv.transpose() << endl;
  // cout <<af_v<<endl;
  // af_v = rotation_vector*v;
  // cout<<af_v<<endl;

  // Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0);
  // cout<<euler_angles<<endl;
  // cout << "yaw pitch roll = " << euler_angles.transpose() << endl;
  // // 회전 벡터 -> 아래의 값은 z축을 기준으로 45' 회전
  // AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));     
  // // cout.precision(3) - 자릿수 정하기
  // cout << "rotation matrix =\n" << rotation_vector.matrix() << endl;  
  // rotation_matrix = rotation_vector.toRotationMatrix();  
  // // 회전벡터를 활용한 좌표변환
  // Vector3d v(1, 0, 0);
  // Vector3d v_rotated = rotation_vector * v;
  // cout << "(1,0,0) after rotation (by angle axis) = " << v_rotated.transpose() << endl;
  // // 회전 행렬 활용한 좌표변환
  // v_rotated = rotation_matrix * v;
  // cout << "(1,0,0) after rotation (by matrix) = " << v_rotated.transpose() << endl;

  // // 회전 행렬을 오일러 각으로 바꾸기
  // Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX 순 -> yaw - pitch - roll 순서
  // cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

  // //Eigen::Isometry
  // Isometry3d T = Isometry3d::Identity();               // 3D 지만 사실 4X4 변환행렬을 의미함
  // T.rotate(rotation_matrix);
  // //T.rotate(rotation_vector);                                     // rotation_vector 로 회전을 표현
  // T.pretranslate(Vector3d(1, 3, 4));                     // 병진 표현
  // cout << "Transform matrix = \n" << T.matrix() << endl;

  // // 변환 행렬을 사용한 좌표 변환
  // Vector3d v_transformed = T * v;                              // R*v+t
  // cout << "v tranformed = " << v_transformed.transpose() << endl;

  // // quaternion을 vector를 통해 정의가능
  // Quaterniond q = Quaterniond(rotation_vector);

   Quaterniond q = Quaterniond(rotation_vector);
   cout<<q.coeffs()<<endl;

  // cout << "quaternion from rotation vector = " << q.coeffs().transpose()
  //      << endl;   // coefficient : (x,y,z,w) 
  // q = Quaterniond(rotation_matrix); // 회전 행렬을 통해서도 할당 가능
  // cout << "quaternion from rotation matrix = " << q.coeffs().transpose() << endl;
  // // quaternion을 활용한 좌표변환
  // v_rotated = q * v; // qvq^{-1}
  // cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;
  
  // cout << "should be equal to " << (q * Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << endl;



  // Vector4d vV(0,1,2,3);
  // Matrix<float ,3,3> M ;
  // M<< 0 , -1*vV[2] ,vV[1] ,vV[2],0,-1*vV[0],vV[1],vV[0],0 ;  

}



// int main(int argc, char **argv) {

//   // 회전 행렬로는 Matrix3d or Matrix3f 사용
//   Matrix3d rotation_matrix = Matrix3d::Identity();
  
//   // // 회전 벡터 -> 아래의 값은 z축을 기준으로 45' 회전
//   // AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));     
//   // // cout.precision(3) - 자릿수 정하기
//   // cout << "rotation matrix =\n" << rotation_vector.matrix() << endl;  
//   // rotation_matrix = rotation_vector.toRotationMatrix();
  
//   // // 회전벡터를 활용한 좌표변환
//   // Vector3d v(1, 0, 0);
//   // Vector3d v_rotated = rotation_vector * v;
//   // cout << "(1,0,0) after rotation (by angle axis) = " << v_rotated.transpose() << endl;
//   // // 회전 행렬 활용한 좌표변환
//   // v_rotated = rotation_matrix * v;
//   // cout << "(1,0,0) after rotation (by matrix) = " << v_rotated.transpose() << endl;

//   // // 회전 행렬을 오일러 각으로 바꾸기
//   // Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX 순 -> yaw - pitch - roll 순서
//   // cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

//   // //Eigen::Isometry
//   // Isometry3d T = Isometry3d::Identity();               // 3D 지만 사실 4X4 변환행렬을 의미함
//   // T.rotate(rotation_matrix);
//   // //T.rotate(rotation_vector);                                     // rotation_vector 로 회전을 표현
//   // T.pretranslate(Vector3d(1, 3, 4));                     // 병진 표현
//   // cout << "Transform matrix = \n" << T.matrix() << endl;

//   // // 변환 행렬을 사용한 좌표 변환
//   // Vector3d v_transformed = T * v;                              // R*v+t
//   // cout << "v tranformed = " << v_transformed.transpose() << endl;

//   // // quaternion을 vector를 통해 정의가능
//   // Quaterniond q = Quaterniond(rotation_vector);



//   // cout << "quaternion from rotation vector = " << q.coeffs().transpose()
//   //      << endl;   // coefficient : (x,y,z,w) 
//   // q = Quaterniond(rotation_matrix); // 회전 행렬을 통해서도 할당 가능
//   // cout << "quaternion from rotation matrix = " << q.coeffs().transpose() << endl;
//   // // quaternion을 활용한 좌표변환
//   // v_rotated = q * v; // qvq^{-1}
//   // cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;
  
//   // cout << "should be equal to " << (q * Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << endl;



//   // Vector4d vV(0,1,2,3);
//   // Matrix<float ,3,3> M ;
//   // M<< 0 , -1*vV[2] ,vV[1] ,vV[2],0,-1*vV[0],vV[1],vV[0],0 ;  

// }