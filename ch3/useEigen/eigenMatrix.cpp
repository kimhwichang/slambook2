#include <iostream>

using namespace std;

#include <ctime>
// Eigen 핵심 부분
#include <Eigen/Core>
//dense matrix의 대수 연산(역, 고유값등)
#include <Eigen/Dense>

using namespace Eigen;

#define MATRIX_SIZE 50

int main(int argc, char **argv) {
  //eigen 기본 형식 -> 데이터 유형,행,렬
  //2x3 float matrix 선언
  Matrix<float, 2, 3> matrix_23;

  //Vector3d 는 사실상 Eigen::Matrix<double, 3, 1>
  Vector3d v_3d;
  
  Matrix<float, 3, 1> vd_3d;

  // Matrix3d ->Eigen::Matrix<double, 3, 3>
  Matrix3d matrix_33 = Matrix3d::Zero(); 
  // Dynamic 차원 지정도 가능
  Matrix<double, Dynamic, Dynamic> matrix_dynamic;
  MatrixXd matrix_x;
  // eigen matrix 값 할당
  matrix_23 << 1, 2, 3, 4, 5, 6;

  //cout << "matrix 2x3 from 1 to 6: \n" << matrix_23 << endl;
  //cout << "print matrix 2x3: " << endl;
//   for (int i = 0; i < 2; i++) {
//     for (int j = 0; j < 3; j++) cout << matrix_23(i, j) << "\t";
//     cout << endl;
//   }

  //vector 값 할당
  v_3d << 3, 2, 1;
  vd_3d << 4, 5, 6;

  // Matrix<double, 2, 1> result_wrong_type = matrix_23 * v_3d; -> float *double 연산 안됨! 반드시 cast 사용
  Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
  //cout << "[1,2,3;4,5,6]*[3,2,1]=" << result.transpose() << endl;

  Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
  //cout << "[1,2,3;4,5,6]*[4,5,6]: " << result2.transpose() << endl;

 
  Eigen::Matrix<double, 2, 1> result_wrong_dimension = matrix_23.cast<double>() * v_3d; //-> wrong dimension도 안됨

  matrix_33 = Matrix3d::Random();      // 随机数矩阵
//   cout << "random matrix: \n" << matrix_33 << endl;
//   cout << "transpose: \n" << matrix_33.transpose() << endl;      // transpose
//   cout << "sum: " << matrix_33.sum() << endl;            // sum
//   cout << "trace: " << matrix_33.trace() << endl;          // trace
//   cout << "times 10: \n" << 10 * matrix_33 << endl;               // scalar 곱
//   cout << "inverse: \n" << matrix_33.inverse() << endl;        // 역 함수
//   cout << "det: " << matrix_33.determinant() << endl;    // determinant 행렬식

  // 고유값
  SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
//   cout << "Eigen values = \n" << eigen_solver.eigenvalues() << endl;
//   cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << endl;

  // 방정식 matrix_NN * x = v_Nd 풀기

  Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN
      = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
  matrix_NN = matrix_NN * matrix_NN.transpose();  // semi-positive definite 보장
  Matrix<double, MATRIX_SIZE, 1> v_Nd = MatrixXd::Random(MATRIX_SIZE, 1);

  clock_t time_stt = clock(); 
  // 역행렬 구하기
  Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
  cout << "time of normal inverse is "
       << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
  //cout << "x = " << x.transpose() << endl;

  // QR분해가 속도가 더 빠름
  time_stt = clock();
  x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
  cout << "time of Qr decomposition is "
       << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
  //cout << "x = " << x.transpose() << endl;

  //정방대칭행렬 (+ semi-positive definite) 대해서는 cholesky분해로 풀수 있다.
  time_stt = clock();
  x = matrix_NN.ldlt().solve(v_Nd);
  cout << "time of ldlt decomposition is "
       << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
  //cout << "x = " << x.transpose() << endl;

  return 0;
}

