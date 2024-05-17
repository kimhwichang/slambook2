//
// Created by xiang on 18-11-19.
//

#include <iostream>
//#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>
#include <random>
using namespace std;

// 비용 함수 계산 모델
struct CURVE_FITTING_COST{   // functor -> 연산자 ()가 오버로드 되어있으므로 마치 함수인 것처럼 호출 가능
  CURVE_FITTING_COST(double x, double y):_x(x),_y(y){}
  template <typename T>
  bool operator()(const T*const abc, T*residual ) const {
    residual[0] = T(_y) - ceres::exp(abc[0]*T(_x)*T(_x)+abc[1]*T(_x)+abc[2]);
    return true;

  }
  const double _x,_y;
};

int main(int argc, char **argv) {
  int num_cameras = 3;
  int num_points = 3;
  double * cameras = new double[9*num_cameras + 3*num_points];
  int * camera_index = new int[5];
  for (int i=0; i<36;i++){
  cameras[i] = i;
  }

  for (int i=0; i<5;i++){
  camera_index[i] = i;
  }
  double *camera = cameras + 9;// 9*camera_index[1];
  
  for (int i=0; i<9;i++){
   std::cout<<camera[i]<<std::endl;
  }


 
  // double ar = 1.0, br = 2.0, cr = 1.0;         // 실제 변수 값
  // double ae = 2.0, be = -1.0, ce = 5.0;        // 예상 매개변수 값
  // int N = 100;                                 // 데이터 갯수
  // double w_sigma = 1.0;                        // 노이즈의 시그마
  // double inv_sigma = 1.0 / w_sigma;
  // //cv::RNG rng;                                 // OpenCV随机数产生器
  // std::random_device rd; 
  // std::mt19937 gen(rd()); 
  // std::normal_distribution<float> d(0, w_sigma); 
  // float sample = d(gen); 
  // //data에 집어넣기 
  // vector<double> x_data , y_data;
  // for(int i=0; i<N; i++){
  //   double x = i/100.0;
  //   x_data.push_back(x);
  //   y_data.push_back(exp(ar*x*x+br*x+cr)+sample);

  // }  
  // double abc[3] = {ae, be, ce};

  // // 최소제곱 문제 생성
  // ceres::Problem problem;
  // for(int i=0; i<N; i++){
  //   problem.AddResidualBlock(   // 목적 함수에 오류 항목 추가 & 미분 방법 선택 :  자동 미분 / 수치적 미분/ 해석적 미분 선택 가능 
  //     new ceres::AutoDiffCostFunction<CURVE_FITTING_COST,1,3>(new CURVE_FITTING_COST(x_data[i],y_data[i])) // 오류는 스칼라 차원 1 / 입력 변수는 차원 3
  //     ,nullptr, // 커널함수
  //     abc // 추정대상 매개변수
  //   );

  //}
  //solver 구성
  
  // ceres::Solver::Options options;     
  // options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // CHOLESKY 분해 사용 
  // options.minimizer_progress_to_stdout = true;   // cout 출력

  // ceres::Solver::Summary summary;                // 최적화 정보
  // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  // ceres::Solve(options, &problem, &summary);  // 최적화 시작
  // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  // cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

  // // 출력결과
  // cout << summary.BriefReport() << endl;
  // cout << "estimated a,b,c = ";
  // for (auto a:abc) cout << a << " ";
  // cout << endl;

  return 0;
}