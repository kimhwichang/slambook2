#include <iostream>
#include <chrono>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {
  //argv[1]에 저장된 이미 읽기
  cv::Mat image;
  // image = cv::imread(argv[1]); //cv::imread
  image = cv::imread("./../distorted.png"); //cv::imread
  //제대로된 이미지 파일인지 확인
  if (image.data == nullptr) { //해당 이미지가 존재하지 않으면 출력
    cerr << "文件" << argv[1] << "不存在." << endl;
    return 0;
  }

  // image size -> 너비 (col) , 높이 (row) , 채널 (ch) -> 일반적으로 col x row 로 표기
  cout << "图像宽为" << image.cols << ",高为" << image.rows << ",通道数为" << image.channels() << endl;
  cv::imshow("image", image);      // cv::imshow
  cv::waitKey(0);                  // 프로그램 일시중단 & 종료버튼 입력 기다림

  // image 유형 판단
  if (image.type() != CV_8UC1 && image.type() != CV_8UC3) {
    // 컬러 or gray scale
    cout << "请输入一张彩色图或灰度图." << endl;
    return 0;
  }

  // 전체 픽셀 돌면서 개별 픽셀 접근
  // std::chrono 동작 시간 체크
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  for (size_t y = 0; y < image.rows; y++) {
    // cv::Mat::ptr 사용하여 개별 행의 pointer 지정 -> Mat::ptr() 함수는 Mat 행렬에서 특정 행의 1번 째 원소 주소를 반환한다.
    unsigned char *row_ptr = image.ptr<unsigned char>(y);
    // 지정된 행 별로 col방향으로 훝으면서 각 col마다 제일 앞의 값을 가리키는 행 pointer 설정 -> 이제 COL은 채널이 된다.
    for (size_t x =0; x <image.cols; x++){
      unsigned char *data_ptr= &row_ptr[x*image.channels()];
      for (size_t c=0; c !=image.channels();c++){
          unsigned char data = data_ptr[c];
      }
    }  //https://blog.naver.com/sees111/222346446876 -> MAT 관련 참조
    
  }
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast < chrono::duration < double >> (t2 - t1);
  cout << "Total time：" << time_used.count() << " 초 " << endl;

  // cv::Mat 복사
  
  cv::Mat image_another = image;
  // image_another 바꾸면 image도 바뀜
  image_another(cv::Rect(0, 0, 100, 100)).setTo(0); 
  cv::imshow("image", image);
  cv::waitKey(0);

  // clone() 을 이용한 복사
  cv::Mat image_clone = image.clone();
  image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
  cv::imshow("image", image);
  cv::imshow("image_clone", image_clone);
  cv::waitKey(0);
  cv::destroyAllWindows();
  return 0;
}
