#include "opencv2/opencv.hpp" //"opencv2/opencv.hpp" 헤더 파일 포함
#include <iostream> //<iostream> 헤더 파일 포함
#include <unistd.h> //<unistd.h>헤더 파일 포함
#include <sys/time.h> //<sys/time.h> 헤더 파일 포함
#include <signal.h> //<signal.h> 헤더 파일 포함
#include "dxl.hpp" //"dxl.hpp" 헤더 파일 포함

using namespace cv; //OpenCV 라이브러리의 모든 기능에 대해 cv 네임스페이스를 사용하도록 선언
using namespace std; //C++ 표준 라이브러리의 모든 기능에 대해 std 네임스페이스를 사용하도록 선언
//lanefollow_100rpm_cw.mp4 
//lanefollow_100rpm_ccw.mp4

bool ctrlCPressed = false; //ctrlC 눌렀는지 여부 기본 false로 설정 
bool startMovement = false;  // 출발 여부 기본 false로 설정

void ctrlCHandler(int) { //ctrl 눌렀으면
    ctrlCPressed = true; //ctrlCPressed 가 true로 설정
} //종료

int main(void) {
    Dxl mx; //생성자임, 멤버변수 초기화작업 수행
    struct timeval start, end1; //시간 측정을 위한 변수 설정
    double time1; //double형 time1 변수

    signal(SIGINT, ctrlCHandler); //시그널 핸들러 지정
    if (!mx.open()) { //mx가 열리지 않으면
        cout << "Dynamixel open error" << endl; //문자열 출력
        return -1; //종료
    } //종료

    VideoCapture source("nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=(int)640, height=(int)360, "
        "format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, "
        "width=(int)640, height=(int)360, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! appsink",
        CAP_GSTREAMER);

    if (!source.isOpened()) {
        cerr << "Camera error" << endl;
        mx.close();
        return -1;
    }

    string dst1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
    h264parse ! rtph264pay pt=96 ! \
    udpsink host=203.234.58.163 port=8300 sync=false"; // gstreamer를 이용하여 네트워크로 전송하는 명령어 저장, host=203.234.58.163 port=8300
    VideoWriter writer1(dst1, 0, (double)30, Size(640, 360), true); // VideoWriter 생성자에 gstreamer 명령어 전달
    if (!writer1.isOpened()) { cerr << "Writer open failed!" << endl; return -1; } //writer1이 열리지 않으면 문자열 출력 후 종료

    string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
    h264parse ! rtph264pay pt=96 ! \
    udpsink host=203.234.58.163 port=8302 sync=false"; // gstreamer를 이용하여 네트워크로 전송하는 명령어 저장, host=203.234.58.163 port=8302
    VideoWriter writer2(dst2, 0, (double)30, Size(640, 90), true); // VideoWriter 생성자에 gstreamer 명령어 전달
    if (!writer2.isOpened()) { cerr << "Writer open failed!" << endl; return -1; } //writer2이 열리지 않으면 문자열 출력 후 종료

    Mat video, frame, gray, bin, dst, label, stat, centroid; //Mat객체 설정

    Mat left_area, left_dst, left_label, left_stat, left_centroid; //왼쪽 라인 검출할때 사용할 Mat객체 사용
    Mat right_area, right_dst, right_label, right_stat, right_centroid; //오른쪽 라인 검출할때 사용할 Mat객체 사용

    int a = 270; //int a = 270으로 설정
    double d, error = 0; //double형 d 변수 생성, error 생성 및 0으로 초기화
    int front = 200; //직진속도 100으로 설정
    double gain = 0.5; //gain값을 설정,100일 경우 0.4, 200일 경우 0.5
    double left_d; //왼쪽 d 변수 생성
    double right_d; //오른쪽 d 변수 생성

    Point left_center; //왼쪽 라인 검출 좌표 저장할 변수
    Point right_center; //오른쪽 라인 검출 좌표 저장할 변수
    Rect left_sq; //왼쪽 라인 바운딩 박스 저장할 변수
    Rect right_sq; //오른쪽 라인 바운딩 박스 저장할 변수

    d = 320; //d에 320 저장(영상 중앙)
    left_d = 160; //left_d에 160 저장 (영상 중앙)
    right_d = 160; //right_d에 160 저장 (영상 중앙)


    double left_prevLineCenter = left_d; //왼쪽 영상의 이전좌표(x값)에  left_d값 저장
    bool left_FirstFrame = true; //왼쪽 처음 영상을 설정하기 위한 bool변수, true로 설정
    bool left_save = false; //left_prevLineCenter = left_currentLineCenter을 할지 결정하는 bool 변수 left_save를 false로 설정

    double right_prevLineCenter = right_d;  //오른쪽 영상의 이전좌표(x값)에   right_d값 저장
    bool right_FirstFrame = true; //오른쪽 처음 영상을 설정하기 위한 bool변수, true로 설정
    bool right_save = false; //right_prevLineCenter = right_currentLineCenter을 할지 결정하는 bool 변수 right_save를 false로 설정
    while (1) { //무한반복
        gettimeofday(&start, NULL); // 시작시간 저장

        source >> frame;  //src를 frame에 저장
        if (frame.empty()) { //frame이 없으면
            cerr << "프레임 오류" << endl; //문자열 출력
            break; //종료
        } //종료
        video = frame.clone(); //video에 frame의 복사본을 저장
        frame = frame(Rect(0, a, frame.cols, frame.rows - a)); //frame 크기 설정(640, 360)
        int mid_cols = frame.cols / 2; //int형 변수 mid_cols에 frame.cols / 2 값 저장;
        cvtColor(frame, gray, COLOR_BGR2GRAY); //그레이변환
        gray = gray + (Scalar(100) - mean(gray)); //밝기 조정
        GaussianBlur(gray, gray, Size(5, 5), 0); //가우시안 블러
        threshold(gray, bin, 147, 255, THRESH_BINARY); //이진화
        morphologyEx(bin, bin, MORPH_CLOSE, Mat(), Point(-1, -1), 3); //모폴로지 연산 중 MORPH_CLOSE 사용
        //adaptiveThreshold(gray, bin, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 151, -52); //적응형 이진화
        //morphologyEx(bin, bin, MORPH_CLOSE, Mat(), Point(-1, -1), 4); //모폴로지 연산 중 MORPH_CLOSE 사용
        left_area = bin(Rect(0, 0, mid_cols, frame.rows)); //왼쪽 라인 검출할 화면설정
        int left_cnt = connectedComponentsWithStats(left_area, left_label, left_stat, left_centroid); //레이블링해서 라인 찾기
        cvtColor(left_area, left_dst, COLOR_GRAY2BGR); //그레이영상을 다시 BGR영상으로 변경
        double left_minDistance = 0; //왼쪽 최소거리 저장 변수
        int left_mincnt = 0; //왼쪽 mincnt를 설정
        double left_currentLineCenter = left_prevLineCenter; //왼쪽 구역의 현재 라인 중심의 x값에 이전 라인 x값 저장

        if (left_FirstFrame) { //왼쪽의 첫번째 프레임이면
            left_currentLineCenter = left_area.cols / 2; //현재 라인 중심의 x값은 160에 위치
            left_save = true; //left_save를 true로 변경
            left_FirstFrame = false; //left_isFirstFrame을 false로 변환
        }//종료

        for (int i = 1; i < left_cnt; i++) { //왼쪽 라인 검출 반복문
            int* l = left_stat.ptr<int>(i); //left_stat 행렬에서 i번째 행에 해당하는 포인터 l를 생성
            double* lc = left_centroid.ptr<double>(i); //left_centroid 행렬에서 i번째 행에 해당하는 포인터 lc를 생성
            if (l[4] < 200) continue; //left_stat 행렬에서, 특정 객체의 넓이(l[4])가 200 미만인 경우에는 무시하고 다음 객체로 넘어감

            left_mincnt++; //left_mincnt 증가
            double left_distanceToCurrentLine = abs(left_prevLineCenter - lc[0]); //현재 객체의 중심이 이전에 계산한 라인 중심과 얼마나 떨어져 있는지를 나타내는 값을 left_distanceToCurrentLine에 저장
            if (left_mincnt == 1) {  //left_mincnt가 1이면
                left_minDistance = left_distanceToCurrentLine; //left_minDistance에 left_distanceToCurrentLine값 저장
            }//종료
            if (left_minDistance >= left_distanceToCurrentLine) { //left_minDistance >= left_distanceToCurrentLine면
                left_minDistance = left_distanceToCurrentLine; //left_minDistance에 left_distanceToCurrentLine 값 저장
                left_currentLineCenter = lc[0]; //left_currentLineCenter에 이때의 x좌표 저장
            }//종료
            if (left_minDistance == left_distanceToCurrentLine && left_distanceToCurrentLine < 100) { //left_minDistance == left_distanceToCurrentLine && left_distanceToCurrentLine < 50이면
                rectangle(left_dst, Rect(l[0], l[1], l[2], l[3]), Scalar(0, 0, 255), 2); //빨간색 바운딩 박스 그리기
                circle(left_dst, Point(lc[0], lc[1]), 3, Scalar(0, 0, 255), -1); //빨간색 원그리기
                left_sq = Rect(l[0], l[1], l[2], l[3]); //left_sq에 바운딩 박스 저장하기
                left_center = Point(lc[0], lc[1]); //left_center에 무게중심좌표 저장
                left_save = true; //left_save를 true로 변경
            }//종료
            else { //그외에는 
                rectangle(left_dst, Rect(l[0], l[1], l[2], l[3]), Scalar(255, 0, 0), 2); //파란색 바운딩 박스 그리기
                circle(left_dst, Point(lc[0], lc[1]), 3, Scalar(255, 0, 0), -1); //파랑색 원 그리기 
            } //종료
        } //종료

        left_mincnt = 0; //left_mincnt를 0으로 초기화
        if (left_save) { //left_save가 true면
            left_prevLineCenter = left_currentLineCenter; //left_prevLineCenter에 left_currentLineCenter값 저장
            left_save = false; //left_save를 false로 변경
        }//종료

        right_area = bin(Rect(mid_cols, 0, mid_cols, frame.rows)); //오른쪽 라인 검출할 화면설정
        int right_cnt = connectedComponentsWithStats(right_area, right_label, right_stat, right_centroid); //레이블링해서 라인 찾기
        cvtColor(right_area, right_dst, COLOR_GRAY2BGR); //그레이영상을 다시 BGR영상으로 변경
        double right_minDistance = 0; //오른쪽 최소거리 저장 변수
        double right_mincnt = 0; //오른쪽 mincnt를 설정
        double right_currentLineCenter = right_prevLineCenter; //오른쪽 구역의 현재 라인 중심의 x값에 이전 라인 x값 저장

        if (right_FirstFrame) { //오른쪽의 첫번째 프레임이면
            right_currentLineCenter = right_area.cols / 2; //현재 라인 중심의 x값은 160에 위치
            right_save = true;  //right_save를 true로 변경
            right_FirstFrame = false; //right_isFirstFrame을 false로 변환
        }//종료
        for (int i = 1; i < right_cnt; i++) { //오른쪽 라인 검출 반복문
            int* r = right_stat.ptr<int>(i); //right_stat 행렬에서 i번째 행에 해당하는 포인터 r를 생성
            double* rc = right_centroid.ptr<double>(i); //right_centroid 행렬에서 i번째 행에 해당하는 포인터 lc를 생성

            if (r[4] < 200) continue; //right_stat 행렬에서, 특정 객체의 넓이(l[4])가 200 미만인 경우에는 무시하고 다음 객체로 넘어감
            right_mincnt++; //right_mincnt 증가
            double right_distanceToCurrentLine = abs(right_prevLineCenter - rc[0]); //현재 객체의 중심이 이전에 계산한 라인 중심과 얼마나 떨어져 있는지를 나타내는 값을 right_distanceToCurrentLine에 저장
            if (right_mincnt == 1) { //right_mincnt가 1이면
                right_minDistance = right_distanceToCurrentLine; //right_minDistance에 right_distanceToCurrentLine 값 저장
            }//종료
            if (right_minDistance >= right_distanceToCurrentLine) { //right_minDistance >= right_distanceToCurrentLine면
                right_minDistance = right_distanceToCurrentLine; //right_minDistance에 right_distanceToCurrentLine 값 저장
                right_currentLineCenter = rc[0]; //right_currentLineCenter에 이때의 x좌표 저장
            }//종료

            if (right_minDistance == right_distanceToCurrentLine && right_distanceToCurrentLine < 100) { //right_minDistance == right_distanceToCurrentLine && right_distanceToCurrentLine < 50이면
                rectangle(right_dst, Rect(r[0], r[1], r[2], r[3]), Scalar(0, 0, 255), 2); //빨간색 바운딩 박스 그리기
                circle(right_dst, Point(rc[0], rc[1]), 3, Scalar(0, 0, 255), -1); //빨간색 원그리기
                right_center = Point(rc[0] + 320, rc[1]); //left_center에 무게중심좌표 저장(오른쪽 라인 검출 영상이어서 x좌표에 320더 함)
                right_sq = Rect(r[0] + 320, r[1], r[2], r[3]); //left_sq에 바운딩 박스 저장하기(오른쪽 라인 검출 영상이어서 x좌표에 320더 함)
                right_save = true; //right_save를 true로 변경
            }//종료
            else {  //그외에는 
                rectangle(right_dst, Rect(r[0], r[1], r[2], r[3]), Scalar(255, 0, 0), 2); //파란색 바운딩 박스 그리기
                circle(right_dst, Point(rc[0], rc[1]), 3, Scalar(255, 0, 0), -1); //파랑색 원 그리기 
            } //종료
        } //종료

        right_mincnt = 0; //right_mincnt를 0으로 초기화
        if (right_save) { //right_save가 true면
            right_prevLineCenter = right_currentLineCenter; //right_prevLineCenter에 right_currentLineCenter값 저장
            right_save = false; //right_save를 false로 변경
        }//종료
        cvtColor(bin, dst, COLOR_GRAY2BGR); //그레이영상을 BGR영상으로 변경후 dst에 저장

        rectangle(dst, left_sq, Scalar(0, 0, 255), 2); //왼쪽라인 바운딩 빅스 그리기
        rectangle(dst, right_sq, Scalar(0, 0, 255), 2); //왼쪽라인 원 그리기
        circle(dst, left_center, 3, Scalar(0, 0, 255), -1); //오른쪽라인 바운딩 빅스 그리기
        circle(dst, right_center, 3, Scalar(0, 0, 255), -1); //오른쪽라인 원 그리기

        Point robot_center; //로봇의 위치를 저장할 Point 변수 설정
        robot_center = Point((right_center.x + left_center.x) / 2, (right_center.y + left_center.y) / 2); //로봇의 중앙좌표를 왼쪽라인과 오른쪽라인의 좌표의 중간값으로 설정
        circle(dst, robot_center, 6, Scalar(255, 0, 255), -1); //로봇의 중앙에 원그리기
        error = d - robot_center.x; //error값 계산

        int vel1 = 0, vel2 = 0; //좌측, 우측 속도제어 값을 저장할 변수선언 및 0으로 초기화
        vel1 = front - gain * error; //좌측 속도제어
        vel2 = -(front + gain * error); //우측 속도 제어

        if (mx.kbhit()) { //키보드가 눌렸으면
            char c = mx.getch(); //getch()함수로 키값을 입력받아 c에 저장
            switch (c) { //스위치 문
            case 's': startMovement = true; //s가 눌리면  startMovement = true로 변경
            }//종료
        }//종료
        if (ctrlCPressed) break;  // Ctrl+C 입력 시 탈출

        if (startMovement) { //startMovement = true면
            mx.setVelocity(vel1, vel2); //모터 제어
        }//종료

        writer1 << video; //writer1에 video영상을 저장
        writer2 << dst; //writer2에 dst영상을 저장

        usleep(20 * 1000); //프로그램의 실행을 중단시키는 함수, (20 * 1000)초 sleep

        gettimeofday(&end1, NULL); // 종료시간 저장
        time1 = (end1.tv_sec - start.tv_sec) + (end1.tv_usec - start.tv_usec) / 1000000.0; //실행시간 = 종료시간 - 시작시간

        cout << "err: " << error << ", lvel: " << vel1 << ", rvel: " << vel2 << ", time: " << time1 << endl; //결과 출력
    }//종료

    mx.close(); //다이내믹셀의 속도를 0으로 설정하고 장치를 닫아 줌
    return 0; //0을 외부로 반환
} //종료