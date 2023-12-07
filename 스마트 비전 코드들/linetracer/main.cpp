#include "opencv2/opencv.hpp" //"opencv2/opencv.hpp" 헤더 파일 포함
#include <iostream> //<iostream> 헤더 파일 포함
#include <unistd.h> //<unistd.h>헤더 파일 포함
#include <sys/time.h> //<sys/time.h> 헤더 파일 포함
#include <signal.h> //<signal.h> 헤더 파일 포함
#include "dxl.hpp" //"dxl.hpp" 헤더 파일 포함

using namespace cv; //OpenCV 라이브러리의 모든 기능에 대해 cv 네임스페이스를 사용하도록 선언
using namespace std; //C++ 표준 라이브러리의 모든 기능에 대해 std 네임스페이스를 사용하도록 선언

bool ctrlCPressed = false; //ctrlC 눌렀는지 여부 기본 false로 설정 
bool startMovement = false;  // 출발 여부 기본 false로 설정

void ctrlCHandler(int) { //ctrl 눌렀으면
    ctrlCPressed = true; //ctrlCPressed 가 true로 설정
} //종료

int main(void) { //메인함수 시작
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

    Mat frame, gray, bin, dst, label, stat, centroid; //Mat 객체 선언
    int a = 270; //int a를 270으로 설정
    int front = 100; //직진속도를 저장한 변수, 직진속도 50으로 설정
    int vel1 = 0, vel2 = 0, error = 0; //좌측, 우측 모터제어 변수 및 error값을 저장한 변수를 설정 및 0으로 초기화
    double gain = 0.15; //gain설정
    double d; //double형 d 선언
    Point prevcircle; //포인트 prevcircle생성 

    d = source.get(CAP_PROP_FRAME_WIDTH) / 2; //d에 영상의 x의 중앙값(320)을 저장

    double prevLineCenter = d; //이전 라인 x값을 저장할 변수 선언 및 d값을 저장
    bool isFirstFrame = true;  //처음 영상을 설정하기 위한 bool, true로 설정
    bool save = false; //prevLineCenter = currentLineCenter을 할지 결정하는 bool 변수 save를 false로 설정

    while (true) { //무한 반복
        gettimeofday(&start, NULL); // 시작시간 저장

        source >> frame; //src를 frame에 저장
        if (frame.empty()) { //frame이 없으면
            cerr << "프레임 오류" << endl; //문자열 출력
            break; //종료
        } //종료

        Mat video = frame.clone(); //video에 frame의 복사본을 저장
        frame = frame(Rect(0, a, frame.cols, frame.rows - a)); //frame 크기 설정(640, 360)
        cvtColor(frame, gray, COLOR_BGR2GRAY); //그레이 변환
        gray += (Scalar::all(100) - mean(gray)); //밝기 보정
        GaussianBlur(gray, gray, Size(5, 5), 0); //가우시안 블러
        threshold(gray, bin, 147, 255, THRESH_BINARY); //이진화
        morphologyEx(bin, bin, MORPH_CLOSE, Mat(), Point(-1, -1), 3); //모폴로지 연산 중 MORPH_CLOSE 사용

        int cnt = connectedComponentsWithStats(bin, label, stat, centroid); //레이블링해서 라인 찾기
        cvtColor(bin, dst, COLOR_GRAY2BGR); //그레이영상을 다시 BGR영상으로 변경
        double minDistance = 0; //최소거리 저장 변수
        double mincnt = 0; //mincnt를 설정
        double currentLineCenter = prevLineCenter; //현재 라인 중심의 x값에 이전 라인 x값 저장

        if (isFirstFrame) { //첫번째 프레임이면
            currentLineCenter = frame.cols / 2; //현재 라인 중심의 x값은 320에 위치
            save = true; //save를 true로 변경
            isFirstFrame = false; //isFirstFrame을 false로 변환
        }//종료

        for (int i = 1; i < cnt; i++) { //라인 검출 반복문
            int* b = stat.ptr<int>(i); //stat 행렬에서 i번째 행에 해당하는 포인터 b를 생성
            double* c = centroid.ptr<double>(i); //centroid 행렬에서 i번째 행에 해당하는 포인터 c를 생성

            if (b[4] < 200) continue; //stat 행렬에서, 특정 객체의 넓이(b[4])가 200 미만인 경우에는 무시하고 다음 객체로 넘어감
            mincnt++; //mincnt 증가
            double distanceToCurrentLine = abs(prevLineCenter - c[0]); //현재 객체의 중심이 이전에 계산한 라인 중심과 얼마나 떨어져 있는지를 나타내는 값을 distanceToCurrentLine에 저장
            if (mincnt == 1) { //mincnt가 1이면
                minDistance = distanceToCurrentLine; //minDistance에 distanceToCurrentLine값 저장
            }//종료
            if (minDistance >= distanceToCurrentLine) { //minDistance >= distanceToCurrentLine면
                minDistance = distanceToCurrentLine; //minDistance에 distanceToCurrentLine 값 저장
                currentLineCenter = c[0]; //currentLineCenter에 이때의 x좌표 저장 
            }//종료

            if (minDistance == distanceToCurrentLine && distanceToCurrentLine < 50) { //minDistance == distanceToCurrentLine && distanceToCurrentLine < 50이면
                rectangle(dst, Rect(b[0], b[1], b[2], b[3]), Scalar(0, 0, 255), 2); //빨간색 바운딩 박스 그리기
                circle(dst, Point(c[0], c[1]), 3, Scalar(0, 0, 255), -1); //빨간색 원그리기
                prevcircle = Point(c[0], c[1]); //prevcircle에 Point(c[0], c[1])좌표 저장
                save = true; //save를 true로 변경
                error = d - c[0]; //에러값 계산
            }
            else { //그외에는
                rectangle(dst, Rect(b[0], b[1], b[2], b[3]), Scalar(255, 0, 0), 2); //파란색 바운딩 박스 그리기
                circle(dst, Point(c[0], c[1]), 3, Scalar(255, 0, 0), -1); //파랑색 원 그리기 
            } //종료
        }//종료
        mincnt = 0; //mincnt를 0으로 초기화

        if (save) { //save가 true면
            prevLineCenter = currentLineCenter; // prevLineCenter에  currentLineCenter값 저장
            save = false; //save를 false로 변경
        }
        circle(dst, prevcircle, 3, Scalar(0, 0, 255), -1); //빨간색 원 그리기(라인이 사라져도 원 표시하여 위치 나타내려고)
        vel1 = front - gain * error; //좌측속도명령
        vel2 = -(front + gain * error); //우측속도명령


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