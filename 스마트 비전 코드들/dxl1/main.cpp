#include <iostream>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include "dxl.hpp"
using namespace std;
bool ctrl_c_pressed = false;
void ctrlc(int){ ctrl_c_pressed = true; }
int main(void)
{
    Dxl mx;
    struct timeval start,end1;
    double time1;
    signal(SIGINT, ctrlc); //시그널 핸들러 지정
    if(!mx.open()) { cout << "dynamixel open error"<<endl; return -1; }
    while(true)
    {
        gettimeofday(&start,NULL); 
        mx.setVelocity(50, -50); //전진 속도명령 전송
        usleep(1000*1000); //3초 sleep
        mx.setVelocity(-50, -50); //좌회전 속도명령 전송
        usleep(1000*1000); //3초 sleep
        mx.setVelocity(-50, 50); //후진 속도명령 전송
        usleep(1000*1000); //3초 sleep
        mx.setVelocity(50, 50); //우회전 속도명령 전송
        usleep(1000*1000); //3초 sleep
        mx.setVelocity(0, 0); //정지 속도명령 전송
        usleep(1000*1000); //3초 sleep
        if (ctrl_c_pressed) break; //Ctrl+c 누르면 탈출
        gettimeofday(&end1,NULL);
        time1=end1.tv_sec-start.tv_sec + (end1.tv_usec-start.tv_usec)/1000000.0;
        cout << "time:" << time1 << endl; //실행시간 출력
    }
    mx.close(); //장치닫기
    return 0;
}
