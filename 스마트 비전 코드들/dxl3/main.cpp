#include <iostream>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include "dxl.hpp"
using namespace std;
bool ctrl_c_pressed = false;
void ctrlc_handler(int){ ctrl_c_pressed = true; }
int main(void)
{
    Dxl mx;
    struct timeval start,end1;
    double time1;
    int vel1 = 0,vel2 = 0, error;
    signal(SIGINT, ctrlc_handler);
    if(!mx.open()) { cout << "dynamixel open error"<<endl; return -1; }
    while(true)
    {
        gettimeofday(&start,NULL); 
        cout << "Enter error:";
        cin >> error;
        vel1 = 100 - 1* error;
        vel2 = -(100 + 1* error);
        mx.setVelocity(vel1, vel2);
        if (ctrl_c_pressed) break; //Ctrl+c입력시 탈출
        usleep(20*1000); 
        gettimeofday(&end1,NULL); 
        time1 =end1.tv_sec-start.tv_sec +(end1.tv_usec-start.tv_usec)/1000000.0;
        cout <<"vel1:"<<vel1<<','<<"vel2:"<<vel2<<",time:"<<time1<< endl;
    } 
    mx.close(); // 장치닫기
    return 0;
}
