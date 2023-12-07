#include <iostream>
#include <unistd.h>
#include <signal.h>
#include "dxl.hpp"
using namespace std;
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}
int main(void)
{
    Dxl mx;
    struct timeval start,end1;
    double diff1;
    int vel1=0,vel2=0;
    int goal1=0,goal2=0;

    signal(SIGINT, ctrlc); 				//시그널 핸들러 지정
    if(!mx.open()) { cout << "dxl open error"<<endl; return -1; } //장치열기
    
    while(true)
    {
        gettimeofday(&start,NULL);
        if (mx.kbhit())
        {
            char c = mx.getch();
            switch(c)
            {
            case 's': goal1 = 0; goal2 = 0; break;
            case ' ': goal1 = 0; goal2 = 0; break;
            case 'f': goal1 = 50; goal2 = -50; break;
            case 'b': goal1 = -50; goal2 = 50; break;
            case 'l': goal1 = -50; goal2 = -50; break;
            case 'r': goal1 = 50; goal2 = 50; break;
            default : goal1 = 0; goal2 = 0; break;
            }         
        }
        
        // generate accel and decel motion
        if(goal1>vel1) vel1 += 5;
        else if(goal1<vel1) vel1 -= 5;
        else vel1 = goal1;

        if(goal2>vel2) vel2 += 5;
        else if(goal2<vel2) vel2 -= 5;
        else vel2 = goal2;

        if(!mx.setVelocity(vel1,vel2)){ cout << "setVelocity error"<<endl; return -1;}
                
        if (ctrl_c_pressed) break;         
        usleep(50*1000);        
        gettimeofday(&end1,NULL);
        diff1 = end1.tv_sec + end1.tv_usec / 1000000.0 - start.tv_sec - start.tv_usec / 1000000.0;
        cout <<"vel1:" << vel1 <<','<<"vel2:"<< vel2 << ",time:" << diff1 << endl;
    }   
    
    mx.close();
    return 0;
}

