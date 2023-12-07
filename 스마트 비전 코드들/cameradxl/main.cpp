#include <iostream>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include "dxl.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

bool ctrlCPressed = false;

void ctrlCHandler(int) {
    ctrlCPressed = true;
}

int main() {
    Dxl dynamixelController;
    struct timeval start, end1;
    double elapsedTime;
    int motorVel1 = 0, motorVel2 = 0;

    signal(SIGINT, ctrlCHandler);

    if (!dynamixelController.open()) {
        cout << "Dynamixel open error" << endl;
        return -1;
    }

    string src = "nvarguscamerasrc sensor-id=0 ! "
                 "video/x-raw(memory:NVMM), width=(int)640, height=(int)360, "
                 "format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, "
                 "width=(int)640, height=(int)360, format=(string)BGRx ! "
                 "videoconvert ! video/x-raw, format=(string)BGR ! appsink";

    VideoCapture source(src, CAP_GSTREAMER);
    if (!source.isOpened()) {
        cerr << "Camera error" << endl;
        dynamixelController.close();
        return -1;
    }

    string dst1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! "
                  "nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! "
                  "h264parse ! rtph264pay pt=96 ! "
                  "udpsink host=203.234.58.151 port=8300 sync=false";

    VideoWriter writer1(dst1, 0, (double)30, Size(640, 360), true);
    if (!writer1.isOpened()) {
        cerr << "Writer 1 open failed!" << endl;
        dynamixelController.close();
        return -1;
    }

    string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! "
                  "nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! "
                  "h264parse ! rtph264pay pt=96 ! "
                  "udpsink host=203.234.58.151 port=8302 sync=false";

    VideoWriter writer2(dst2, 0, (double)30, Size(640, 360), false);
    if (!writer2.isOpened()) {
        cerr << "Writer 2 open failed!" << endl;
        dynamixelController.close();
        return -1;
    }

    Mat frame, gray;

    while (true) {
        gettimeofday(&start, NULL);

        if (dynamixelController.kbhit()) {
            char userInput = dynamixelController.getch();

            switch (userInput) {
                case 's': motorVel1 = 0; motorVel2 = 0; break;
                case 'f': motorVel1 = 50; motorVel2 = -50; break;
                case 'b': motorVel1 = -50; motorVel2 = 50; break;
                case 'l': motorVel1 = -50; motorVel2 = -50; break;
                case 'r': motorVel1 = 50; motorVel2 = 50; break;
                default: motorVel1 = 0; motorVel2 = 0; break;
            }

            dynamixelController.setVelocity(motorVel1, motorVel2);
        }

        if (ctrlCPressed) {
            break;
        }

        source >> frame;
        if (frame.empty()) {
            cerr << "Frame empty!" << endl;
            break;
        }

        cvtColor(frame, gray, COLOR_BGR2GRAY);
        writer1 << frame;
        writer2 << gray;

        usleep(20 * 1000);

        gettimeofday(&end1, NULL);
        elapsedTime = end1.tv_sec - start.tv_sec + (end1.tv_usec - start.tv_usec) / 1000000.0;

        cout << "Motor Velocities: " << motorVel1 << ',' << motorVel2 << ", Elapsed Time: " << elapsedTime << " seconds" << endl;
    }

    dynamixelController.close();

    return 0;
}