#include "opencv2/opencv.hpp"
#include <iostream>
using namespace cv;
using namespace std;
int main(){
    string src = "nvarguscamerasrc sensor-id=0 ! \
    video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
    format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
    width=(int)640, height=(int)360, format=(string)BGRx ! \
    videoconvert ! video/x-raw, format=(string)BGR ! appsink"; 
    VideoCapture source(src, CAP_GSTREAMER); 
    if (!source.isOpened()){ cout << "Camera error "<< endl; return -1;}

    string dst1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
    h264parse ! rtph264pay pt=96 ! \
    udpsink host=192.168.0.28 port=8300 sync=false";
    VideoWriter writer1(dst1, 0, (double)30, Size(640, 360), true);
    if (!writer1.isOpened()) { cerr << "Writer open failed!" << endl; return -1;}
    
    string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
    h264parse ! rtph264pay pt=96 ! \
    udpsink host=192.168.0.28 port=8302 sync=false";
    VideoWriter writer2(dst2, 0, (double)30, Size(640, 360), false);
    if (!writer2.isOpened()) { cerr << "Writer open failed!" << endl; return -1;}

    string dst3 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
    h264parse ! rtph264pay pt=96 ! \
    udpsink host=192.168.0.28 port=8303 sync=false";
    VideoWriter writer3(dst3, 0, (double)30, Size(640, 360), false);
    if (!writer3.isOpened()) { cerr << "Writer open failed!" << endl; return -1;}


    Mat frame, gray, binary;
    while (true) {
        source >> frame;
        if (frame.empty()){ cerr << "frame empty!" << endl; break; }
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        threshold(gray, binary, 128, 255, THRESH_BINARY);

        writer1 << frame;
        writer2 << gray;
        writer3 << binary;
        waitKey(30);
    }
    return 0;
}
