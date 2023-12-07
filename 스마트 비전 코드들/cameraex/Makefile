CX = g++ 
CFLAGS = -g -Wall
CVFLAGS = `pkg-config opencv4 --cflags --libs` # 백쿼트(`명령어`)는 쉘에서 명령어를 실행한 결과를 의미
SRCS = main.cpp 
TARGET = camera
$(TARGET):$(SRCS)
	$(CX) -o $(TARGET) $(SRCS) $(CFLAGS) $(CVFLAGS)
.PHONY: all clean
all: $(TARGET)
clean:
	rm -f $(OBJS) $(TARGET) 