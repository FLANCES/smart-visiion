CC = g++   
CCFLAGS = -g -Wall

DXLFLAGS = -I/usr/local/include/dynamixel_sdk
DXLFLAGS += -ldxl_x64_cpp             

TARGET = actuator
OBJS = main.o dxl.o
$(TARGET) :  $(OBJS)
	$(CC) $(CCFLAGS) -o $(TARGET) $(OBJS) $(DXLFLAGS)
main.o : main.cpp
	$(CC) $(CCFLAGS) -c main.cpp $(DXLFLAGS)
dxl.o : dxl.hpp dxl.cpp
	$(CC) $(CCFLAGS) -c dxl.cpp $(DXLFLAGS)

.PHONY: all clean 

all: $(TARGET)

clean:
	rm -rf $(TARGET) $(OBJS)
