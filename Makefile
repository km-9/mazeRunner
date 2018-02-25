CXX ?= g++

CXXFLAGS += -c -Wall $(shell pkg-config --cflags opencv)
LDFLAGS += $(shell pkg-config --libs opencv)

all: opencv_runTest1

#opencv_runTest1: runTest1.o; $(CXX) $< PCA9685.cpp -L /home/pi/Downloads/lidar_sdk/sdk/output/Linux/Release -lrplidar_sdk -pthread -o $@ $(LDFLAGS)
opencv_runTest1: runTest1.o; $(CXX) $< PCA9685.cpp -L /home/pi/Downloads/lidar_sdk/sdk/output/Linux/Release -lrplidar_sdk -pthread -o $@ $(LDFLAGS)

%.o: %.cpp; $(CXX) $< -o $@ $(CXXFLAGS)

clean: ; rm -f runTest1.o opencv_runTest1
