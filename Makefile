#------------------------------------------------------------------------------

SOURCE=runTest1.cpp
PROGRAM=runTest1
INCLUDESPATH=/home/pi/Downloads/lidar_sdk/sdk/sdk/include
INCLUDES=-rplidar.h -rplidar_driver.h -rplidar_protocol.h -rptypes.h -rplidar_cmd.h
LIBRARIESPATH=/home/pi/Downloads/lidar_sdk/sdk/output/Linux/Release
LIBRARIES= lrplidar_sdk
CC=g++

#------------------------------------------------------------------------------
all: $(PROGRAM)

$(PROGRAM): $(SOURCE)

	$(CC) $(SOURCE) -L $(LIBRARIESPATH) -$(LIBRARIES) -pthread PCA9685.cpp -o$(PROGRAM)

clean:

	rm -f $(PROGRAM)
