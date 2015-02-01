CC=g++
CFLAGS=-I/usr/local/include/opencv -I/usr/local/include/opencv2 -L/usr/local/lib/ -I/usr/include/linux
LIBS=-lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_stitching

all: test threads

test: test.o
	$(CC) test.o -o test $(CFLAGS) $(LIBS)

threads: threads.o
	$(CC) threads.o -o threads $(CFLAGS) $(LIBS)

test.o: test.cpp
	$(CC) -c test.cpp $(CFLAGS)

threads.o: threads.cpp
	$(CC) -c threads.cpp $(CFLAGS)

roboriosocket.o: roboriosocket.cpp
	$(CC) -c roboriosocket.cpp $(CFLAGS)