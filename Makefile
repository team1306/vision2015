CC=g++

CFLAGS=-std=c++11

OPENCV_CFLAGS=-I/usr/local/include/opencv -I/usr/local/include/opencv2 -L/usr/local/lib/
OPENCV_LIBS=-lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_stitching

THREAD_LIBS=-pthread

all: main test threads

main: main.o
	$(CC) main.o -o main $(THREAD_LIBS) $(OPENCV_LIBS)

test: test.o
	$(CC) test.o -o test $(OPENCV_CFLAGS) $(OPENCV_LIBS)

threads: threads.o
	$(CC) threads.o -o threads $(OPENCV_CFLAGS) $(OPENCV_LIBS)

client: client.o
	$(CC) client.o -o client $(THREAD_LIBS)

main.o: main.cpp
	$(CC) -c main.cpp $(THREAD_CFLAGS) $(OPENCV_CFLAGS) $(CFLAGS)

test.o: test.cpp
	$(CC) -c test.cpp $(OPENCV_CFLAGS) $(CFLAGS)

threads.o: threads.cpp
	$(CC) -c threads.cpp $(OPENCV_CFLAGS) $(CFLAGS)

roboriosocket.o: roboriosocket.cpp
	$(CC) -c roboriosocket.cpp $(OPENCV_CFLAGS) $(CFLAGS)

client.o: client.cpp
	$(CC) -c client.cpp $(THREAD_CFLAGS) $(CFLAGS)

clean:
	rm ./*.o
	rm ./test
	rm ./main
	rm ./threads