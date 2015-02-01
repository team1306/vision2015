#define PRINT

#include <thread>
#include <mutex>

#include <string>
#include <cstring>
#include <iostream>
#include <vector>

#ifdef PRINT
#include <chrono>
#endif

#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netdb.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;

std::mutex lateralMtx;
double lateral;

std::mutex imageMtx;
Mat image;

bool compareArea(std::vector<Point>, std::vector<Point>);
void ServeRoboRIO();
void GrabImage();
void ProcessImage();

int main() {
  lateral = 0.5;

  std::thread (ServeRoboRIO).detach();
  std::thread (GrabImage).detach();
  std::thread (ProcessImage).detach();

  while(1) {}

  return 0;
}

bool compareArea(const std::vector<Point> a, const std::vector<Point> b) {
  return (contourArea(a) > contourArea(b));
}

void ServeRoboRIO() {
  struct sockaddr_in servaddr;
  int sockfd, new_fd;

  sockfd = socket(AF_INET, SOCK_STREAM, 0);

  bzero(&servaddr, sizeof(servaddr));
  
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = htons(INADDR_ANY);
  servaddr.sin_port = htons(5800);
  
  bind(sockfd, (struct sockaddr *) &servaddr, sizeof(servaddr));
  listen(sockfd, 10);

  new_fd = accept(sockfd, (struct sockaddr*) NULL, NULL);
  char incoming [100];
  int s = 0;
  while(1) {
    bzero(incoming, 100);
    s = recv(new_fd, incoming, 100, 0);
    if(s > 0) {
      double lat;
      lateralMtx.lock();
      lat = lateral;
      lateralMtx.unlock();
    
      std::string message = std::to_string(lat*std::stod(incoming));
      send(new_fd, message.c_str(), message.size(), 0);
    }
  }

  shutdown(new_fd, 2);
  shutdown(sockfd, 2);
}

void GrabImage() {
  VideoCapture vcap;
  //vcap.open("http://10.13.6.11/mjpg/video.mjpg");
  vcap.open(0);

  Mat tmp;

  #ifdef PRINT
  std::chrono::high_resolution_clock::time_point begin, end;
  #endif

  while(1) {
    #ifdef PRINT
    begin = std::chrono::high_resolution_clock::now();
    #endif

    vcap.read(tmp);
    imageMtx.lock();
    tmp.copyTo(image);
    imageMtx.unlock();

    #ifdef PRINT
    end = std::chrono::high_resolution_clock::now();
    std::cout << "Read: " << (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()/1000 << " secs" << std::endl;
    #endif
  }
}

void ProcessImage() {
  Mat edges, thresh;
  std::vector<std::vector<Point> > contours;
  std::vector<Vec4i> hierarchy;
  Mat colors [3];

  #ifdef PRINT
  std::chrono::high_resolution_clock::time_point begin, end;
  #endif

  while(1) {
    #ifdef PRINT
    begin = std::chrono::high_resolution_clock::now();
    #endif

    Mat frame;
    imageMtx.lock();
    image.copyTo(frame);
    imageMtx.unlock();

    if(frame.data) {
      split(frame, colors);
      imshow("colors", colors[1]);

      #ifdef PRINT
      end = std::chrono::high_resolution_clock::now();
      std::cout << "Split: " << (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()/1000 << " secs" << std::endl;
      begin = end;
      #endif

      threshold(colors[1], edges, 180, 255, THRESH_BINARY);
      imshow("thresh", edges);
      GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
	
      #ifdef PRINT
      end = std::chrono::high_resolution_clock::now();
      std::cout << "Preprocessing: " << (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()/1000 << " secs" << std::endl;
      begin = end;
      #endif

      findContours(edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

      #ifdef PRINT
      end = std::chrono::high_resolution_clock::now();
      std::cout << "Found contours: " << (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()/1000 << " secs" << std::endl;
      begin = end;
      #endif
	
      std::vector<std::vector<Point> > targets;
      std::vector<std::vector<Point> > blobs;
      Mat drawing = Mat::zeros( edges.size(), CV_8UC3 );
      for( int i = 0; i<contours.size(); i++ ) {
	if(contourArea(contours[i]) < 200) continue;
	std::vector<Point> approx;
	approxPolyDP(contours[i], approx, arcLength(Mat(contours[i]), true) * 0.01, true);
	  
	int vtc = approx.size();
	if(vtc == 6 /*&& mincos >= 1.5 && maxcos <= 1.7*/) {
	  blobs.push_back(approx);
	  RotatedRect r = minAreaRect(approx);
	  Point2f verts [4];
	  r.points(verts);
	  for(int y=0; y<4; y++) {
	    line(drawing, verts[y], verts[(y+1)%4], Scalar(255,0,0));
	  }
	  Scalar color = Scalar(0,0,255);
	  std::vector<std::vector<Point> > cons;
	  cons.push_back(approx);	
	  drawContours( drawing, cons, 0, color, 2, 8, hierarchy, 0, Point() );
	}
	else {
	  Scalar color = Scalar(0,255,0);
	  std::vector<std::vector<Point> > cons;
	  cons.push_back(approx);
	  drawContours( drawing, cons, 0, color, 2, 8, hierarchy, 0, Point() );	      
	}
      }

      if(blobs.size() > 1) {
	std::cout << "sorting" << std::endl;
	std::sort(blobs.begin(), blobs.end(), compareArea);
	targets.push_back(blobs[0]);
	targets.push_back(blobs[1]);

	std::vector<Moments> mu (targets.size());
	for(int i=0; i<targets.size(); i++) {
	  mu[i] = moments(targets[i], false);
	}

	std::vector<Point2f> centers (targets.size());
	for(int i=0; i<targets.size(); i++) {
	  centers[i] = Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);
	  circle(drawing, centers[i], 4, Scalar(0, 0, 255), -1, 8, 0);
	}

	Point2f mean ((centers[0].x + centers[1].x) / 2, (centers[0].y + centers[1].y) / 2);

	circle(drawing, mean, 4, Scalar(255, 0, 255), -1, 8, 0);
      }

      imshow("edges", drawing);
      if(waitKey(30) >= 0) break;
    }

    #ifdef PRINT
    end = std::chrono::high_resolution_clock::now();
    std::cout << "Final: " << (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()/1000 << " secs" << std::endl << std::endl;
    #endif
  }
}
