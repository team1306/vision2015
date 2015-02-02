/**
 * @file main.cpp
 * @brief Spawn threads to grab and process images and serve requests from the RoboRIO
 */

#define PRINT
#define DISPLAY

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

/// Mutex lock for lateral
std::mutex lateralMtx;
/// Horizontal distance to target
double lateral;

/// Mutex lock for image
std::mutex imageMtx;
/// Imaged grabbed from the camera to be processed for targets
Mat image;

/// Mutex lock for freshImage
std::mutex freshImageMtx;
/// Signals to the processing thread whether it has a new image to read
bool freshImage = false;

/// Mutex lock for distanceToTarget
std::mutex distanceMtx;
/// Distance to target received from RoboRIO
double distanceToTarget;

/**
 * Compare the area bounded by two sets of points.
 *
 * @param a First vector of points
 * @param b Second vector of points
 * @return True if Area(a) is less than Area(b)
 */

bool compareArea(std::vector<Point>, std::vector<Point>);

/**
 * Find the center of the vision targets in a given image
 *
 * @param tmp The image to process for vision targets
 * @return The mean point of the vision targets
 */

Point2f findTarget(Mat);

/**
 * Serve TCP/IP socket connections from the RoboRIO to receive distances
 * from the range finder and return the horizontal distance from the vision
 * targets.
 */

void ServeRoboRIO();

/**
 * Retrieve image from the webcam and copy it to the global Mat as fast as
 * possible.
 */

void GrabImage();

/**
 * Process the image, extracting targets and totes from the image.
 */

void ProcessImage();

int main() {
  distanceToTarget = 20.0;

  std::thread (ServeRoboRIO).detach();
  std::thread (GrabImage).detach();
  std::thread (ProcessImage).detach();

  while(1) {}

  return 0;
}

bool compareArea(const std::vector<Point> a, const std::vector<Point> b) {
  return (contourArea(a) > contourArea(b));
}

Point2f findTarget(Mat tmp) {
  Mat edges, thresh;
  std::vector<std::vector<Point> > contours;
  std::vector<Vec4i> hierarchy;
  Mat colors [3];
  double lat = 0;
  Point2f mean (0, 0);

#ifdef PRINT
  std::chrono::high_resolution_clock::time_point begin, end;
  begin = std::chrono::high_resolution_clock::now();
#endif

  std::cout << (tmp.data ? "data" : "no data") << std::endl;
  if(tmp.data) {
    split(tmp, colors);

#ifdef PRINT
    end = std::chrono::high_resolution_clock::now();
    std::cout << "Split: " << (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()/1000 << " secs" << std::endl;
    begin = end;
#endif

    threshold(colors[1], edges, 180, 255, THRESH_BINARY);
    GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);

#ifdef DISPLAY
    imshow("blur", edges);
#endif
	
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

	Scalar color = Scalar(0,0,255);
	std::vector<std::vector<Point> > cons;
	cons.push_back(approx);
#ifdef DISPLAY
	drawContours( drawing, cons, 0, color, 2, 8, hierarchy, 0, Point() );
#endif
      }
      else {
	Scalar color = Scalar(0,255,0);
	std::vector<std::vector<Point> > cons;
	cons.push_back(approx);
#ifdef DISPLAY
	drawContours( drawing, cons, 0, color, 2, 8, hierarchy, 0, Point() );
#endif
      }
    }

#ifdef DISPLAY
    imshow("contours", drawing);
#endif

    std::cout << blobs.size() << std::endl;

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
      }

      mean = Point2f((centers[0].x + centers[1].x) / 2, (centers[0].y + centers[1].y) / 2);
    }
  }

#ifdef PRINT
  end = std::chrono::high_resolution_clock::now();
  std::cout << "Final: " << (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()/1000 << " secs" << std::endl << std::endl;
#endif

  return mean;
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
  double dist, lat;
  std::string message;

  while(1) {
    bzero(incoming, 100);
    s = recv(new_fd, incoming, 100, 0);
    if(s > 0) {
      dist = stod(std::string(incoming));
      distanceMtx.lock();
      distanceToTarget = dist;
      distanceMtx.unlock();

      lateralMtx.lock();
      lat = lateral;
      lateralMtx.unlock();
    
      message = std::to_string(lat);
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
    
    freshImageMtx.lock();
    freshImage = true;
    freshImageMtx.unlock();

#ifdef PRINT
    end = std::chrono::high_resolution_clock::now();
    std::cout << "Read: " << (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()/1000 << " secs" << std::endl;
#endif
  }
}

void ProcessImage() {
  Mat tmp;
  bool fresh = false;

  while(1) {
    freshImageMtx.lock();
    fresh = freshImage;
    freshImageMtx.unlock();

    if(fresh) {
      imageMtx.lock();
      image.copyTo(tmp);
      imageMtx.unlock();

      Point mean = findTarget(tmp);
      distanceMtx.lock();
      lateralMtx.lock();
      lateral = (mean.x - tmp.cols/2)*distanceToTarget;
      std::cout << lateral << std::endl;
      lateralMtx.unlock();
      distanceMtx.unlock();

#ifdef DISPLAY
      imshow("tmp", tmp);
      if(waitKey(30) >= 0) {break;}
#endif
      
      freshImageMtx.lock();
      freshImage = false;
      freshImageMtx.unlock();
    }
  }
}
