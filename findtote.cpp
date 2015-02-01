#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <math.h>
#include <ctime>

using namespace cv;

double angle(const Point& v1, const Point& v2, const Point& v3) {
  Point p1 = Point(v1.x - v3.x, v1.y - v3.y);
  Point p2 = Point(v2.x - v3.x, v2.y - v3.y);
  
  float len1 = std::sqrt(p1.x * p1.x + p1.y * p1.y);
  float len2 = std::sqrt(p2.x * p2.x + p2.y * p2.y);

  float dot = p1.x * p2.x + p1.y * p2.y;

  float a = dot / (len1 * len2);

  if(a >= 1.0) {
    return 0.0;
  }
  else if(a <= -1.0) {
    return M_PI;
  }
  else {
    return std::acos(a);
  }
}

bool compareArea(const std::vector<Point> a, const std::vector<Point> b) {
  return (contourArea(a) > contourArea(b));
}

int main(int, char**)
{
  Mat edges, thresh;
  Mat colors [3];
  VideoCapture vcap;
  std::cout << "Connecting..." << std::endl;
  vcap.open("http://10.13.6.11/mjpg/video.mjpg");
  //vcap.open(0);
  std::cout << "Connected" << std::endl << std::endl;
  namedWindow("edges",1);
  for(;;)
    {
      Mat frame;
      Mat drawing;
      vcap.read(frame);

      if(frame.data) {
	drawing = frame.clone();

	cvtColor(frame, edges, CV_BGR2GRAY);
	threshold(edges, edges, mean(edges).val[0]-10, 255, THRESH_BINARY_INV);
	imshow("colors", edges);
	GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
	Canny(edges, edges, 30, 100, 3);
	imshow("edges", edges);

	std::vector<std::vector<Point> > contours;
	std::vector<Vec4i> hierarchy;
	findContours(edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	
	for(int i=0; i<contours.size(); i++) {
	  drawContours(drawing, contours, i, Scalar(0, 0, 255), 2, 8, hierarchy, 0, Point());
	}

	imshow("drawing", drawing);
	if(waitKey(30) >= 0) break;
      }
      sleep(0.05);
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
