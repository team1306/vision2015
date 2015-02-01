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
  std::vector<std::vector<Point> > contours;
  std::vector<Vec4i> hierarchy;
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
      vcap.read(frame);

      if(frame.data) {
	split(frame, colors);
	imshow("colors", colors[1]);

	threshold(colors[1], edges, 180, 255, THRESH_BINARY);
	imshow("thresh", edges);
	GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);

	findContours(edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	
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
      sleep(0.05);
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
