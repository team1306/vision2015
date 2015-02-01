#include <pthread.h>
#include <mutex>

#include <string>
#include <cstring>
#include <iostream>

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

void *ServeRoboRIO(void *);
void *GrabImage(void *);

int main() {
  pthread_t server;
  pthread_t retrieveImage;
  int iret1, iret2;

  iret1 = pthread_create(&server, NULL, ServeRoboRIO, NULL);
  iret2 = pthread_create(&retrieveImage, NULL, GrabImage, NULL);

  lateral = 0.5;
  pthread_join(server, NULL);
  pthread_join(retrieveImage, NULL);

  return 0;
}

void *ServeRoboRIO(void *dummy) {
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
  return NULL;
}

void *GrabImage(void *) {
  VideoCapture vcap;
  //vcap.open("http://10.13.6.11/mjpg/video.mjpg");
  vcap.open(0);

  Mat tmp;
  while(1) {
    vcap.read(tmp);
    imageMtx.lock();
    tmp.copyTo(image);
    imageMtx.unlock();
  }
}
