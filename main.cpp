#include <pthread.h>
#include <mutex>

#include <string>
#include <cstring>
#include <iostream>

#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netdb.h>

std::mutex lateralMtx;
double lateral;

void *ServeRoboRIO(void *);
void *GrabIamge(void *);

int main() {
  pthread_t server;
  int iret1;

  iret1 = pthread_create(&server, NULL, ServeRoboRIO, NULL);

  lateral = 0.5;
  pthread_join(server, NULL);

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
  
}
