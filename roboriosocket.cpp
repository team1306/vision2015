#include "roboriosocket.h"
#include <iostream>

#define RSIZE TCP_BUFSIZE_READ

using namespace std;

RoboRIOSocket::RoboRIOSocket() {
  sock = -1;
  port = 5800;
  address = "10.13.6.2";
}

bool RoboRIOSocket::conn(std::string address, int port) {
  if(sock == -1) {
    sock = socket(AF_INET, SOCK_STREAM, 0);
    std::cout << "Socket created\n";
  }
  
  if(inet_addr(address.c_str()) == -1) {
    struct hostent *he;
    struct in_addr **addr_list;
    if((he = gethostbyname(address.c_str())) == NULL) {
      std::cout << "Failed to resolve hostnam\n"
