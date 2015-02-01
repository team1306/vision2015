#include <string>
#include <cstring>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netdb.h>

#define MYPORT "5800"  // the port users will be connecting to
#define BACKLOG 1     // how many pending connections queue will hold

int main(void)
{
  struct sockaddr_storage their_addr;
  socklen_t addr_size;
  struct addrinfo hints;
  struct addrinfo *res;
  int sockfd, new_fd;

  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_PASSIVE;

  getaddrinfo(NULL, MYPORT, &hints, &res);

  sockfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  bind(sockfd, res->ai_addr, res->ai_addrlen);
  listen(sockfd, BACKLOG);

  addr_size = sizeof(their_addr);
  while(1) {
    new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &addr_size);
    
    std::string message = "hello world";
    send(new_fd, message.c_str(), message.size(), 0);
    
    shutdown(new_fd, 2);
  }
  shutdown(sockfd, 2);

  return 0;
}
