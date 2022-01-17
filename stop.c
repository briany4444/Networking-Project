#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "simulator.h"

int main() {
      //our varibles
      int                 clientSocket, addrSize, bytesReceived;
      struct sockaddr_in  clientAddr;
      unsigned char buffer[1];

      // Register with the server
      clientSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
      if (clientSocket < 0) { //error check
        exit(-1);
      }

      memset(&clientAddr, 0, sizeof(clientAddr));
      clientAddr.sin_family = AF_INET;
      clientAddr.sin_addr.s_addr = inet_addr(SERVER_IP);
      clientAddr.sin_port = htons((unsigned short) SERVER_PORT);

      buffer[0] = STOP; //sends 1 byte to the server, we don't need to wait for a response
      sendto(clientSocket, buffer, 1, 0, (struct sockaddr*)&clientAddr, sizeof(clientAddr));
    	close(clientSocket);
  }
 
