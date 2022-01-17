#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "simulator.h"
//our definitions
#define ROBOT_ID 7
#define MOST_SIG_X 1
#define X 2
#define MOST_SIG_Y 3
#define Y 4
#define RESPONSE 0
#define COMMAND 0
#define NEGATIVE 5
#define DIRECTION 6
#define BUFFER_SIZE 10
#define DECIMAL_X 8
#define DECIMAL_Y 9

//the buffer
unsigned char buffer[BUFFER_SIZE];
void turnAround(int clockwise);
void updateLocation();
float extract(unsigned char mostSig, unsigned char lowerSig, unsigned char decimal);

#define FIVE_TWELVE 3
#define TWO_FIFTY_SIX 2
#define UNSIGNED_CHAR_SIZE 1

// This is the main function that simulates the "life" of the robot
// The code will exit whenever the robot fails to communicate with the server
int main() {
  int                 clientSocket, addrSize, bytesReceived;
  struct sockaddr_in  clientAddr;
  int online = 1;

  // Set up the random seed
  srand(time(NULL));

  // Send register command to server.  Get back response data
  // and store it.   If denied registration, then quit.
  clientSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (clientSocket < 0) { //error check
    exit(-1);
  }

  // Setup address
  memset(&clientAddr, 0, sizeof(clientAddr));
  clientAddr.sin_family = AF_INET;
  clientAddr.sin_addr.s_addr = inet_addr(SERVER_IP);
  clientAddr.sin_port = htons((unsigned short) SERVER_PORT);

  //registers the robot using 1
	buffer[COMMAND] = REGISTER;      //sends 1 byte to the server
  sendto(clientSocket, buffer, 1, 0, (struct sockaddr*)&clientAddr, sizeof(clientAddr));
  //waits for a response after registration
  addrSize = sizeof(clientAddr);
  bytesReceived = recvfrom(clientSocket, buffer, BUFFER_SIZE, 0,(struct sockaddr *) &clientAddr, &addrSize);
  //if the response was successful
  if(bytesReceived > 0){
    // if ok, then go into an infinite loop exhibiting the robot behavior
    if(buffer[RESPONSE] == OK){
      while(online){
        // Check if can move forward
        buffer[COMMAND] = CHECK_COLLISION;
        sendto(clientSocket,buffer , BUFFER_SIZE, 0, (struct sockaddr*)&clientAddr, sizeof(clientAddr));
        //receives 1 byte from the user
        addrSize = sizeof(clientAddr);    //receives 1 byte
        int bytesReceive = recvfrom(clientSocket, buffer, 1, 0,(struct sockaddr *) &clientAddr, &addrSize);
        // if something was sent from the server
        if(bytesReceive > 0){
          //if its ok (aka no collisions), then update the location
          if(buffer[RESPONSE] == OK){
            updateLocation();
          }
          //if it loses contact, break
          else if(buffer[RESPONSE] == LOST_CONTACT){
            break;
          }
          //if it would collide with another robot or wall, then turn around
          else if(buffer[RESPONSE] == NOT_OK_BOUNDARY || buffer[RESPONSE] == NOT_OK_COLLIDE){
            int cantMove = 1;
            //turns a random direction and continues to turn the SAME direction until it is able to move
            int clockwise = rand()%2;
            while(cantMove){
              turnAround(clockwise);
              //every time it turns even a little bit, it sends a status update to the server
              buffer[COMMAND] = STATUS_UPDATE;
              sendto(clientSocket, buffer, BUFFER_SIZE, 0, (struct sockaddr*)&clientAddr, sizeof(clientAddr));
              //checks to see if we can move given our new direction
              buffer[COMMAND] = CHECK_COLLISION;
              sendto(clientSocket, buffer, BUFFER_SIZE, 0, (struct sockaddr*)&clientAddr, sizeof(clientAddr));
              addrSize = sizeof(clientAddr);
              //checks the response
              bytesReceived = recvfrom(clientSocket, buffer,1, 0,(struct sockaddr *) &clientAddr, &addrSize);
              if(bytesReceive > 0){
                if(buffer[RESPONSE] == OK){
                  cantMove = 0;
                }
                //every time we receive lost contact, we break
                else if(buffer[RESPONSE] == LOST_CONTACT){
                  break;
                }
              }
            }
          }
          //once either the direction or location is updated, sends a status update to server
          buffer[COMMAND] = STATUS_UPDATE;
          sendto(clientSocket,buffer , BUFFER_SIZE, 0, (struct sockaddr*)&clientAddr, sizeof(clientAddr));
        }
        usleep(1000);
      }
    }
    //when there are 20 robots
    else if(buffer[RESPONSE] == NOT_OK){
      printf("Robot could not be added, max amount of robots\n");
    }
  }
  close(clientSocket);
}

//compresses both x and y into 2 bytes by taking in the registration Data and x, y
void Compress(float coordinate, int mostSigIndex, int lowerIndex, int decimal){
	//compresses either x or  y into its most significant byte and lower bits
	float direction = coordinate;
	if(coordinate >= 512){
		buffer[mostSigIndex] = FIVE_TWELVE;
		direction -= 512;
	}
	else if (coordinate >= 256){
		buffer[mostSigIndex] = TWO_FIFTY_SIX;
		direction -= 256;
	}
	else{
		buffer[mostSigIndex] = UNSIGNED_CHAR_SIZE;
	}
	buffer[lowerIndex] = direction;  //adds the lower bits
  buffer[decimal] = (float)(direction - buffer[lowerIndex]) * 10; //for the decimal
}

//extracts the bits and turns it into an int so calculations can be done
float extract(unsigned char mostSig, unsigned char lowerSig, unsigned char decimal){
	float total = 0;
  if(mostSig == FIVE_TWELVE){
    total += 512.0;
  }
  else if(mostSig == TWO_FIFTY_SIX){
   total += 256.0;
  }
  total += lowerSig;
  //adds the first decimal place
  total += ((float)decimal) / 10;
  return total;
}

//turns the robot around until it can move again
void turnAround(int counterClockwise){
 	if(counterClockwise == 1){
    if(buffer[NEGATIVE] == 0){
		    int newDirection = buffer[DIRECTION] + ROBOT_TURN_ANGLE;
		    if(newDirection> 180){
          float difference = newDirection - 180;
		      newDirection = 180 - difference;
          buffer[NEGATIVE]= 1;
        }
        buffer[DIRECTION] = newDirection;
		}
    else{
      int newDirection = buffer[DIRECTION] - ROBOT_TURN_ANGLE;
      if(newDirection < 0){
            newDirection = newDirection * -1;
            buffer[NEGATIVE] = 0;
       }
       buffer[DIRECTION] = newDirection;
	    }
    }
	//for clockwise when its positive
	else{
    if(buffer[NEGATIVE] == 0){
        int newDirection = buffer[DIRECTION] - ROBOT_TURN_ANGLE;
        if(newDirection < 0){
          newDirection = newDirection * -1;
          buffer[NEGATIVE] = 1;
        }
        buffer[DIRECTION] = newDirection;
    }
    //for clockwise to when its negative
    else{
      int newDirection = buffer[DIRECTION] + ROBOT_TURN_ANGLE;
      if(newDirection > 180){
        float difference = newDirection - 180;
        newDirection = 180 - difference;
        buffer[NEGATIVE]= 0;
     }
     buffer[DIRECTION] = newDirection;
    }
	}
}
//updates the new x and y location
void updateLocation(){
  //extracts the x and y from the buffer
  float x = extract(buffer[MOST_SIG_X], buffer[X], buffer[DECIMAL_X]);
  float y = extract(buffer[MOST_SIG_Y], buffer[Y], buffer[DECIMAL_Y]);
  //converts degrees to radians
  float inRadians = (buffer[DIRECTION] * PI) / 180;
  if(buffer[NEGATIVE] == 1){
  	inRadians *= -1;
  }
  //calculates the new position after radian conversion
  float newX = x + ROBOT_SPEED*cos(inRadians);
  float newY = y + ROBOT_SPEED*sin(inRadians);
  //compresses both x an y back into the buffer
  Compress(newX, MOST_SIG_X, X, DECIMAL_X);
  Compress(newY, MOST_SIG_Y, Y, DECIMAL_Y);
}
