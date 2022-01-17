#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "simulator.h"

Environment   environment;  // The environment that contains all the robots
void updateLocation();
float extract(unsigned char mostSig, unsigned char lowBits, unsigned char decimal);
void Compress(float coordinate, int mostSigIndex, int lowerIndex, int decimal);
void registerRobot();
void checkCollision();
int collidesWithRobot(float x1, float y1);
void startingCoordinates(Robot *robot);

//defines the registration array data
#define RESPONSE 0
#define ROBOT_ID 7
#define MOST_SIG_X 1
#define X 2
#define MOST_SIG_Y 3
#define Y 4
#define COMMAND 0
#define NEGATIVE 5
#define DIRECTION 6
#define BUFFER_SIZE 10
#define DECIMAL_X 8
#define DECIMAL_Y 9

#define FIVE_TWELVE 3
#define TWO_FIFTY_SIX 2
#define UNSIGNED_CHAR_SIZE 1
unsigned char buffer[BUFFER_SIZE];	//buffer for sending and receiveing messages
unsigned char sendCommand[1];		//for sending back responses to the client

// Handle client requests coming in through the server socket.  This code should run
// indefinitiely.  It should repeatedly grab an incoming messages and process them.
// The requests that may be handled are STOP, REGISTER, CHECK_COLLISION and STATUS_UPDATE.
// Upon receiving a STOP request, the server should get ready to shut down but must
// first wait until all robot clients have been informed of the shutdown.   Then it
// should exit gracefully.
void *handleIncomingRequests(void *e) {
		//variables for the server
		char   							online = 1;
		int                 serverSocket;
  	struct sockaddr_in  serverAddr, clientAddr;
  	int                 status, addrSize, bytesReceived;
  	fd_set              readfds, writefds;
		int numShutDown = 0;

  	// Initialize the server
		serverSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if(serverSocket < 0){    //if server can't be initialized
		 	exit(-1);
		}
		//sets the memory, server address etc.
		memset(&serverAddr, 0, sizeof(serverAddr));
	 	serverAddr.sin_family = AF_INET;
	 	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	 	serverAddr.sin_port = htons((unsigned short) SERVER_PORT);

		//binds the sockets
	 	status = bind(serverSocket, (struct sockaddr*) &serverAddr, sizeof(serverAddr));
	 	if(status < 0){
	 		exit(-1);
	 	}
  	// Wait for clients now as long as its online
		while (online) {
				//variables for the server
			FD_ZERO(&readfds);
   		FD_SET(serverSocket, &readfds);
    	FD_ZERO(&writefds);
    	FD_SET(serverSocket, &writefds);
			//selects a request
    	status = select(FD_SETSIZE, &readfds, &writefds, NULL, NULL);
	 		if (status < 0) {
	 			exit(-1);
	 		}
	 		else{
				//receives info back form the client
				addrSize = sizeof(clientAddr);
		 		bytesReceived = recvfrom(serverSocket, buffer, BUFFER_SIZE, 0, (struct sockaddr *) &clientAddr, &addrSize);
		 		//checks to make sure if at least 1 byte was received
		 		if (bytesReceived > 0) {
					if(environment.shutDown){
						numShutDown++;
						sendCommand[RESPONSE]= LOST_CONTACT;
						sendto(serverSocket, sendCommand, 1, 0, (struct sockaddr *) &clientAddr,sizeof(clientAddr));
					}
					if(environment.numRobots == numShutDown && environment.shutDown == 1){
						break;
					}
		 			//if we want to register a robot
		 			if (buffer[COMMAND] == REGISTER){
		 				if(environment.numRobots < MAX_ROBOTS){
							//sends all this info to the client
							registerRobot();
							buffer[RESPONSE] = OK;		//sends the buffer as we're sending more than 1 byte
		 					sendto(serverSocket, buffer, BUFFER_SIZE, 0, (struct sockaddr *) &clientAddr, sizeof(clientAddr));
							}
		 				//means there are too many robots so sends back NOT OK
		 				else{
		 					sendCommand[RESPONSE] = NOT_OK;		//sends back 1 byte
		 					sendto(serverSocket, sendCommand, 1, 0, (struct sockaddr *) &clientAddr, sizeof(clientAddr));
		 				}
			 		}
					//if the robot requests to moves
			 		else if(buffer[COMMAND] == CHECK_COLLISION){
							//if it can move, then move it
							checkCollision();		//the command is updated in checkcollision
							sendto(serverSocket, sendCommand, 1, 0, (struct sockaddr *) &clientAddr,sizeof(clientAddr));
		 			}
					//this is status response and updates the robot's coordinates and direction accordingly
					else if(buffer[COMMAND] == STATUS_UPDATE){
							updateLocation();	 //doesn't send back a response to client
					}
		 			// Quit if someone sent a STOP command
   				else if (buffer[COMMAND]== STOP){
 						environment.shutDown = 1;
  			}
			}
  	}
	}
		//closes the server socket
  close(serverSocket);
	pthread_exit(NULL);
}

//registers the robot using random values
void registerRobot(){
	Robot robot;
	//first updates the buffer{ROBOT_ID} since we need it to be updated before calling starting coordinates
	buffer[ROBOT_ID] = environment.numRobots;
	startingCoordinates(&robot);		//finds coordinates where another robot wont immediatemly move into
	//sets the direction between -180 to 180  (181 is excluded)
	robot.direction = rand()%181;
	//adds the new robot to the list of robots
	environment.robots[environment.numRobots] = robot;
	//sends back the info regarding the robot
	Compress(robot.x, MOST_SIG_X, X, DECIMAL_X);	//registers the x and y values through compression
	Compress(robot.y, MOST_SIG_Y, Y, DECIMAL_Y);	//registers the x and y values through compression
	buffer[NEGATIVE] = rand()%2;	//either 1 or 0, 0 is a positive value
	buffer[DIRECTION] = robot.direction;	//only the magnitude of the direction
	environment.numRobots++;
	//in case we have a negative value
	if(buffer[NEGATIVE]){
		robot.direction *= -1;
	}
}

//compresses both x and y into 2 bytes by taking in the registration Data and x, y
void Compress(float coordinate, int mostSigIndex, int lowerIndex, int decimal){
	//compresses either x or  y into its most significant byte and lower bits
	float dimension = coordinate;
	if(dimension >= 512){
		buffer[mostSigIndex] = FIVE_TWELVE;
		dimension -= 512;
	}
	else if (dimension >= 256){
		buffer[mostSigIndex] = TWO_FIFTY_SIX;
		dimension -= 256;
	}
	else{
		buffer[mostSigIndex] = UNSIGNED_CHAR_SIZE;
	}
	buffer[lowerIndex] = dimension;
	buffer[decimal] = (dimension - buffer[lowerIndex]) * 10;
}

//extracts the bytes and totals them up
float extract(unsigned char mostSig, unsigned char lowBits, unsigned char decimal){
	float total = 0;
  if(mostSig == FIVE_TWELVE){
    total += 512.0;
  }
  else if( mostSig == TWO_FIFTY_SIX){
    total += 256.0;
  }
  total += lowBits;
  total += ((float)decimal) / 10;
  return total;
}

//function that moves a particular robot
void updateLocation(){

	//extracts the x and y from the buffer by combining all the x bytes and all the y bytes
	float x = extract(buffer[MOST_SIG_X], buffer[X], buffer[DECIMAL_X]);
	float y = extract(buffer[MOST_SIG_Y], buffer[Y], buffer[DECIMAL_Y]);
	//updates the x, y, and direction
	environment.robots[buffer[ROBOT_ID]].x = x;
	environment.robots[buffer[ROBOT_ID]].y = y;
	environment.robots[buffer[ROBOT_ID]].direction = buffer[DIRECTION];
	//if flag is there, makes the robot direction negative
	if(buffer[NEGATIVE] == 1){
		environment.robots[buffer[ROBOT_ID]].direction *= -1;
	}
}

//looks for a free starting location for a robot
void startingCoordinates(Robot *robot){
	float x, y;
	//creates random values until it finds coordinates that will not be moved into immediately by a different robot
	do{
		x = rand()%((ENV_SIZE - 2*ROBOT_RADIUS) + 1) + ROBOT_RADIUS;			///make this random
		y = rand()%((ENV_SIZE - 2*ROBOT_RADIUS) + 1) + ROBOT_RADIUS;
	}while(collidesWithRobot(x, y));
	//sets the new x and y after it leaves the while loop (means it found a unique location)
	robot->x = x;
	robot->y = y;
}

//checks if 2 robots collide or hits boundary, sends back 1 byte
void checkCollision(){
	//this is the coordinates of the current robot and direction
	float x1 = extract(buffer[MOST_SIG_X], buffer[X], buffer[DECIMAL_X]);
	float y1 = extract(buffer[MOST_SIG_Y], buffer[Y], buffer[DECIMAL_Y]);
	float inRadians = buffer[DIRECTION] * PI / 180;
	if(buffer[NEGATIVE] == 1){
		inRadians *= -1;
	}
	//calculates its new location
	x1 = x1+ ROBOT_SPEED*cos(inRadians);
	y1 = y1+ ROBOT_SPEED*sin(inRadians);

	//if it will hit the boundary on its next movement
	if(x1  + ROBOT_RADIUS >= ENV_SIZE || y1 + ROBOT_RADIUS >= ENV_SIZE || x1 - ROBOT_RADIUS <= 0 || y1 - ROBOT_RADIUS <= 0){
		sendCommand[RESPONSE] = NOT_OK_BOUNDARY;
		return;
	}
	if(collidesWithRobot(x1, y1)){
		sendCommand[RESPONSE] = NOT_OK_COLLIDE;
		return;
	}
	//if its fine, then sends back 1
	sendCommand[RESPONSE] = OK;
}

//checks to see if the current robot in the buffer collides with any of the other robots
int collidesWithRobot(float x1, float y1){
	//goes through all the robots and checks collisions
	for(int i =0; i < environment.numRobots; i++){
		//can't compare to itself
		if(buffer[ROBOT_ID] == i){
			continue;
		}
		//dont need to extract because they're already floats
		float x2 = environment.robots[i].x;
		float y2 = environment.robots[i].y;
		//float direction = environment.robots[i].direction;
		//float radians = direction * PI / 180;	//dont need to check flag, already negative if it is
		//x2 = x2 + ROBOT_SPEED * cos(radians);
		//y2 = y2 + ROBOT_SPEED * sin(radians);
		float distance = (sqrt(((x2 - x1) * (x2 - x1)) + ((y2-y1)*(y2-y1)))) - 2 * ROBOT_RADIUS;
		//means 2 robots would collide
		if(distance <= 0){
			return 1;
		}
	}
	return 0;
}

//main method
int main() {
	srand(time(NULL));
	// So far, the environment is NOT shut down
	environment.shutDown = 0;
	//creates the threads
	pthread_t requests;
	pthread_t drawing;
	pthread_create(&requests, NULL, handleIncomingRequests,(void*)&environment);
	pthread_create(&drawing, NULL, redraw, (void*)&environment);
	//until the threads exit
	pthread_join(requests, NULL);
	pthread_join(drawing, NULL);
}
