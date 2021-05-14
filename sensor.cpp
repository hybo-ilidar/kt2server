// Hybo-KT LIDAR Development Project
// Viewer/Logger program
// 15 March 2021
// OpenCV 2.4.0 / pthread
// Son, Youngbin
// tech@hybo.co

// 4 May 2021 - Row count is changed to 40
// 5 May 2021 - replaced viewer code with UDP server

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <cstring>

using namespace std;

#include "libserialport.h"
#include "main.h"

// Shared variables, not synchronized
int imgReady = 0;

int nframes_rcvd = 0;
int nframes_sent = 0;
#define MODULO_DISPLAY (100)

unsigned char sat[2][320][160];
short img[2][320][160];

// Just send raw bytes over UDP
uint8_t udp_buf[40][646]; 
uint8_t udp_queue[40][646]; 

// Row count for each image type
// Currently 40 for depth, 160 for grayscale
const int cnt_row[2] = {40, 160};

// tSerial
// Get data from the sensor over serial port
// Should be called as a real-time thread (high priority)
void Serial(int thread_argument) {

  struct sp_port *port;
  struct sp_port_config *port_config;
  unsigned char buffer[5000];     // Row buffer
  int baud;
  char portname[256] = "COM4";
  char baudrate[256] = "12000000";
  int err;
  


  unsigned char serialArray[2048];  // Scratchpad buffer
  unsigned short c;
  int ptr = 0;
  int i, j;
  int bufPtr;

  short buf;

  FILE *fp = fopen("./comport.txt", "r");
  if(fp == NULL){
    printf("Using default port: %s\n", portname);
    printf("Note: port in comport.txt can override default\n");
  } else {
    fscanf(fp, "%s", portname);
    fclose(fp);
  }

// baudrate (this could come from user argument as string)
  char *endptr=NULL;
  errno=0;
  baud = (int)strtol( baudrate, &endptr, 10 );
  if(endptr == baudrate) {
    fprintf(stderr,"error reading baudrate: %s\n", baudrate );
    exit(0);
  }


// summarize results to user:
  fprintf(stderr,"Requested serial port %s at %d baud\n",portname, baud);

// Open serial port
  err=sp_get_port_by_name(portname,&port);
  if (err==SP_OK)
  err=sp_open(port,SP_MODE_READ_WRITE);
  if (err!=SP_OK) {
    fprintf(stderr,"Can't open port %s\n", portname);
    exit(2);
  }
  fprintf(stderr,"Serial port %s opened\n",portname);

  // set flowcontrol
  // this is tedious, because need to create and 
  // load the port config structure
  err = sp_new_config( &port_config );
  if (err!=SP_OK) {
    fprintf(stderr,"Error allocating port configuration\n");
    exit(2);
  }
  err = sp_get_config( port, port_config );
  if (err!=SP_OK) {
    fprintf(stderr,"Error getting port configuration\n");
    exit(2);
  }
  err = sp_set_config_flowcontrol(port_config, SP_FLOWCONTROL_NONE);
  if (err!=SP_OK) {
    fprintf(stderr,"Can't set flowcontrol\n");
    exit(2);
  }
  err = sp_set_config(port, port_config);
  fprintf(stderr,"Set no flowcontrol\n");
  if (err!=SP_OK) {
    fprintf(stderr,"Can't set port config\n");
    exit(2);
  }
  fprintf(stderr,"Set flowcontrol to off\n");

// set Baud rate
  sp_set_baudrate(port,baud);
  fprintf(stderr,"Set baudrate %d\n", baud);

// flush serial port buffers
  sp_flush( port, SP_BUF_BOTH );
  fprintf(stderr,"flushed serial port buffers\n");

  auto t0 = tic();
  auto tref = t0;
  while(1) {
    // Get some data to scratchpad
    // int maxCnt = serialReadBlock(serialArray, 1000);
    int maxCnt = sp_nonblocking_read(port, (void*)serialArray, (size_t)1000);
    
    // Iterate over the received data
		// printf("maxCnt: %d\n", maxCnt);
    for (bufPtr = 0; bufPtr<maxCnt; bufPtr++) {
      c = serialArray[bufPtr]; // Get a byte
      buffer[ptr] = (char)(c & 0xff); // Fill the buffer with the byte
      // Synchronization: 'buffer' array should look like: {0x5A 0xA5 0x5A 0xA5 ... }
      // Discard data if such pattern is not made at the first part of the buffer.
      // Current packet looks like...
      // Preamble     Row#    ImgType     Pixel0    Pixel1    ...   Pixel319
      // 4B:0x5AA55AA5  1B:0~159  1B:0 or 1   2B      2B      ...   2B
      if (ptr > 9000
        || (ptr == 0 && ((buffer[0] != 0x5A)))
        || (ptr == 1 && ((buffer[0] != 0x5A) || (buffer[1] != 0xA5)))
        || (ptr == 2 && ((buffer[0] != 0x5A) || (buffer[1] != 0xA5) || (buffer[2] != 0x5A)))
        || (ptr >= 3 && ((buffer[0] != 0x5A) || (buffer[1] != 0xA5) || (buffer[2] != 0x5A) || (buffer[3] != 0xA5)))
        ) ptr = -1; // Reset the pointer

      if (ptr == 320*2 + 5) { // buffer is properly filled, process the data
        ptr = -1; // Reset the pointer
        // Print current row # to see what is going on
        // printf("%d ", buffer[4]);
        int row = buffer[4];    // Row #
        int imgType = buffer[5];  // Type of the image - 0 = Depth, 1 = Grayscale
        // If image type and row number is valid...
        if( (imgType >= 0 && imgType <= 1) && (row >= 0 && row < 40) ) {
					// Copy this row's data to UDP intermediate queue buffer
					for(i=0;i<646;i++) udp_queue[row][i] = buffer[i]; 
					if(buffer[4] == cnt_row[imgType]-1) { // this is the final row...
						uint64_t dt = toc_u64(_timeOffset); // frame timestamping
						_timeOffset = tic();
					  // transfer the intermediate udp queue buffer into udp transmit buffer
					  for(i=0;i<40;i++) for(j=0;j<646;j++) udp_buf[i][j] = udp_queue[i][j];
						// Print a message to see what is going on
						// printf("SERrecv dt: %ld Image Type: %d imgReady %d\n", dt, imgType, imgReady);
						nframes_rcvd++;
						if( 0 == (nframes_rcvd % MODULO_DISPLAY) ) {
              auto t = tic();
              double dt = toc_double(tref);
              double fps = (double)(MODULO_DISPLAY) / dt;
              tref = t;
              printf("rcvd: %10d dt: %.3lf fps: %.1lf\n", nframes_rcvd, dt, fps );
            }
						imgReady = 1; // Signal to the udp thread / weak synchronization
					} // END final line of video
        } // END line of video is valid (row and type)
      } // END valid line of video data reveived
      ptr++; // Get to next place
		} // END loop over this buffer from serial port read
  } // END while(1) loop

  sp_close(port);
}



// Note: size of one frame
// 40 * 646 = 25840
// udp server
void UdpServer(int thread_argument) {
  int i,j;
  
  int sockfd;
  int length;

  if((sockfd = socket(AF_INET,SOCK_DGRAM,0)) < 0) {
    printf("Socket Opening Failed\n");
    return;
  }

  // get host address (for info only?)
  // see here for some examples:  https://baboc.tistory.com/262
  char local[256] = "localhost"; // will hold host name
  gethostname(local,256);
  struct hostent* pHostEnt = gethostbyname(local);
  struct in_addr lcl;
  char *lcl_addr;
  lcl.s_addr =((struct in_addr*)pHostEnt->h_addr)->s_addr;  
  lcl_addr = inet_ntoa(lcl);  
  printf("Local IP Addr : %s\n", lcl_addr);  
  
  int myIP = inet_addr(lcl_addr);  
  int subnet = inet_addr("255.255.255.0");  
  int temp1 = myIP & subnet;  
  int temp2 = subnet ^ 0xFFFFFFFF;  

  // make destination address
  struct sockaddr_in addr;
  memset((void *)&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(8282);
  addr.sin_addr.s_addr = temp1 + temp2;
  //addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  //addr.sin_addr.s_addr = inet_addr("192.168.1.250");

  int broadcast = true;
  if( setsockopt( sockfd, SOL_SOCKET, SO_BROADCAST, (void *)&broadcast, sizeof(broadcast) ) ) {
    printf("setting broadcast option Failed\n");
    return;
  }

  printf("Socket ready\n");

  auto t0 = tic();
  auto tref = t0;
  while(1){
    if(!imgReady) {
      usleep(1000); // sleep 1 ms
      continue;
    }

    // release intermetdiate queue
		imgReady = 0;

    for(i=0;i<40;i++) { // send the whole image as 40 UDP datagrams 646 bytes long
		  int nbytes = sendto( sockfd, (const char*)udp_buf[i], 646, 0, (struct sockaddr*)&addr, sizeof(addr));
		  if( nbytes < 0) {
			  perror("sendto");
			  return;
		  }
    }
		// printf("Send one frame\n" );
	  // putchar('x');
    nframes_sent++;
		if( 0 == (nframes_sent % MODULO_DISPLAY) ) {
      auto t = tic();
      double dt = toc_double(tref);
      double fps = (double)(MODULO_DISPLAY) / dt;
      tref = t;
      printf("sent: %10d dt: %.3lf fps: %.1lf\n", nframes_sent, dt, fps );
    }

		// Log the image if below key is pressed (focus should be at some opencv window)
		// The images are 16bit unsigned. Some viewer programs may not cope with this.
		// Do not delete waitKey function or this will not work
		// char ch = cv::waitKey(1);
		// else if(ch == 'q')
		//  // quit here???
		// else
		//  Sleep(1);

  } // end forever loop

}

