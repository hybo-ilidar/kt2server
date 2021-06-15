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

/* Added for CRC */
static const uint16_t CRC16Table[256] = {
	0x0000,	0x1021,	0x2042,	0x3063,	0x4084,	0x50a5,	0x60c6,	0x70e7,
	0x8108,	0x9129,	0xa14a,	0xb16b,	0xc18c,	0xd1ad,	0xe1ce,	0xf1ef,
	0x1231,	0x0210,	0x3273,	0x2252,	0x52b5,	0x4294,	0x72f7,	0x62d6,
	0x9339,	0x8318,	0xb37b,	0xa35a,	0xd3bd,	0xc39c,	0xf3ff,	0xe3de,
	0x2462,	0x3443,	0x0420,	0x1401,	0x64e6,	0x74c7,	0x44a4,	0x5485,
	0xa56a,	0xb54b,	0x8528,	0x9509,	0xe5ee,	0xf5cf,	0xc5ac,	0xd58d,
	0x3653,	0x2672,	0x1611,	0x0630,	0x76d7,	0x66f6,	0x5695,	0x46b4,
	0xb75b,	0xa77a,	0x9719,	0x8738,	0xf7df,	0xe7fe,	0xd79d,	0xc7bc,
	0x48c4,	0x58e5,	0x6886,	0x78a7,	0x0840,	0x1861,	0x2802,	0x3823,
	0xc9cc,	0xd9ed,	0xe98e,	0xf9af,	0x8948,	0x9969,	0xa90a,	0xb92b,
	0x5af5,	0x4ad4,	0x7ab7,	0x6a96,	0x1a71,	0x0a50,	0x3a33,	0x2a12,
	0xdbfd,	0xcbdc,	0xfbbf,	0xeb9e,	0x9b79,	0x8b58,	0xbb3b,	0xab1a,
	0x6ca6,	0x7c87,	0x4ce4,	0x5cc5,	0x2c22,	0x3c03,	0x0c60,	0x1c41,
	0xedae,	0xfd8f,	0xcdec,	0xddcd,	0xad2a,	0xbd0b,	0x8d68,	0x9d49,
	0x7e97,	0x6eb6,	0x5ed5,	0x4ef4,	0x3e13,	0x2e32,	0x1e51,	0x0e70,
	0xff9f,	0xefbe,	0xdfdd,	0xcffc,	0xbf1b,	0xaf3a,	0x9f59,	0x8f78,
	0x9188,	0x81a9,	0xb1ca,	0xa1eb,	0xd10c,	0xc12d,	0xf14e,	0xe16f,
	0x1080,	0x00a1,	0x30c2,	0x20e3,	0x5004,	0x4025,	0x7046,	0x6067,
	0x83b9,	0x9398,	0xa3fb,	0xb3da,	0xc33d,	0xd31c,	0xe37f,	0xf35e,
	0x02b1,	0x1290,	0x22f3,	0x32d2,	0x4235,	0x5214,	0x6277,	0x7256,
	0xb5ea,	0xa5cb,	0x95a8,	0x8589,	0xf56e,	0xe54f,	0xd52c,	0xc50d,
	0x34e2,	0x24c3,	0x14a0,	0x0481,	0x7466,	0x6447,	0x5424,	0x4405,
	0xa7db,	0xb7fa,	0x8799,	0x97b8,	0xe75f,	0xf77e,	0xc71d,	0xd73c,
	0x26d3,	0x36f2,	0x0691,	0x16b0,	0x6657,	0x7676,	0x4615,	0x5634,
	0xd94c,	0xc96d,	0xf90e,	0xe92f,	0x99c8,	0x89e9,	0xb98a,	0xa9ab,
	0x5844,	0x4865,	0x7806,	0x6827,	0x18c0,	0x08e1,	0x3882,	0x28a3,
	0xcb7d,	0xdb5c,	0xeb3f,	0xfb1e,	0x8bf9,	0x9bd8,	0xabbb,	0xbb9a,
	0x4a75,	0x5a54,	0x6a37,	0x7a16,	0x0af1,	0x1ad0,	0x2ab3,	0x3a92,
	0xfd2e,	0xed0f,	0xdd6c,	0xcd4d,	0xbdaa,	0xad8b,	0x9de8,	0x8dc9,
	0x7c26,	0x6c07,	0x5c64,	0x4c45,	0x3ca2,	0x2c83,	0x1ce0,	0x0cc1,
	0xef1f,	0xff3e,	0xcf5d,	0xdf7c,	0xaf9b,	0xbfba,	0x8fd9,	0x9ff8,
	0x6e17,	0x7e36,	0x4e55,	0x5e74,	0x2e93,	0x3eb2,	0x0ed1,	0x1ef0
};
static uint16_t getCRC16(const uint8_t* packet, int length) {
	// use CRC-16 CCITT standard
	volatile uint16_t crc = 0;
	volatile int i;
	for (i = 0; i < length; i++) {
		crc = (crc << 8) ^ (CRC16Table[((crc >> 8) ^ packet[i]) & 0xFF]);
	}
	return crc;
}
/* Added for CRC */

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
      // CRC UPDATE  15-Jun-2021
      // additional 2 bytes of CRC at the end
      if (ptr > 9000
        || (ptr == 0 && ((buffer[0] != 0x5A)))
        || (ptr == 1 && ((buffer[0] != 0x5A) || (buffer[1] != 0xA5)))
        || (ptr == 2 && ((buffer[0] != 0x5A) || (buffer[1] != 0xA5) || (buffer[2] != 0x5A)))
        || (ptr >= 3 && ((buffer[0] != 0x5A) || (buffer[1] != 0xA5) || (buffer[2] != 0x5A) || (buffer[3] != 0xA5)))
        ) ptr = -1; // Reset the pointer

      if (ptr == 320*2 + 9) { // buffer is properly filled, process the data
        ptr = -1; // Reset the pointer
        // Print current row # to see what is going on
        // printf("%d ", buffer[4]);
        int row = buffer[4];    // Row #
        int imgType = buffer[5];  // Type of the image - 0 = Depth, 1 = Grayscale
        uint16_t CRC_packet = *(uint16_t*)&buffer[648];
        uint16_t CRC_calculated = getCRC16(&buffer[4], 644);

        // If image type and row number and the CRC are valid...
        if( (imgType >= 0 && imgType <= 1) && (row >= 0 && row < 40) 
            && (CRC_packet == CRC_calculated) ) {
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

