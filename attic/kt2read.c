#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include "libserialport.h"
 
#define DEFBAUD 3000000
#define DEFPORTNAME "/dev/null"
#define MAXPORTNAME (128)
char portname[MAXPORTNAME];

// Shared variables, not synchronized
int imgReady = 0;

int nframes_rcvd = 0;
int nframes_sent = 0;
#define MODULO_DISPLAY (1)

unsigned char sat[2][320][160];
short img[2][320][160];

// Just send raw bytes over UDP
uint8_t udp_buf[40][646]; 
uint8_t udp_queue[40][646]; 

// Row count for each image type
// Currently 40 for depth, 160 for grayscale
const int cnt_row[2] = {40, 160};


int main(int argc, char *argv[]) {
  struct sp_port *port;
  struct sp_port_config *port_config;
  int err;
  int iarg=0;
  bool looping = true;
  int waiting=0;
  int nrb=0;
  int baud=DEFBAUD;


  unsigned char buffer[5000];     // Row buffer
  unsigned char serialArray[2048];  // Scratchpad buffer
  unsigned short c;
  int ptr = 0;
  int i, j;
  int bufPtr;

  short buf;

 
// We need a serial port name
  if (argc<2) {
    fprintf(stderr,"Usage:\n");
    fprintf(stderr,"  rdport serial-port [baud]\n");
    exit(1);
  }

  iarg++;
  if (argc>iarg) {
    strncpy( portname, argv[iarg], MAXPORTNAME );
  } else {
    strncpy( portname, DEFPORTNAME, MAXPORTNAME );
  }

// optional baudrate
  iarg++;
  if (argc>iarg) {
    char *endptr=NULL;
    errno=0;
    fprintf(stderr, "baudstring: %d, %s\n", iarg, argv[iarg]);
    baud = (int)strtol( argv[iarg], &endptr, 10 );
    if(endptr == argv[iarg]) {
      fprintf(stderr,"error reading baudrate: %s\n", argv[iarg] );
      exit(99);
    }
  }
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

// read data 
#define TIMEOUT_MAX (500000)
  uint32_t timeout_cnt = 0;
  bool finished = false;
  while(!finished) {
    // Get some data to scratchpad
    // int maxCnt = serialReadBlock(serialArray, 1000);
    int maxCnt = sp_nonblocking_read(port, (void*)serialArray, (size_t)1000);
		//if(maxCnt) printf("maxCnt: %d\n", maxCnt);
    if(maxCnt < 0) {
      printf("error %d\n", maxCnt);
      exit(0);
    }

    // Iterate over the received data
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
						//uint64_t dt = toc();
						//_timeOffset = tic();
					  // transfer the intermediate udp queue buffer into udp transmit buffer
					  for(i=0;i<40;i++) for(j=0;j<646;j++) udp_buf[i][j] = udp_queue[i][j];
						// Print a message to see what is going on
						// printf("\nDone. dt: %lld Image Type: %d imgReady %d\n\n", dt, imgType, imgReady);
						nframes_rcvd++;
						if( 0 == (nframes_rcvd % MODULO_DISPLAY) ) 
							printf("rcvd: %10d\n", nframes_rcvd );
						imgReady = 1; // Signal to the udp thread / weak synchronization
					} // END final line of video
        } // END line of video is valid (row and type)
      } // END valid line of video data reveived
      ptr++; // Get to next place
		} // END loop over this buffer from serial port read
  } // END while(1) loop

  sp_close(port);
}

