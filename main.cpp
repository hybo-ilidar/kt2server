// Hybo-KT LIDAR Development Project
// Viewer/Logger program
// 15 March 2021
// OpenCV 2.4.0 / pthread
// Son, Youngbin
// tech@hybo.co

// 4 May 2021 - Row count is changed to 40
// 7 May 2021:
//   modified to run under Linux Ubuntu
//   (note: libserialport from sigrok required)


#include <thread>
#include <iostream>
#include <chrono>
#include <cstring>
#include <pthread.h>
using namespace std;

#include "main.h"

// timer definition
// host clock definition
ABSTIME _timeOffset;
ABSTIME _clockOffset;
double timeHost(void);

int main(void) {
  int th_id;
  int status;
  int argument=0;
  int ret;
  
  // timer initialization
  _timeOffset = tic();
  _clockOffset = _timeOffset; // sets the beginning of program execution

  // Thread initiation
  sched_param sch;
  int priority;

  thread tSensor( Serial, argument );
  thread tUdpServer( UdpServer, argument );

  pthread_getschedparam(tSensor.native_handle(), &priority, &sch);
  sch.sched_priority = sched_get_priority_max(priority);
  if( pthread_setschedparam(tSensor.native_handle(), SCHED_FIFO, &sch)) {
    cout << "Error setting priority: " << strerror(errno) << endl;
  }

  tSensor.join();
  tUdpServer.join();

  return 0;
}

ABSTIME tic(void) {
  return chrono::high_resolution_clock::now();
}

uint64_t toc_u64(ABSTIME tref) {
  auto t1 = tic();
  uint64_t dt = chrono::duration_cast<chrono::microseconds>(t1 - tref).count();
  return dt;
}

double toc_double(ABSTIME tref) {
  return ((double)toc_u64(tref)) / 1000000.0;
}

