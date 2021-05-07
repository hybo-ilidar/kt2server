// Hybo-KT LIDAR Development Project
// Viewer/Logger program
// 15 March 2021
// OpenCV 2.4.0 / pthread
// Son, Youngbin
// tech@hybo.co

// 4 May 2021 - Row count is changed to 40

#ifndef MAIN_H
#define MAIN_H

#include <chrono>


// Timer function
typedef chrono::time_point<chrono::high_resolution_clock> ABSTIME;

extern ABSTIME _timeOffset;
extern ABSTIME tic(void);
extern uint64_t toc_u64(ABSTIME tref);
extern double   toc_double(ABSTIME tref);

extern void Serial(int thread_argument);
extern void UdpServer( int thread_argument);

#endif

