#ifndef _H_THREADS_
#define _H_THREADS_

#include "globals.h"

//Thread for fading memory of the octomap
void *fadeTask(void *threadID);

//Thread for constantly updating the tfTree values
void *tfTask(void *threadID);

//Thread for publishing tfTree (used when loading real data without tf data)
void *tfPub(void *threadID);



#endif