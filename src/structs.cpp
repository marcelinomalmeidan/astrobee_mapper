
#include "structs.h"

void initializeMutexes(mutexStruct &mutexes){
	pthread_mutex_init(&mutexes.sampledTraj, NULL);
	pthread_mutex_init(&mutexes.tf, NULL);
	pthread_mutex_init(&mutexes.threadCount, NULL);
	pthread_mutex_init(&mutexes.obsTree, NULL);
}

void destroyMutexes(mutexStruct &mutexes){
	pthread_mutex_destroy(&mutexes.sampledTraj);
	pthread_mutex_destroy(&mutexes.tf);
	pthread_mutex_destroy(&mutexes.threadCount);
	pthread_mutex_destroy(&mutexes.obsTree);
}