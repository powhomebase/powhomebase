
#include "cmsis_os2.h"


osSemaphoreId_t osSemaphoreNew(uint32_t max_count, uint32_t initial_count, osSemaphoreAttr_t *attr){
    sem_t * p_this = attr;

    p_this->count = initial_count;
    p_this->max_count = max_count;

    return p_this;
}

osStatus_t osSemaphoreAcquire (osSemaphoreId_t semaphore_id, uint32_t timeout){
    sem_t * p_this = semaphore_id;

    while(p_this->count == 0){
    }

    p_this->count--;

    return osOK;
}

osStatus_t osSemaphoreRelease (osSemaphoreId_t semaphore_id)
{
    sem_t * p_this = semaphore_id;

    p_this->count++;

    if(p_this->count > p_this->max_count)
    {
        while(1){}
    }
}

uint32_t osSemaphoreGetCount (osSemaphoreId_t semaphore_id)
{
    sem_t * p_this = semaphore_id;
    return p_this->count;
}


osMutexId_t osMutexNew (const osMutexAttr_t *attr)
{
    mutex_t * p_this = attr;

    p_this->is_locked = 0;

    return p_this;
}

osStatus_t osMutexAcquire (osMutexId_t mutex_id, uint32_t timeout)
{
    mutex_t * p_this = mutex_id;

    while(p_this->is_locked == 1){
    }

    p_this->is_locked = 1;

    return osOK;
}

osStatus_t osMutexRelease (osMutexId_t mutex_id)
{
    mutex_t * p_this = mutex_id;
    
    p_this->is_locked = 0;

    return osOK;
}