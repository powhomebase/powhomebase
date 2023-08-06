#pragma once

#include <cmsis_compiler.h>

#define OS_ATTR_STATIC_SEMAPHORE_CREATE(x) (sem_t){0}
#define osWaitForever 0xFFFFFFFF

typedef struct {
    uint32_t count;
    uint32_t max_count;
} sem_t;


typedef sem_t* osSemaphoreId_t;
typedef sem_t osSemaphoreAttr_t;

typedef enum {
    osOK
} osStatus_t;

osSemaphoreId_t osSemaphoreNew(uint32_t max_count, uint32_t initial_count, osSemaphoreAttr_t *attr);

osStatus_t osSemaphoreAcquire (osSemaphoreId_t semaphore_id, uint32_t timeout);

osStatus_t osSemaphoreRelease (osSemaphoreId_t semaphore_id);

uint32_t osSemaphoreGetCount (osSemaphoreId_t semaphore_id);
