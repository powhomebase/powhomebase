/***********************************************************************************************************************
 * Description: Header containing definition of timestamp
 **********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Project related */
#include <reset_manager.h>

/**********************************************************************************************************************/
/* Typedefs                                                                                                           */
/**********************************************************************************************************************/

/* Timestamp type definition */
typedef struct __PACKED {
    uint16_t reset_counter;
    uint32_t rtc_counter_ms;
} timestamp_t;