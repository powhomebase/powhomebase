/***********************************************************************************************************************
 * Description: Types for every occasion.
 **********************************************************************************************************************/
#pragma once

/***********************************************************************************************************************
 * Typedefs
 **********************************************************************************************************************/

typedef enum result_e {
    ERR_BUSY        = -3,
    ERR_UNSUPPORTED = -2,
    ERR             = -1,
    OK,
} result_e;

/* Data Append Socket events for the manager */
typedef enum {
    FLUSH_ENDED_SUCCESSFULLY,
    FLUSH_ENDED_ERROR,
} flush_events_e;

/* A callback definition for notifying on Data flush events */
typedef void (*flush_event_cb)(flush_events_e event, void* p_parent);