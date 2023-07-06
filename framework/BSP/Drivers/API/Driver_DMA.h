#ifndef __DRIVER_DMA_H
#define __DRIVER_DMA_H


#include "Driver_Common.h"

#define ARM_DMA_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0)  /* API version */

/****** DMA Control Codes *****/
#define ARM_DMA_CONTROL_Pos                  0
#define ARM_DMA_CONTROL_Msk                 (0x3UL << ARM_DMA_CONTROL_Pos)

/*----- DMA Control Codes -----*/
#define ARM_DMA_STREAMING                   (1UL << ARM_DMA_CONTROL_Pos)  ///< Set DMA streaming flag. arg: 0 = disabled, 1 = enabled
#define ARM_DMA_STOP_STREAM                 (2UL << ARM_DMA_CONTROL_Pos)  ///< Stop streaming code 
#define ARM_DMA_ABORT_TRANSFER              (3UL << ARM_DMA_CONTROL_Pos)  ///< Abort current DMA transfer

/**
\brief DMA Status
*/
typedef struct _ARM_DMA_STATUS {
  uint32_t busy             : 1;        ///< DMA transmission busy flag
  uint32_t streaming        : 1;        ///< DMA streaming flag (1 = streaming, 0 = not streaming)
  uint32_t underflow        : 1;        ///< Transmit data underflow detected (cleared on start of next send operation)
  uint32_t overflow         : 1;        ///< Receive data overflow detected (cleared on start of next receive operation)
} ARM_DMA_STATUS;

/****** DMA Event *****/
#define ARM_DMA_EVENT_TRANSFER_COMPLETE     (1UL << 0)  ///< Send completed; however USART may still transmit data
#define ARM_DMA_EVENT_ERROR                 (1UL << 1)  ///< DMA error 
#define ARM_DMA_EVENT_OVERFLOW              (1UL << 2)  ///< DMA overflow
#define ARM_DMA_EVENT_HALF_BUFFER           (1UL << 3)  ///< Half of the buffer has been filled

/* Signal DMA events */
typedef void (*ARM_DMA_SignalEvent_t) (uint32_t event);

/**
\brief DMA Device Driver Capabilities.
*/
typedef struct _ARM_DMA_CAPABILITIES {
    uint32_t double_buffer  : 1;        ///< Two buffers available for DMA transactions 
} ARM_DMA_CAPABILITIES;

typedef struct _ARM_DRIVER_DMA {
  int32_t                (*Initialize)      (ARM_DMA_SignalEvent_t cb_event);
  int32_t                (*Uninitialize)    (void);                          
  int32_t                (*PowerControl)    (ARM_POWER_STATE state);         
  int32_t                (*Transfer)        (const void *source, void* dest, uint32_t num);
  uint32_t               (*GetTransferCount)(void);
  int32_t                (*Control)         (uint32_t control, uint32_t arg);    
  ARM_DMA_STATUS         (*GetStatus)       (void);                                                          
} const ARM_DRIVER_DMA;

#endif /* __DRIVER_DMA_H */
