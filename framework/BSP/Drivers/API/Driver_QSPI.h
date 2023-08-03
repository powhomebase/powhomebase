/* -----------------------------------------------------------------------------
 *
 * $Date:        17. Apr 2014
 * $Revision:    V2.01
 *
 * Project:      QSPI (Quad Serial Peripheral Interface) Driver definitions
 * -------------------------------------------------------------------------- */


#ifndef __DRIVER_QSPI_H
#define __DRIVER_QSPI_H

#include "Driver_Common.h"

#define ARM_QSPI_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,00)  /* API version */


/****** QSPI Send Command Modes Flags *****/
#define ARM_QSPI_INSMODE_Pos             0
#define ARM_QSPI_INSMODE_Msk            (3UL << ARM_QSPI_INSMODE_Pos)
#define ARM_QSPI_INSMODE_NONE           (0UL << ARM_QSPI_INSMODE_Pos)   ///< 
#define ARM_QSPI_INSMODE_1LINE          (1UL << ARM_QSPI_INSMODE_Pos)   ///< 
#define ARM_QSPI_INSMODE_2LINES         (2UL << ARM_QSPI_INSMODE_Pos)   ///< 
#define ARM_QSPI_INSMODE_4LINES         (3UL << ARM_QSPI_INSMODE_Pos)   ///< 

#define ARM_QSPI_ADDRESS_Pos             2
#define ARM_QSPI_ADDRESS_Msk            (3UL << ARM_QSPI_ADDRESS_Pos)
#define ARM_QSPI_ADDRESS_NONE           (0UL << ARM_QSPI_ADDRESS_Pos)   ///< 
#define ARM_QSPI_ADDRESS_1LINE          (1UL << ARM_QSPI_ADDRESS_Pos)   ///< 
#define ARM_QSPI_ADDRESS_2LINES          (2UL << ARM_QSPI_ADDRESS_Pos)   ///< 
#define ARM_QSPI_ADDRESS_4LINES          (3UL << ARM_QSPI_ADDRESS_Pos)   ///< 

#define ARM_QSPI_ABMODE_Pos             4
#define ARM_QSPI_ABMODE_Msk            (3UL << ARM_QSPI_ABMODE_Pos)
#define ARM_QSPI_ABMODE_NONE           (0UL << ARM_QSPI_ABMODE_Pos)   ///< 
#define ARM_QSPI_ABMODE_1LINE          (1UL << ARM_QSPI_ABMODE_Pos)   ///< 
#define ARM_QSPI_ABMODE_2LINES          (2UL << ARM_QSPI_ABMODE_Pos)   ///< 
#define ARM_QSPI_ABMODE_4LINES          (3UL << ARM_QSPI_ABMODE_Pos)   ///< 

#define ARM_QSPI_DATAMODE_Pos             6
#define ARM_QSPI_DATAMODE_Msk            (3UL << ARM_QSPI_DATAMODE_Pos)
#define ARM_QSPI_DATAMODE_NONE           (0UL << ARM_QSPI_DATAMODE_Pos)   ///< 
#define ARM_QSPI_DATAMODE_1LINE          (1UL << ARM_QSPI_DATAMODE_Pos)   ///< 
#define ARM_QSPI_DATAMODE_2LINES          (2UL << ARM_QSPI_DATAMODE_Pos)   ///< 
#define ARM_QSPI_DATAMODE_4LINES          (3UL << ARM_QSPI_DATAMODE_Pos)   ///< 

#define ARM_QSPI_ADDRESS_SIZE_Pos        8
#define ARM_QSPI_ADDRESS_SIZE_Msk        (3UL << ARM_QSPI_ADDRESS_SIZE_Pos)
#define ARM_QSPI_ADDRESS_SIZE_8BITS      (0UL << ARM_QSPI_ADDRESS_SIZE_Pos)   ///< 
#define ARM_QSPI_ADDRESS_SIZE_16BITS     (1UL << ARM_QSPI_ADDRESS_SIZE_Pos)   ///< 
#define ARM_QSPI_ADDRESS_SIZE_24BITS     (2UL << ARM_QSPI_ADDRESS_SIZE_Pos)   ///< 
#define ARM_QSPI_ADDRESS_SIZE_32BITS     (3UL << ARM_QSPI_ADDRESS_SIZE_Pos)   ///< 

#define ARM_QSPI_AB_SIZE_Pos             10
#define ARM_QSPI_AB_SIZE_Msk             (3UL << ARM_QSPI_AB_SIZE_Pos)
#define ARM_QSPI_AB_SIZE_8BITS           (0UL << ARM_QSPI_AB_SIZE_Pos)   ///< 
#define ARM_QSPI_AB_SIZE_16BITS          (1UL << ARM_QSPI_AB_SIZE_Pos)   ///< 
#define ARM_QSPI_AB_SIZE_24BITS          (2UL << ARM_QSPI_AB_SIZE_Pos)   ///< 
#define ARM_QSPI_AB_SIZE_32BITS          (3UL << ARM_QSPI_AB_SIZE_Pos)   ///< 

#define ARM_QSPI_CMDTYPE_Pos             12
#define ARM_QSPI_CMDTYPE_Msk             (3UL << ARM_QSPI_CMDTYPE_Pos)
#define ARM_QSPI_CMDTYPE_NORMAL          (0UL << ARM_QSPI_CMDTYPE_Pos)   ///< 
#define ARM_QSPI_CMDTYPE_TX              (1UL << ARM_QSPI_CMDTYPE_Pos)   ///< 
#define ARM_QSPI_CMDTYPE_RX              (2UL << ARM_QSPI_CMDTYPE_Pos)   ///< 
#define ARM_QSPI_CMDTYPE_FUTUREUSE       (3UL << ARM_QSPI_CMDTYPE_Pos)   ///< 


/****** QSPI Control Codes *****/

#define ARM_QSPI_CONTROL_Pos              0
#define ARM_QSPI_CONTROL_Msk             (0xFFUL << ARM_QSPI_CONTROL_Pos)

/*----- SPI Control Codes: Mode -----*/
#define ARM_QSPI_MODE_INACTIVE           (0x00UL << ARM_SPI_CONTROL_Pos)     ///< SPI Inactive
#define ARM_QSPI_MODE_MASTER_SIMPLEX     (0x03UL << ARM_SPI_CONTROL_Pos)     ///< SPI Master (Output/Input on MOSI); arg = Bus Speed in bps
#define ARM_QSPI_MODE_SLAVE_SIMPLEX      (0x04UL << ARM_SPI_CONTROL_Pos)     ///< SPI Slave  (Output/Input on MISO)

/****** SPI Slave Select Signal definitions *****/
#define ARM_SPI_SS_INACTIVE              0                                  ///< SPI Slave Select Signal Inactive
#define ARM_SPI_SS_ACTIVE                1                                  ///< SPI Slave Select Signal Active


/****** SPI specific error codes *****/
#define ARM_SPI_ERROR_MODE              (ARM_DRIVER_ERROR_SPECIFIC - 1)     ///< Specified Mode not supported


/****** SPI Autopooling Autostop *****/
#define ARM_QSPI_DRIVER_AUTOPOOL_AUTOSTOP_DISABLED           (0x00)
#define ARM_QSPI_DRIVER_AUTOPOOL_AUTOSTOP_ENABLE             (0x01)

/****** SPI Autopooling Match Mode *****/
#define ARM_QSPI_DRIVER_AUTOPOOL_MATCHMODE_AND               (0x00)
#define ARM_QSPI_DRIVER_AUTOPOOL_MATCHMODE_OR                (0x01)




/**
\brief SPI Status
*/
typedef struct _ARM_QSPI_STATUS {
  uint32_t busy       : 1;              ///< Transmitter/Receiver busy flag
  uint32_t data_lost  : 1;              ///< Data lost: Receive overflow / Transmit underflow (cleared on start of transfer operation)
} ARM_QSPI_STATUS;


/****** SPI Event *****/
#define ARM_QSPI_EVENT_TRANSFER_COMPLETE (1UL << 0)  ///< Data Transfer completed
#define ARM_QSPI_EVENT_DATA_LOST         (1UL << 1)
#define ARM_QSPI_EVENT_AUTOPOL_TRIGGER   (1UL << 2)

// Function documentation

typedef void (*ARM_QSPI_SignalEvent_t) (uint32_t event);  ///< Pointer to \ref ARM_SPI_SignalEvent : Signal SPI Event.


/**
\brief SPI Driver Capabilities.
*/
typedef struct _ARM_QSPI_CAPABILITIES {
  uint32_t simplex          : 1;        ///< supports Simplex Mode (Master and Slave)
} ARM_QSPI_CAPABILITIES;


/**
\brief Access structure of the SPI Driver.
*/
typedef struct _ARM_DRIVER_QSPI {
  ARM_DRIVER_VERSION   (*GetVersion)      (void);                             ///< Pointer to \ref ARM_SPI_GetVersion : Get driver version.
  ARM_QSPI_CAPABILITIES(*GetCapabilities) (void);                             ///< Pointer to \ref ARM_SPI_GetCapabilities : Get driver capabilities.
  int32_t              (*Initialize)      (ARM_QSPI_SignalEvent_t cb_event);  ///< Pointer to \ref ARM_SPI_Initialize : Initialize SPI Interface.
  int32_t              (*Uninitialize)    (void);                             ///< Pointer to \ref ARM_SPI_Uninitialize : De-initialize SPI Interface.
  int32_t              (*PowerControl)    (ARM_POWER_STATE state);            ///< Pointer to \ref ARM_SPI_PowerControl : Control SPI Interface Power.
  int32_t              (*SendCommand)     (uint32_t  cmd_modes,
                                           uint32_t instruction,
                                           uint32_t address,
                                           uint32_t alternate_bytes,
                                           void *data,
                                           uint32_t data_num,
                                           uint32_t dummy_clocks);
  int32_t              (*SetAutoPolling)  (uint32_t match,
                                           uint32_t mask,
                                           uint8_t match_mode,
                                           uint8_t status_byte_size,
                                           uint16_t interval,
                                           uint8_t auto_stop);   ///< Pointer to \ref ARM_SPI_Receive : Start receiving data from SPI Interface.
  int32_t              (*Control)         (uint32_t control, uint32_t arg);   ///< Pointer to \ref ARM_SPI_Control : Control SPI Interface.
  ARM_QSPI_STATUS      (*GetStatus)       (void);                             ///< Pointer to \ref ARM_SPI_GetStatus : Get SPI status.
} const ARM_DRIVER_QSPI;

#endif /* __DRIVER_QSPI_H */

