/***********************************************************************************************************************
 * Description: Internal flash driver for STM32 declarations
 **********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Project Related */
#include <Driver_Flash.h>
#include <global.h>
#include <cmsis_os2.h>
#include CMSIS_device_header

/* Libraries */
#include <assert.h>

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/
#define CONFIG_FLASH_SIZE (FLASH_SIZE)
/***********************************************************************************************************************
 * Description: Generates an Internal Flash driver object
 **********************************************************************************************************************/
#define INTERNAL_FLASH_GENERATE_OBJECT()                                                                               \
    ARM_DRIVER_FLASH Driver_Flash = {                                                                                  \
        .Initialize   = internal_flash_Initialize,                                                                                     \
        .Uninitialize = internal_flash_Uninitialize,                                                                                   \
        .PowerControl = internal_flash_PowerControl,                                                                                   \
        .ReadData     = internal_flash_ReadData,                                                                                       \
        .ProgramData  = internal_flash_ProgramData,                                                                                    \
        .EraseSector  = internal_flash_EraseSector,                                                                                    \
        .EraseChip    = internal_flash_EraseChip,                                                                                      \
        .GetStatus    = internal_flash_GetStatus,                                                                                      \
        .GetInfo      = internal_flash_GetInfo,                                                                                        \
    }

/**********************************************************************************************************************/
/* Interface Function Declarations                                                                                    */
/**********************************************************************************************************************/

/* Initializes the internal flash driver */
int32_t internal_flash_Initialize(ARM_Flash_SignalEvent_t cb_event);

/* Uninitializes the internal flash driver */
int32_t internal_flash_Uninitialize(void);

/* Controls the power state of the internal flash */
int32_t internal_flash_PowerControl(ARM_POWER_STATE state);

/* Reads data from flash into buffer */
int32_t internal_flash_ReadData(uint32_t addr, void *data, uint32_t cnt);

/* Writes data from buffer to flash */
int32_t internal_flash_ProgramData(uint32_t addr, const void *data, uint32_t cnt);

/* Erases a sector within the flash */
int32_t internal_flash_EraseSector(uint32_t addr);

/* Completely erases flash */
int32_t internal_flash_EraseChip(void);

/* Returns status of the internal flash/driver */
ARM_FLASH_STATUS internal_flash_GetStatus(void);

/* Returns flash info */
ARM_FLASH_INFO *internal_flash_GetInfo(void);