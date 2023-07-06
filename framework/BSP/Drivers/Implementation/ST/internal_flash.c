/***********************************************************************************************************************
 * Description: Internal flash driver for STM32 implementation.
 * Note: STM32L4 implements a special handling of ECCD errors (this was copied from the L4 old internal flash driver).
 * In this implementation, in case of an ECCD, we'll try to write zeros to the defected double word (up to two write
 * trials). If we succeed, then we return the "fixed" buffer to the user.
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Own header */
#include "internal_flash.h"

/* Related Project's headers */
#include <faults.h>
#include <logging.h>
#include <proj_exception.h>
#include <syscalls.h>
#if defined(STM32F1)
#include <stm32f1xx_hal_flash.h>
#elif defined(STM32F4)
#include <flash_utils.h>
#include <stm32f4xx_hal_flash.h>
#elif defined(STM32L4)
#include <stm32l4xx_hal_flash.h>
#elif defined(STM32WL)
#include <stm32wlxx_hal_flash.h>
#else
#error "Unsupported device"
#endif

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

#if defined(FLASH_TYPEPROGRAM_BYTE) && IS_FLASH_TYPEPROGRAM(FLASH_TYPEPROGRAM_BYTE)
#define PROGRAM_UNIT 1
#define PROGRAM_TYPE FLASH_TYPEPROGRAM_BYTE
typedef uint8_t program_unit_t;


#elif defined(FLASH_TYPEPROGRAM_HALFWORD) && IS_FLASH_TYPEPROGRAM(FLASH_TYPEPROGRAM_HALFWORD)
#define PROGRAM_UNIT 2
#define PROGRAM_TYPE FLASH_TYPEPROGRAM_HALFWORD
typedef uint16_t program_unit_t;

#elif defined(FLASH_TYPEPROGRAM_WORD) && IS_FLASH_TYPEPROGRAM(FLASH_TYPEPROGRAM_WORD)
#define PROGRAM_UNIT 4
#define PROGRAM_TYPE FLASH_TYPEPROGRAM_WORD
typedef uint32_t program_unit_t;

#elif IS_FLASH_TYPEPROGRAM(FLASH_TYPEPROGRAM_DOUBLEWORD)
#define PROGRAM_UNIT 8
#define PROGRAM_TYPE FLASH_TYPEPROGRAM_DOUBLEWORD
typedef uint64_t program_unit_t;
#endif

#if defined(FLASH_BANK1_END)
#define BANK_SIZE (FLASH_BANK1_END - FLASH_BASE)
#else
#define BANK_SIZE CONFIG_FLASH_SIZE
#endif

#define ERASED_VALUE 0xFF

#if defined(STM32F4)
/* NOTE NOTE NOTE - STM32F4's memory layout is special.
   We treat our internal flash sectors 5,6 and 7 as 128 kB long pages. (We don't use the following functions
   on sectors 0-4).
*/
#define FLASH_PAGE_SIZE 0x20000  // Code is located at sectors 5, 6 and 7. Each sector is 128 kB.
#endif

/**********************************************************************************************************************/
/* Typedefs                                                                                                           */
/**********************************************************************************************************************/

typedef enum {
    INTFLASH_UNINITIALIZED,
    INTFLASH_INITIALIZED,
    INTFLASH_POWERED,
    INTFLASH_ERROR,
} internal_flash_state_t;

typedef struct {
    ARM_FLASH_INFO info; /* Flash driver information */
    osMutexAttr_t  mutex_attr;
} const internal_flash_conf_t;

typedef struct {
    internal_flash_state_t  state;          /* Flash driver state */
    osMutexId_t             flash_mutex_id; /* Flash mutex ID */
    ARM_FLASH_STATUS        status;         /* Flash driver status */
    ARM_Flash_SignalEvent_t callback;       /* Callback function */
#if defined(STM32L4)
    void* eccd_addr; /* ECCD address */
#endif               /* STM32L4 */
} internal_flash_resources_t;

/**********************************************************************************************************************/
/* Private Function Declarations                                                                                      */
/**********************************************************************************************************************/

#if defined(STM32L4) || defined(STM32WL)
static uint32_t get_page_num(uint32_t addr);
#endif

#if (defined(FLASH_BANK_1) || defined(FLASH_BANK_2)) && !defined(STM32F4)
static uint32_t get_bank(uint32_t addr);
#endif

#if defined(STM32L4)
static void* get_eccd_error_address(void);
static bool  nmi_handler(void* arg);
#endif /* STM32L4 */

/**********************************************************************************************************************/
/* Variables                                                                                                          */
/**********************************************************************************************************************/

static internal_flash_conf_t internal_flash_conf = {
    .info =
        {
            .sector_info  = NULL,                                  /* All sectors are of equal size - no map needed */
            .sector_count = (CONFIG_FLASH_SIZE / FLASH_PAGE_SIZE), /* Number of sectors/pages */
            .sector_size  = FLASH_PAGE_SIZE,
            .page_size    = FLASH_PAGE_SIZE,
            .program_unit = PROGRAM_UNIT, /* Single Programming unit size in bytes */
            .erased_value = ERASED_VALUE, /* Erase value */
        },
    .mutex_attr = OS_ATTR_STATIC_MUTEX_CREATE(Internal Flash, osMutexRecursive),
};

/* internal flash object resources initialization */
static internal_flash_resources_t this = {
    .state          = INTFLASH_UNINITIALIZED,
    .flash_mutex_id = NULL,
    .status =
        {
            .busy  = false,
            .error = false,
        },
#if defined(STM32L4)
    .eccd_addr = NULL,
#endif /* STM32L4 */
};

/**********************************************************************************************************************/
/* Interface Function Definitions                                                                                     */
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * Description: Initializes the internal flash driver.
 * Input      : cb_event - callback for events
 * Return     : Execution status
 **********************************************************************************************************************/
int32_t internal_flash_Initialize(ARM_Flash_SignalEvent_t cb_event)
{
    /* In respect to other CMSIS drivers which are meant to be used by one client with one cb_event,
       the internal flash is blocking and we want it to be multi client. Because we still want this
       driver to be initialize when it's needed (and not on the 'main' function) then we need to
       allow multiple initializations (so every client will be able to call 'initialize' independently */
    if (this.state != INTFLASH_UNINITIALIZED)
    {
        return ARM_DRIVER_OK;
    }

    this.flash_mutex_id = osMutexNew(&(internal_flash_conf.mutex_attr));

    this.callback = cb_event;

    this.status.busy = false;

    this.status.error = false;

#if defined(STM32L4)
    faults_nmi_add_handler(nmi_handler, NULL);
#endif /* STM32L4 */

    this.state = INTFLASH_INITIALIZED;

    return ARM_DRIVER_OK;
}

/***********************************************************************************************************************
 * Description: Does nothing - unsupported.
 * Return     : Execution status
 **********************************************************************************************************************/
int32_t internal_flash_Uninitialize(void)
{
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/***********************************************************************************************************************
 * Description: Controls the power state of the internal flash.
 * Input      : state - new power state
 * Return     : Execution status
 **********************************************************************************************************************/
int32_t internal_flash_PowerControl(ARM_POWER_STATE state)
{
    proj_assert(this.state != INTFLASH_UNINITIALIZED);

    switch (state)
    {
        case ARM_POWER_OFF:
        {
            /* Return ARM_DRIVER_OK if driver is already powered to the desired state */
            if (this.state == INTFLASH_INITIALIZED)
            {
                return ARM_DRIVER_OK;
            }

            nvic_enable_irq(FLASH_IRQn, false);

            this.state = INTFLASH_INITIALIZED;

            return ARM_DRIVER_OK;
        }
        case ARM_POWER_LOW:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
        case ARM_POWER_FULL:
        {
            /* Return ARM_DRIVER_OK if driver is already powered to the desired state */
            if (this.state == INTFLASH_POWERED)
            {
                return ARM_DRIVER_OK;
            }

            this.state = INTFLASH_POWERED;

            nvic_enable_irq(FLASH_IRQn, true);

            return ARM_DRIVER_OK;
        }
        default:
        {
            return ARM_DRIVER_ERROR_PARAMETER;
        }
    }
}

/***********************************************************************************************************************
 * Description: Read data from the internal flash.
 * Input      : addr - address from which to read data
 *              data - pointer to buffer into which to read data, it should be at least `cnt` bytes long
 *              cnt  - how many bytes to read from flash
 * Return     : Number of bytes read
 **********************************************************************************************************************/
int32_t internal_flash_ReadData(uint32_t addr, void* data, uint32_t cnt)
{
    proj_assert((void*)addr != NULL);
    proj_assert(data != NULL);
    proj_assert(cnt > 0);

    proj_assert(this.state == INTFLASH_POWERED);

    /* Wait on mutex until available */
    if (osOK != osMutexAcquire(this.flash_mutex_id, osWaitForever))
    {
        proj_exception();
    }

    this.status.busy = true;

    this.status.error = false;

    memcpy(data, (void*)addr, cnt);

    uint32_t read_bytes_count = cnt;

#if defined(STM32L4)
    /* Get defected double word address if there was a defected one */
    void* defect_address = get_eccd_error_address();

    /* Counter that will enable us to limit the number of DW zero-writing (fixing) tries */
    uint8_t ecc_error_try_count = 0;

    /* If the ECC has detected read error try to zero the defective DW up to 2 times */
    while (defect_address != NULL && ecc_error_try_count < 2)
    {
        /* Unlock Flash */
        HAL_FLASH_Unlock();

        /* Write zeros to the defective double word */
        HAL_StatusTypeDef result =
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)defect_address, (uint64_t)0);

        /* Lock Flash */
        HAL_FLASH_Lock();

        /* set error return value and break in case of failure */
        if (result != HAL_OK)
        {
            /* Set flash driver error flag true */
            this.status.error = true;

            /* Return no bytes read in order to signal error */
            read_bytes_count = 0;

            /* Break from the loop on production */
            break;
        }

        /* Increase the try count */
        ecc_error_try_count++;

        /* Try to read from flash again */
        memcpy(data, (void*)addr, cnt);

        /* Get defected double word address if there was a defected one */
        defect_address = get_eccd_error_address();
    }
#endif /* STM32L4 */

    this.status.busy = false;

    if (osMutexRelease(this.flash_mutex_id) != osOK)
    {
        proj_exception();
    }

    return read_bytes_count;
}

/***********************************************************************************************************************
 * Description: Write data from buffer to flash.
 * Input      : addr - address to which to write the data
 *              data - pointer to buffer from which to read data, it should be at least `cnt` bytes long
 *              cnt  - how many bytes to write to flash
 * Return     : Number of bytes written to flash
 **********************************************************************************************************************/
int32_t internal_flash_ProgramData(uint32_t addr, const void* data, uint32_t cnt)
{
    /* Assert that the data address is a multiple of the programming unit */
    proj_assert((addr & (PROGRAM_UNIT - 1)) == 0);
    /* Assert that the data size is a multiple of the programming unit */
    proj_assert((cnt & (PROGRAM_UNIT - 1)) == 0);
    /* Assert that the input pointer is 4B aligned - FIXME: is this necessary? */
    proj_assert(((uint32_t)data & 0x3) == 0);

    proj_assert(this.state == INTFLASH_POWERED);

    if (osMutexAcquire(this.flash_mutex_id, osWaitForever) != osOK)
    {
        proj_exception();
    }

    this.status.busy = true;

    this.status.error = false;

    /* This casting is for pointer arithmetics when writing the data several codes lines below */
    program_unit_t* p_data_unit = (program_unit_t*)data;

    uint32_t programmed_bytes_count = 0;

    HAL_FLASH_Unlock();

    /* Write to Flash using blocking mode */
    while (programmed_bytes_count < cnt)
    {
        /* Write program unit to the correct position according to the address given  */
        HAL_StatusTypeDef result = HAL_FLASH_Program(PROGRAM_TYPE, addr, *(p_data_unit++));

        /* set error return value and break in case of failure */
        if (result != HAL_OK)
        {
            /* Set flash driver error flag true */
            this.status.error = true;

            /* Exit the writing loop */
            break;
        }

        /* Update write counters */
        addr += PROGRAM_UNIT;
        programmed_bytes_count += PROGRAM_UNIT;
    }

    HAL_FLASH_Lock();

    this.status.busy = false;

    if (osMutexRelease(this.flash_mutex_id) != osOK)
    {
        proj_exception();
    }

    return programmed_bytes_count;
}

/***********************************************************************************************************************
 * Description: Erases a sector within the flash.
 * Input      : addr - sector address
 * Return     : Arm driver result
 **********************************************************************************************************************/
int32_t internal_flash_EraseSector(uint32_t addr)
{
    proj_assert((void*)addr != NULL);

    proj_assert(this.state == INTFLASH_POWERED);

    if (osOK != osMutexAcquire(this.flash_mutex_id, osWaitForever))
    {
        proj_exception();
    }
#if defined(STM32F4)
    int32_t sector = address_to_sector(addr);

    if (SECTOR_NOT_FOUND == sector)
    {
        proj_exception();
    }
#endif
    this.status.busy = true;

    this.status.error = false;

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase_init = {
#if defined(STM32F4)
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Banks        = FLASH_BANK_1,
        .Sector       = sector,
        .NbSectors    = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
#else
        .TypeErase = FLASH_TYPEERASE_PAGES,

#if defined(STM32L4) || defined(STM32WL)
        .Page      = get_page_num(addr),
#elif defined(STM32F1)
        .PageAddress = addr,
#endif
        .NbPages   = 1,
#if defined(FLASH_BANK_1) || defined(FLASH_BANK_2)
        .Banks     = get_bank(addr),
#endif
#endif
    };

    uint32_t page_error;

    HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&erase_init, &page_error);

    int32_t ret = ARM_DRIVER_OK;

    if (result != HAL_OK)
    {
        this.status.error = true;

        ret = ARM_DRIVER_ERROR;
    }

    /* If the ST HAL returned erase error */
    if (this.state == INTFLASH_ERROR)
    {
        this.status.error = true;

        ret = ARM_DRIVER_ERROR;

        /* Change internal state machine back to powered */
        this.state = INTFLASH_POWERED;
    }

    this.status.busy = false;

    HAL_FLASH_Lock();

    if (osMutexRelease(this.flash_mutex_id) != osOK)
    {
        proj_exception();
    }

    return ret;
}

/***********************************************************************************************************************
 * Description: Does nothing - unsupported .
 * Return     : ARM_DRIVER_ERROR_UNSUPPORTED
 **********************************************************************************************************************/
int32_t internal_flash_EraseChip(void)
{
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/***********************************************************************************************************************
 * Description: Returns status of the internal flash/driver.
 * Return     : ARM_FLASH_STATUS - Flash status
 **********************************************************************************************************************/
ARM_FLASH_STATUS internal_flash_GetStatus(void)
{
    return this.status;
}

/***********************************************************************************************************************
 * Description: Returns flash info
 * Return     : ARM_FLASH_INFO - Pointer to Flash information
 **********************************************************************************************************************/
ARM_FLASH_INFO* internal_flash_GetInfo(void)
{
    return &(internal_flash_conf.info);
}

/**********************************************************************************************************************/
/* Private function implementation                                                                                    */
/**********************************************************************************************************************/

#if defined(STM32L4) || defined(STM32WL)
/***********************************************************************************************************************
 * Description: Calculates in which page the address is located.
 * Input      : addr - address whose page to calculate
 * Return     : page number
 **********************************************************************************************************************/
static uint32_t get_page_num(uint32_t addr)
{
    uint32_t page = 0;

    if (addr < (FLASH_BASE + BANK_SIZE))
    {
        /* Bank 1 */
        page = (addr - FLASH_BASE) / FLASH_PAGE_SIZE;
    }
    else
    {
        /* Bank 2 */
        page = (addr - (FLASH_BASE + BANK_SIZE)) / FLASH_PAGE_SIZE;
    }

    return page;
}
#endif /* STM32L4 || STM32WL */

#if (defined(FLASH_BANK_1) || defined(FLASH_BANK_2)) && !defined(STM32F4)
/***********************************************************************************************************************
 * Description: Calculates in which bank the address is located.
 * Input      : addr - address whose bank to calculate
 * Return     : bank number (FLASH_BANK_1 or FLASH_BANK_2)
 **********************************************************************************************************************/
static uint32_t get_bank(uint32_t addr)
{
#if defined(FLASH_BANK_2)
    uint32_t bank = 0;

    if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
    {
        /* No Bank swap */
        if (addr < (FLASH_BASE + FLASH_BANK_SIZE))
        {
            bank = FLASH_BANK_1;
        }
        else
        {
            bank = FLASH_BANK_2;
        }
    }
    else
    {
        /* Bank swap */
        if (addr < (FLASH_BASE + FLASH_BANK_SIZE))
        {
            bank = FLASH_BANK_2;
        }
        else
        {
            bank = FLASH_BANK_1;
        }
    }

    return bank;
#else
    ARGUMENT_UNUSED(addr);
    return FLASH_BANK_1;
#endif /* FLASH_BANK_2 */
}
#endif /* FLASH_BANK_1 || FLASH_BANK_2 */

#if defined(STM32L4)
/***********************************************************************************************************************
 * Description: NMI Handler - checks if an ECCD error happened and saves the error address.
 * Input      : arg - unused
 * Return     : Whether the NMI source can/was be handled by this handler.
 **********************************************************************************************************************/
static bool nmi_handler(void* arg)
{
    ARGUMENT_UNUSED(arg);

    /* In case that we got ECC detection error from internal flash */
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_ECCD))
    {
        /* Clear the error flags to prevent multiple NMIs */
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCD | FLASH_ECCR_ECCC);

        /* Calculate the defective address according to the internal flash's ECCR register (use FLASH_ECCR_BK_ECC if
         * relevant to jump to the wanted flash bank) */
#if defined(FLASH_ECCR_BK_ECC)
        this.eccd_addr = (void*)(FLASH_BASE + ((FLASH->ECCR & FLASH_ECCR_BK_ECC) == FLASH_ECCR_BK_ECC) * FLASH_BANK_SIZE
                                 + (FLASH->ECCR & FLASH_ECCR_ADDR_ECC));
#else
        this.eccd_addr = (void*)(FLASH_BASE + (FLASH->ECCR & FLASH_ECCR_ADDR_ECC));
#endif
        uint32_t addr = (uint32_t)this.eccd_addr;
        LOG_ERROR("ECCD error occured in 0x{0:X8}", addr);

        return true;
    }

    return false;
}

/***********************************************************************************************************************
 * Description: Returns address of last ECCD error
 * Return     : ECCD address
 **********************************************************************************************************************/
static void* get_eccd_error_address(void)
{
    /* Save the defected address */
    void* ret = this.eccd_addr;

    /* Zero the defected address global */
    this.eccd_addr = NULL;

    /* Return the defected address */
    return ret;
}
#endif /* STM32L4 */

/* -------------------------------------------------- IRQ Handlers -------------------------------------------------- */

/***********************************************************************************************************************
 * Description: Internal Flash IRQ handler, calls HAL for further processing.
 **********************************************************************************************************************/
void FLASH_IRQHandler()
{
    /* Call HAL FLASH IRQHandler to handle the interrupt */
    HAL_FLASH_IRQHandler();
}

/***********************************************************************************************************************
 * Description: Called by the HAL when an operation fails, sets state to error and signals the rest of the module.
 * Input      : ret_val - unused
 **********************************************************************************************************************/
void HAL_FLASH_OperationErrorCallback(uint32_t ret_val)
{
    ARGUMENT_UNUSED(ret_val);

    /* Change internal state machine to error */
    this.state = INTFLASH_ERROR;
}