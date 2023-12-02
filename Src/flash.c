
#include "flash.h"

#include "led.h"
#include "stm32f0xx_hal.h"
#include "timers.h"

#define FLASH_USER_START_ADDR ADDR_FLASH_PAGE_16                 /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR ADDR_FLASH_PAGE_63 + FLASH_PAGE_SIZE /* End @ of user Flash area */

#define __FLASH_GET_FLAG(__FLAG__) (((FLASH->SR) & (__FLAG__)) == (__FLAG__))
#define __FLASH_CLEAR_FLAG(__FLAG__) ((FLASH->SR) = (__FLAG__))

extern volatile int enable_counter;
extern volatile unsigned tickcounter;

My_StatusTypeDef FLASHAPI My_FLASH_Unlock(void)
{
    My_StatusTypeDef status = My_OK;

    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
    {
        /* Authorize the FLASH Registers access */
        WRITE_REG(FLASH->KEYR, FLASH_KEY1);
        WRITE_REG(FLASH->KEYR, FLASH_KEY2);

        /* Verify Flash is unlocked */
        if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
        {
            status = My_ERROR;
        }
    }

    return status;
}

static My_StatusTypeDef FLASHAPI WaitForLastFlashOperation(uint32_t Timeout)
{
    /* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
       Even if the FLASH operation fails, the BUSY flag will be reset and an error
       flag will be set */

    uint32_t tickstart = GetTick();

    while (__FLASH_GET_FLAG(FLASH_FLAG_BSY))
    {
        if ((Timeout == 0U) || ((GetTick() - tickstart) > Timeout))
        {
            return My_TIMEOUT;
        }
    }

    /* Check FLASH End of Operation flag  */
    if (__FLASH_GET_FLAG(FLASH_FLAG_EOP))
    {
        /* Clear FLASH End of Operation pending bit */
        __FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
    }

    if (__FLASH_GET_FLAG(FLASH_FLAG_WRPERR) || __FLASH_GET_FLAG(FLASH_FLAG_PGERR))
    {
        return My_ERROR;
    }

    /* There is no error flag set */
    return My_OK;
}

static void NOINLINE FLASHAPI My_PageErase(uint32_t PageAddress)
{
    /* Proceed to erase the page */
    // My_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
    SET_BIT(FLASH->CR, FLASH_CR_PER);

    // My_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
    WRITE_REG(FLASH->AR, PageAddress);

    // My_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
    SET_BIT(FLASH->CR, FLASH_CR_STRT);
}

static My_StatusTypeDef NOINLINE FLASHAPI EraseFlash(My_EraseInitTypeDef *pEraseInit, uint32_t *PageError)
{
    My_StatusTypeDef status = My_ERROR;

    /* Page Erase requested on address located on bank1 */
    /* Wait for last operation to be completed */
    if (HAL_OK == WaitForLastFlashOperation((uint32_t)FLASH_TIMEOUT_VALUE))
    {
        /*Initialization of PageError variable*/
        *PageError = 0xFFFFFFFFU;

        /* Erase page by page to be done*/
        for (uint32_t address = pEraseInit->PageAddress;
             address < ((pEraseInit->NbPages * FLASH_PAGE_SIZE) + pEraseInit->PageAddress); address += FLASH_PAGE_SIZE)
        {
            My_PageErase(address);

            /* Wait for last operation to be completed */
            status = WaitForLastFlashOperation((uint32_t)FLASH_TIMEOUT_VALUE);

            /* If the erase operation is completed, disable the PER Bit */
            CLEAR_BIT(FLASH->CR, FLASH_CR_PER);

            if (status != My_OK)
            {
                /* In case of error, stop erase procedure and return the faulty address */
                *PageError = address;
                break;
            }
        }
    }

    return status;
}

My_StatusTypeDef NOINLINE RAMFUNC DoErase(void)
{
    /* Unlock the Flash to enable the flash control register access *************/
    My_StatusTypeDef status = My_FLASH_Unlock();
    if (My_OK != status) return status;

    /* Erase the user Flash area
        (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
    /* Fill EraseInit structure*/
    My_EraseInitTypeDef EraseInitStruct;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
    EraseInitStruct.NbPages = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;

    My_LED_Off(LED6);

    uint32_t PageError = 0;
    status = EraseFlash(&EraseInitStruct, &PageError);

    return status;
}
