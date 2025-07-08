/***************************************************************************//**
* \file main.c
* \version 1.0
*
* \brief Main source file of the FX2G3 device Hello World application.
*
*******************************************************************************
* \copyright
* (c) (2025), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include "cy_pdl.h"
#include <string.h>
#include "app_version.h"
#include "cy_debug.h"
#include "cybsp.h"

/* Debug log related initialization */
#define LOGGING_SCB             (SCB4)
#define LOGGING_SCB_IDX         (4)
#define DEBUG_LEVEL             (3u)

#define LOGBUF_SIZE (1024u)

uint8_t logBuff[LOGBUF_SIZE];
cy_stc_debug_config_t dbgCfg = {
    .pBuffer         = logBuff,
    .traceLvl        = DEBUG_LEVEL,
    .bufSize         = LOGBUF_SIZE,
#if USBFS_LOGS_ENABLE
    .dbgIntfce       = CY_DEBUG_INTFCE_USBFS_CDC,
#else
    .dbgIntfce       = CY_DEBUG_INTFCE_UART_SCB4,
#endif
    .printNow        = true
};

/**
 * \name Cy_PrintVersionInfo
 * \brief Function to print version information to UART console.
 * \param type Type of version string.
 * \param typeLen Length of version type string.
 * \param vMajor Major version number (0 - 99)
 * \param vMinor Minor version number (0 - 99)
 * \param vPatch Patch version number (0 - 99)
 * \param vBuild Build number (0 - 9999)
 * \retval None
 */
void Cy_PrintVersionInfo (const char *type, uint8_t typeLen,
                       uint8_t vMajor, uint8_t vMinor, uint8_t vPatch, uint16_t vBuild)
{
    DBG_APP_INFO("%s%d.%d.%d.%d\r\n", type, vMajor, vMinor, vPatch, vBuild);
}

/**
 * \name Cy_Fx2g3_InitPeripheralClocks
 * \brief Function used to enable clocks to different peripherals on the FX2G3 device.
 * \param adcClkEnable Whether to enable clock to the ADC in the USBSS block.
 * \param usbfsClkEnable Whether to enable bus reset detect clock input to the USBFS block.
 * \retval None
 */
void Cy_Fx2g3_InitPeripheralClocks (
        bool adcClkEnable,
        bool usbfsClkEnable)
{
    if (adcClkEnable) {
        /* Divide PERI clock at 75 MHz by 75 to get 1 MHz clock using 16-bit divider #1. */
        Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 1, 74);
        Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 1);
        Cy_SysLib_DelayUs(10U);
        Cy_SysClk_PeriphAssignDivider(PCLK_LVDS2USB32SS_CLOCK_SAR, CY_SYSCLK_DIV_16_BIT, 1);
    }

    if (usbfsClkEnable) {
        /* Divide PERI clock at 75 MHz by 750 to get 100 KHz clock using 16-bit divider #2. */
        Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 2, 749);
        Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 2);
        Cy_SysLib_DelayUs(10U);
        Cy_SysClk_PeriphAssignDivider(PCLK_USB_CLOCK_DEV_BRS, CY_SYSCLK_DIV_16_BIT, 2);
    }
}

/**
 * \name PeripheralInit
 * \brief Initialize peripherals used by the application.
 * \retval None
 */
void PeripheralInit (void)
{
    cy_stc_gpio_pin_config_t pinCfg;

    /* Do all the relevant clock configuration */
    Cy_Fx2g3_InitPeripheralClocks(false, true);

    /* Configure the GPIOs used for firmware activity indication. */
    memset ((void *)&pinCfg, 0, sizeof(pinCfg));
    pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
    pinCfg.hsiom     = P4_3_GPIO;
    Cy_GPIO_Pin_Init(P4_3_PORT, P4_3_PIN, &pinCfg);
    pinCfg.hsiom     = P4_4_GPIO;
    Cy_GPIO_Pin_Init(P4_4_PORT, P4_4_PIN, &pinCfg);

}

/**
 * \name main
 * \brief Entry to the application.
 * \retval Does not return.
 */
int main(void)
{
    const char start_string[] = "********** FX2G3:Hello World Application**********\r\n";
    uint32_t loopCount = 0;

    /* Initialize the PDL driver library and set the clock variables. */
    /* Note: All FX devices,  share a common configuration structure. */
    Cy_PDL_Init(&cy_deviceIpBlockCfgFX3G2);

    /* Initialize the device and board peripherals */
    cybsp_init();

    /* Perform PDL and UART initialization as the first step. */
    PeripheralInit();

    /* Unlock and then disable the watchdog. */
    Cy_WDT_Unlock();
    Cy_WDT_Disable();

    /*
     * If logging is done through the USBFS port, ISR execution is required in this application.
     * Set BASEPRI value to 0 to ensure all exceptions can run and then enable interrupts.
     */
#if (CY_CPU_CORTEX_M4)
    __set_BASEPRI(0);
#endif /* CY_CPU_CORTEX_M4 */

    __enable_irq();

#if !USBFS_LOGS_ENABLE
    /* Initialize the UART for logging. */
    InitUart(LOGGING_SCB_IDX);
#endif /* USBFS_LOGS_ENABLE */

    /*
     * Initialize the logger module. We are using a blocking print option which will
     * output the messages immediately without buffering.
     */
    Cy_Debug_LogInit(&dbgCfg);
    Cy_SysLib_Delay(500);

    /* Print start-up string using SCB0 - UART. */
    DBG_APP_INFO("%s", start_string);

    /* Print application and USBD stack version information. */
    Cy_PrintVersionInfo("APP_VERSION: ", 13, APP_VERSION_MAJOR, APP_VERSION_MINOR,
            APP_VERSION_PATCH, APP_VERSION_BUILD);

    while (1) {
        /* Toggle P4.3, print UART message, toggle P4.4 and then wait for 1 second. */
        Cy_GPIO_Inv(P4_3_PORT, P4_3_PIN);
        DBG_APP_INFO("Completed %d loops\r\n", loopCount++);
        Cy_GPIO_Inv(P4_4_PORT, P4_4_PIN);
        Cy_SysLib_Delay(1000);
    }

    /* Return statement will not be hit. */
    return 0;
}

/* [] END OF FILE */
