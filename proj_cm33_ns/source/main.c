/*******************************************************************************
* File Name       : main.c
*
* Description     : This source file contains the main routine for non-secure
*                   application in the CM33 CPU for SPI DMA code example.
*                   This example demonstrates SPI communication between two SCB  
*                   of PSOC Edge device (one configured as Controller and another
*                   as target) blocks. Polling method is used on the controller
*                   side to confirm data transfer completion.
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2023-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
#include "spi_controller.h"
#include "cybsp.h"
#include "interface.h"
#include "spi_dma.h"
#include "spi_target.h"
#include "cybsp_types.h"

/*******************************************************************************
* Macro definitions
*******************************************************************************/

/* Delay between successive SPI Controller command transmissions */
#define CMD_DELAY_MS                      (1000U)

/* Number of elements in the transmit and receive buffer */
/* There are three elements - one for head, one for command and one for tail */
#define NUMBER_OF_ELEMENTS                (3UL)
#define SIZE_OF_ELEMENT                   (4UL)

/* The timeout value in microsecond used to wait for core to be booted */
#define CM55_BOOT_WAIT_TIME_USEC          (10U)
#define RESET_VALUE                       (0U)

/* App boot address for CM55 project */
#define CM55_APP_BOOT_ADDR                (CYMEM_CM33_0_m55_nvm_START + \
                                           CYBSP_MCUBOOT_HEADER_SIZE)

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: update_led
********************************************************************************
* Summary   : This function updates the LED based on the command received by
*             the SPI target from controller.
*
* Parameters: (uint32_t) led_cmd - command to turn LED ON or OFF
*
* Return    : None
*
*******************************************************************************/
static void update_led(uint32_t led_cmd)
{
    switch (led_cmd)
    {
        case CYBSP_LED_STATE_ON:
            /* Turn ON the LED */
            Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN,
                    CYBSP_LED_STATE_ON);
            break;

        case CYBSP_LED_STATE_OFF:
            /* Turn OFF the LED */
            Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN,
                    CYBSP_LED_STATE_OFF);
            break;

        default:
            break;
    }
}

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary   : 1.  Sets up two SPI blocks - one as controller and another as target
*             2.  SPI controller sends commands to the target to turn LED ON or
*                 OFF, every one second.
*             3.  SPI controller polls for the data transfer completion.
*             4.  When the required number of bytes is transferred, data
*             received by the target is checked and LED is turned ON or OFF
*             based on the received command.
*
* Parameters: None
*
* Return:     Int
*
*******************************************************************************/
int main(void)
{
    uint32_t status = RESET_VALUE;
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

#if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_CONTROLLER))

    /* Buffer to hold command packet to be sent to the target by the controller */
    uint32_t  tx_buffer[NUMBER_OF_ELEMENTS]={RESET_VALUE};

    /* Local command variable */
    uint32_t cmd = CYBSP_LED_STATE_OFF;

    /* Initialize the SPI Controller */
    status = init_controller();

    if (INIT_FAILURE == status)
    {
        handle_app_error();
    }

    /* Configure Tx DMA */
    status = configure_tx_dma(tx_buffer);

    if (INIT_FAILURE == status)
    {
        handle_app_error();
    }

#endif /* #if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_CONTROLLER)) */

#if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_TARGET))

    /* Buffer to save the received data by the target */
    uint32_t  rx_buffer[NUMBER_OF_ELEMENTS]={RESET_VALUE};

    /* Initialize the SPI Target */
    status = init_target();

    if (INIT_FAILURE == status)
    {
        handle_app_error();
    }

    /* Configure Rx DMA */
    status = configure_rx_dma(rx_buffer);

    if (INIT_FAILURE == status)
    {
        handle_app_error();
    }

#endif /* #if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_TARGET)) */

    /* Enable global interrupt */
    __enable_irq();

   /* Enable CM55. CM55_APP_BOOT_ADDR must be updated if CM55
    * memory layout is changed.
    */
    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);
 
    for (;;)
    {

#if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_CONTROLLER))

        /* Toggle the current LED state */
        cmd = (cmd == CYBSP_LED_STATE_ON) ? CYBSP_LED_STATE_OFF :
               CYBSP_LED_STATE_ON;

        /* Form the command packet */
        tx_buffer[PACKET_SOP_POS] = PACKET_SOP;
        tx_buffer[PACKET_CMD_POS] = cmd;
        tx_buffer[PACKET_EOP_POS] = PACKET_EOP;

       /* Pass the command packet to the controller along with the number of
        * bytes to be sent to the target.
        */
        send_packet();

        /* Wait until controller complete the transfer */
        while (false == tx_dma_done) {}
        tx_dma_done = false;

        /* Give delay before initiating the next command */
        Cy_SysLib_Delay(CMD_DELAY_MS);

#endif /* #if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_CONTROLLER)) */

       /* The below code is for target function. It is implemented in this same
        * code example so that the controller function can be tested without the
        * need of one more kit.
        */
#if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_TARGET))

        if(rx_dma_done)
        {
            /* Check start and end of packet markers */
            if (( PACKET_SOP == rx_buffer[PACKET_SOP_POS]) && 
            ( PACKET_EOP == rx_buffer[PACKET_EOP_POS]))
            {
                /* Communication succeeded. Update the LED. */
                update_led(rx_buffer[PACKET_CMD_POS]);
            }
            else
            {
               /* Data was not received correctly and hence
                * Communication failed
                */
                handle_app_error();
            }
            
            rx_dma_done = false;

            /* Get the bytes received by the target */
            receive_packet();
        }

#endif /* #if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_TARGET)) */
    }
}


/* [] END OF FILE */
