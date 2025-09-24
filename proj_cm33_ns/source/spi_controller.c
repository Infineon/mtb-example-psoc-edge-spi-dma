/*******************************************************************************
* File Name       : spi_controller.c
*
* Description     : This file contains function definitions for SPI Controller.
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
#include "interface.h"
#include "spi_dma.h"

#if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_CONTROLLER))
/*******************************************************************************
* Function Name: init_controller
********************************************************************************
* Summary   : This function initializes the SPI Controller based on the
*             configuration done in design.modus file.
*
* Parameters: None
*
* Return    : (uint32_t) INIT_SUCCESS or INIT_FAILURE
*
*******************************************************************************/
uint32_t init_controller(void)
{
    uint32_t status = INIT_SUCCESS;
    cy_en_scb_spi_status_t init_status;

    /* Configure SPI block */
    init_status = Cy_SCB_SPI_Init(CYBSP_SPI_CONTROLLER_HW,
            &CYBSP_SPI_CONTROLLER_config, NULL);

    /* If the initialization fails, update status */
    if ( CY_SCB_SPI_SUCCESS != init_status )
    {
        status = INIT_FAILURE;
    }

    if (status == INIT_SUCCESS)
    {
        /* Set active target select to line 0 */
        Cy_SCB_SPI_SetActiveSlaveSelect(CYBSP_SPI_CONTROLLER_HW,
                CY_SCB_SPI_SLAVE_SELECT0);

        /* Enable SPI Controller block. */
        Cy_SCB_SPI_Enable(CYBSP_SPI_CONTROLLER_HW);
    }

    return status;
}

/*******************************************************************************
* Function Name: send_packet
********************************************************************************
* Summary   : This function transfers data from txBuffer to
*             CYBSP_SPI_CONTROLLER TX-FIFO. The below function enables
*             channel and DMA block to start descriptor execution process for
*             CYBSP_SPI_CONTROLLER_TX_DMA.
*******************************************************************************/
void send_packet(void)
{
   /* Enable DMA channel to transfer 12 bytes of data from txBuffer into
    * CYBSP_SPI_CONTROLLER TX-FIFO */
    Cy_DMA_Channel_Enable(CYBSP_SPI_CONTROLLER_TX_DMA_HW,
            CYBSP_SPI_CONTROLLER_TX_DMA_CHANNEL);
}

#endif /* #if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_CONTROLLER)) */
