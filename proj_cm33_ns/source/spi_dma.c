/*******************************************************************************
* File Name       : spi_dma.c
*
* Description     : This file contains function definitions for DMA operation.
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
#include "spi_dma.h"
#include "interface.h"

/*******************************************************************************
* Macro definitions
*******************************************************************************/

/* Interrupt priority for CYBSP_SPI_TARGET_RX_DMA */
#define CYBSP_SPI_TARGET_RX_DMA_INTERRUPT_PRIORITY (7U)

/* Interrupt priority for CYBSP_SPI_CONTROLLER_TX_DMA */
#define CYBSP_SPI_CONTROLLER_TX_DMA_INTERRUPT_PRIORITY (7U)

/* variable to check the rx dma transection status */
#if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_TARGET))
volatile bool rx_dma_done = false;
#endif

/* variable to check the tx dma transection status */
#if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_CONTROLLER))
volatile bool tx_dma_done = false;

/*******************************************************************************
* Function Name: configure_tx_dma
********************************************************************************
* Summary   :  This function configure the transmit DMA block 
*
* Parameters:   tx_buffer
*
* Return    :  (uint32_t) INIT_SUCCESS or INIT_FAILURE
*
*******************************************************************************/
uint32_t configure_tx_dma(uint32_t* tx_buffer)
{
    uint32_t status = INIT_SUCCESS;
    cy_en_dma_status_t dma_init_status;
    const cy_stc_sysint_t intCYBSP_SPI_CONTROLLER_TX_DMA_cfg =
    {
        .intrSrc      = CYBSP_SPI_CONTROLLER_TX_DMA_IRQ,
        .intrPriority = CYBSP_SPI_CONTROLLER_TX_DMA_INTERRUPT_PRIORITY
    };

    /* Initialize descriptor */
    dma_init_status = Cy_DMA_Descriptor_Init
            (&CYBSP_SPI_CONTROLLER_TX_DMA_Descriptor_0,
            &CYBSP_SPI_CONTROLLER_TX_DMA_Descriptor_0_config);

    if (CY_DMA_SUCCESS != dma_init_status)
    {
        status = INIT_FAILURE;
    }

    if (INIT_SUCCESS == status)
    {
        dma_init_status = Cy_DMA_Channel_Init
                (CYBSP_SPI_CONTROLLER_TX_DMA_HW,
                CYBSP_SPI_CONTROLLER_TX_DMA_CHANNEL,
                &CYBSP_SPI_CONTROLLER_TX_DMA_channelConfig);

        if (CY_DMA_SUCCESS != dma_init_status)
        {
            status = INIT_FAILURE;
        }
    }

    if (INIT_SUCCESS == status)
    {
        /* Set source and destination for descriptor 1 */
        Cy_DMA_Descriptor_SetSrcAddress
                (&CYBSP_SPI_CONTROLLER_TX_DMA_Descriptor_0,
                (uint8_t *)tx_buffer);
        Cy_DMA_Descriptor_SetDstAddress
                (&CYBSP_SPI_CONTROLLER_TX_DMA_Descriptor_0,
                (void *)&CYBSP_SPI_CONTROLLER_HW->TX_FIFO_WR);

        /* Initialize and enable the interrupt from
         * CYBSP_SPI_CONTROLLER_TX_DMA */
        Cy_SysInt_Init(&intCYBSP_SPI_CONTROLLER_TX_DMA_cfg,
                (cy_israddress)tx_dma_complete);
        NVIC_EnableIRQ
        ((IRQn_Type)intCYBSP_SPI_CONTROLLER_TX_DMA_cfg.intrSrc);

        /* Enable DMA interrupt source. */
        Cy_DMA_Channel_SetInterruptMask(CYBSP_SPI_CONTROLLER_TX_DMA_HW,
                CYBSP_SPI_CONTROLLER_TX_DMA_CHANNEL,CY_DMA_INTR_MASK);

        /* Enable DMA block to start descriptor execution process */
        Cy_DMA_Enable(CYBSP_SPI_CONTROLLER_TX_DMA_HW);
    }

    return status;
}

/*******************************************************************************
* Function Name: configure_tx_dma
********************************************************************************
* Summary   : This function check the tx DMA status.
*******************************************************************************/
void tx_dma_complete(void)
{
     /* Check tx DMA status */
     if ((CY_DMA_INTR_CAUSE_COMPLETION    != Cy_DMA_Channel_GetStatus
        (CYBSP_SPI_CONTROLLER_TX_DMA_HW,
         CYBSP_SPI_CONTROLLER_TX_DMA_CHANNEL))&&
        (CY_DMA_INTR_CAUSE_CURR_PTR_NULL !=
         Cy_DMA_Channel_GetStatus(CYBSP_SPI_CONTROLLER_TX_DMA_HW,
         CYBSP_SPI_CONTROLLER_TX_DMA_CHANNEL)))
     {
         /* DMA error occurred while TX operations */
         handle_app_error();
     }

     tx_dma_done = true;

     /* Clear tx DMA interrupt */
     Cy_DMA_Channel_ClearInterrupt(CYBSP_SPI_CONTROLLER_TX_DMA_HW,
             CYBSP_SPI_CONTROLLER_TX_DMA_CHANNEL);
}
#endif /* #if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_CONTROLLER)) */

#if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_TARGET))
/*******************************************************************************
* Function Name: configure_tx_dma
********************************************************************************
* Summary   : This function configure the receive DMA block 
*
* Parameters: rx_buffer
*
* Return    : (uint32_t) INIT_SUCCESS or INIT_FAILURE
*
*******************************************************************************/
uint32_t configure_rx_dma(uint32_t* rx_buffer)
{
    uint32_t result = INIT_SUCCESS;
    cy_en_dma_status_t dma_init_status;
    const cy_stc_sysint_t intCYBSP_SPI_TARGET_RX_DMA_cfg =
    {
        .intrSrc      = CYBSP_SPI_TARGET_RX_DMA_IRQ,
        .intrPriority = CYBSP_SPI_TARGET_RX_DMA_INTERRUPT_PRIORITY
    };

    /* Initialize descriptor */
    dma_init_status = Cy_DMA_Descriptor_Init
            (&CYBSP_SPI_TARGET_RX_DMA_Descriptor_0,
             &CYBSP_SPI_TARGET_RX_DMA_Descriptor_0_config);

    if ( CY_DMA_SUCCESS != dma_init_status )
    {
        result = INIT_FAILURE;
    }
    else
    {
        /* Initialize channel */
        dma_init_status = Cy_DMA_Channel_Init(CYBSP_SPI_TARGET_RX_DMA_HW,
                CYBSP_SPI_TARGET_RX_DMA_CHANNEL,
                &CYBSP_SPI_TARGET_RX_DMA_channelConfig);

        if ( CY_DMA_SUCCESS != dma_init_status )
        {
            result = INIT_FAILURE;
        }
        else
        {
            /* Set source and destination for descriptor 1 */
            Cy_DMA_Descriptor_SetSrcAddress
                    (&CYBSP_SPI_TARGET_RX_DMA_Descriptor_0,
                    (void *)&CYBSP_SPI_TARGET_HW->RX_FIFO_RD);
            Cy_DMA_Descriptor_SetDstAddress
                    (&CYBSP_SPI_TARGET_RX_DMA_Descriptor_0,
                    (uint8_t *)rx_buffer);

            /* Initialize and enable the interrupt from
             * CYBSP_SPI_CONTROLLER_TX_DMA */
            Cy_SysInt_Init(&intCYBSP_SPI_TARGET_RX_DMA_cfg,
                    (cy_israddress)rx_dma_complete);
            NVIC_EnableIRQ((IRQn_Type)
                    intCYBSP_SPI_TARGET_RX_DMA_cfg.intrSrc);

            /* Enable DMA interrupt source. */
            Cy_DMA_Channel_SetInterruptMask(CYBSP_SPI_TARGET_RX_DMA_HW,
                    CYBSP_SPI_TARGET_RX_DMA_CHANNEL,
                    CY_DMA_INTR_MASK);
            /* Enable channel and DMA block to start descriptor
             * execution process */
            Cy_DMA_Channel_Enable(CYBSP_SPI_TARGET_RX_DMA_HW,
                    CYBSP_SPI_TARGET_RX_DMA_CHANNEL);
            Cy_DMA_Enable(CYBSP_SPI_TARGET_RX_DMA_HW);
        }
    }

    return result;
}

/*******************************************************************************
* Function Name: configure_tx_dma
********************************************************************************
* Summary   : This function check the rx DMA status.
*******************************************************************************/
void rx_dma_complete(void)
{
    if (CY_DMA_INTR_MASK == Cy_DMA_Channel_GetInterruptStatusMasked
            (CYBSP_SPI_TARGET_RX_DMA_HW,
                    CYBSP_SPI_TARGET_RX_DMA_CHANNEL))
    {
        /* Get the interrupt cause */
        cy_en_dma_intr_cause_t cause = Cy_DMA_Channel_GetStatus
                (CYBSP_SPI_TARGET_RX_DMA_HW,
                        CYBSP_SPI_TARGET_RX_DMA_CHANNEL);

        if (CY_DMA_INTR_CAUSE_COMPLETION != cause)
        {
            /* DMA error occurred while RX operations */
            handle_app_error();
        }
        else
        {
            rx_dma_done = true;
        }

        /* Clear the interrupt */
        Cy_DMA_Channel_ClearInterrupt(CYBSP_SPI_TARGET_RX_DMA_HW,
                CYBSP_SPI_TARGET_RX_DMA_CHANNEL);
    }
}

#endif /* #if ((SPI_MODE == SPI_MODE_BOTH) || (SPI_MODE == SPI_MODE_TARGET)) */


/* [] END OF FILE */
