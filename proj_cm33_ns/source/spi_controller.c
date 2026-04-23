/*******************************************************************************
* File Name       : spi_controller.c
*
* Description     : This file contains function definitions for SPI Controller.
*
* Related Document: See README.md
*
********************************************************************************
* (c) 2023-2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
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
