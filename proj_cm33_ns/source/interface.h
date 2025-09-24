/*******************************************************************************
* File Name       : interface.h
*
* Description     : This file contains all the macros related to configuration
*                   and communication protocol used by application.
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
#ifndef INTERFACE_H_
#define INTERFACE_H_

/* This code example requires two SPI ports, one as controller and the other one
 * as target. Set the SPI_MODE macro depending on your use case. You can
 * configure the kit in SPI_MODE_CONTROLLER or SPI_MODE_TARGET or SPI_MODE_BOTH.
 * See README.md to know more on the kit specific configuration.
 */
#define SPI_MODE_BOTH           (0U)
#define SPI_MODE_CONTROLLER     (1U)
#define SPI_MODE_TARGET         (2U)

#define SPI_MODE SPI_MODE_BOTH

/* Initialization status */
#define INIT_SUCCESS            (0U)
#define INIT_FAILURE            (1U)

/* Communication status */
#define TRANSFER_COMPLETE       (0U)
#define TRANSFER_FAILURE        (1U)
#define TRANSFER_IN_PROGRESS    (2U)
#define IDLE                    (3U)

/* TX Packet Head and Tail */
#define PACKET_SOP              (0x01UL)
#define PACKET_EOP              (0x17UL)

/* Element index in the packet */
#define PACKET_SOP_POS          (0UL)
#define PACKET_CMD_POS          (1UL)
#define PACKET_EOP_POS          (2UL)

#endif /* INTERFACE_H_ */

/* [] END OF FILE */
