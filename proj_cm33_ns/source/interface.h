/*******************************************************************************
* File Name       : interface.h
*
* Description     : This file contains all the macros related to configuration
*                   and communication protocol used by application.
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
