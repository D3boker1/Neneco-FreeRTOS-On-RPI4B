/**
 * @file board.h
 * @author Francisco Marques (fmarques_00@protonmail.com)
 * @brief Contains some information about the RPI4 model B.
 * @version 0.1
 * @date 2022-01-13
 * 
 * @copyright Copyright (c) 2022
 * 
 * This file contains generic information about the the RPI4 model B board.
 * The information available are the high hierarchy register devices. This is, the 
 * devices register base. 
 * In futures version all the registers will be migrated to here.
 * 
 */

#ifndef _BOARD_H_
#define _BOARD_H_

/**< GIC-400 registers */
#define GIC_BASE	(0xFF840000UL)
#define GICD_BASE	(GIC_BASE + 0x00001000UL)
#define GICC_BASE	(GIC_BASE + 0x00002000UL)

/**< The number of IRQs on BCM2711 */
#define MAX_NUM_IRQS (224U)

#define core_clock 500000000 /**<  the core count frequency is 500 MHz*/

/**< IRQ number */
#define VC_PERIPHERAL_IRQ 96
#define IRQ_VTIMER (27)
#define IRQ_GPIO0 (VC_PERIPHERAL_IRQ + 49)
#define IRQ_GPIO1 (VC_PERIPHERAL_IRQ + 50)
#define IRQ_GPIO2 (VC_PERIPHERAL_IRQ + 51)
#define IRQ_GPIO3 (VC_PERIPHERAL_IRQ + 52)
#define IRQ_SPI (VC_PERIPHERAL_IRQ + 54)
#define IRQ_VC_UART (VC_PERIPHERAL_IRQ + 57) /**< 153 = 96(IRQ VC peripheral base value) + 57( UART2 interruption offset)*/


#define ARM_IO_BASE		0xFE000000

#endif //_BOARD_H_