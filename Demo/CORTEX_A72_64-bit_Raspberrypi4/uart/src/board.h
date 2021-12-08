
#ifndef _BOARD_H_
#define _BOARD_H_
/* GIC-400 registers */
#define GIC_BASE	(0xFF840000UL)
#define GICD_BASE	(GIC_BASE + 0x00001000UL)
#define GICC_BASE	(GIC_BASE + 0x00002000UL)

/* The number of IRQs on BCM2711 */
#define MAX_NUM_IRQS (224U)

/* IRQ number */
#define VC_PERIPHERAL_IRQ 96
#define IRQ_VTIMER (27)
#define IRQ_GPIO0 (VC_PERIPHERAL_IRQ + 49)
#define IRQ_GPIO1 (VC_PERIPHERAL_IRQ + 50)
#define IRQ_GPIO2 (VC_PERIPHERAL_IRQ + 51)
#define IRQ_GPIO3 (VC_PERIPHERAL_IRQ + 52)
#define IRQ_VC_UART (VC_PERIPHERAL_IRQ + 57) // 153 = 96(base das IRQ VC peripheral) + 57(offset da interrupção uart2)

#endif //_BOARD_H_