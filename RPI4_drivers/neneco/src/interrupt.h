/**
 * @file interrupt.h
 * @author Francisco Marques (fmarques_00@protonmail.com)
 * @brief A simple implementation of gic-400 controller
 * @version 0.1
 * @date 2022-01-19
 * 
 * @copyright Copyright (c) 2022
 * 
 * It consists of a Distributor and one CPU interface and virtual CPU interface for each processor 
 * in the system.
 * Although there are more functional blocks in GIC-400 they are not relevant for now and 
 * for this reason are not mentioned here.
 * The Distributor, known as GICD, is responsible for detect and prioritize interrupts, and then 
 * forwards them to the target CPU interfaces. Notice that interrupts are not forward to the CPU yet. 
 * The other major functional block is the CPU interface, GICC, that performs priority masking 
 * and preemption handling of physical interrupts, signals them to the corresponding processor, and 
 * receives acknowledge and End of Interrupt (EOI) accesses from that processor.
 * 
 * The GIC-400 has several interrupts input, two of them will be needed to the project and they are:
 * 1. Software Generated Interrupt Register (SGIs);
 * 2. Shared Peripheral Interrupt (SPIs).
 * 
 * SPIs are triggered by events generated on associated interrupt input lines. The GIC-400 can support 
 * up to 480 SPIs corresponding to the external IRQS[479:0] signal. The number of SPIs available depends 
 * on the implemented configuration of the GIC-400. The permitted values are 0-480, in steps of 32. 
 * SPIs start at ID 32.
 * 
 * Is important to notice that the base address of the GIC-400 is not fixed, and can be different for a 
 * particular system implementation. For the BCM2711 the GIC-400 base address is 0xFF840000.
 * In the other hand, the GIC-400 register have fixed offsets.
 * 	0x1000 - Distributor
 * 	0x2000 - CPU interfaces
 * 
 * Finally, as final considerations about the GIC-400: in the GIC prioritization scheme, lower numbers 
 * have higher priority. 
 * To determine the number of priority bits implemented, software can write 0xFF hex to a writable 
 * GICD_IPRIORITYRn priority field, and read back the value stored.
 * 
 * This text can be found in chapter "Theoretical Foundations" in Susanoo report. It is based on 
 * "ARM®Generic Interrupt Controller"
 * 
 * Other importante link is: https://www.keil.com/support/man/docs/armasm/armasm_dom1361289871865.htm
 */

#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_
#include <stdint.h>

/* Functions */

/**
 * @brief Responsible for register a new interrupt into GIC-400 interrupt vector.
 * 
 * @param intno: interruption number 
 * @param pri: interruption priority 
 * @param cpumask: CPU mask to be applied 
 * @param fn: function callback to be called when the specified interrupt occur 
 * @return int 
 */
int isr_register(uint32_t intno, uint32_t pri, uint32_t cpumask, void (*fn)(void));

/**
 * @brief Notify the system that a given interrupt is done.
 * 
 * @param val 
 */
void eoi_notify(uint32_t val);

/**
 * @brief wait for linux to set free the gic interrupt controller
 * 
 */
void wait_gic_init(void);

/* Interrupt handler table */

/**
 * @brief Define INTERRUPT_HANDLER has a function pointer that return void and receive void
 * 
 */
typedef void (*INTERRUPT_HANDLER)(void);

/**
 * @brief Define a structure that contains the interrupt function pointer.
 * 
 * The structure will make more sense in future versions.
 */
typedef struct {
    INTERRUPT_HANDLER fn;
} INTERRUPT_VECTOR;

#endif //_INTERRUPT_H_