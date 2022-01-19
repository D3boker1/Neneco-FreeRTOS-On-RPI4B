/**
 * @file interrupt.c
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
#include "interrupt.h"
#include "board.h"

/**< Vector table */
INTERRUPT_VECTOR InterruptHandlerFunctionTable[MAX_NUM_IRQS] = {0};

/**< Interrupt handler registration */
int isr_register(uint32_t intno, uint32_t pri, uint32_t cpumask, void (*fn)(void))
{
	uint32_t n, reg, shift;
	uint32_t *addr;

	if (intno > 0xFFU) {
		return -1;
	}
	if (pri > 0xFFU) {
		return -2;
	}
	if (cpumask > 0xFFU) {
		return -3;
	}
	if (!fn) {
		return -4;
	}

	/* GICD_ISENABLERn */
	n = intno / 32U;
	addr = (uint32_t *)(GICD_BASE + 0x100U + 4U * n);
	reg = *addr;
	*addr = (reg | (0x1U << (intno % (32U))));
    
	/* GICD_IPRIORITYRn */
   	n = intno / 4U;
	addr = (uint32_t *)(GICD_BASE + 0x400U + 4U * n);
	shift = (intno % 4U) * 8U;
	reg = (*addr) & ~(0xFFU << shift);
	*addr = (reg | pri << shift);
	
	/**< This information was obtained in "ARM®Generic Interrupt Controller"*/
	/**< The algorithm is based in the descriptions made in the same document*/
	/**
	 *  GICD_ITARGETSRn (only for SPIs) 
	 * This field stores the list of target processors for the interrupt. That is, it holds
	 * the list of CPU interfaces to which the Distributor forwards the interrupt if it is asserted and
	 * has sufficient priority.
	*/
	/**<For interrupt ID m, when DIV and MOD are the integer division and modulo operations:*/
	if (intno >= 32U) {
		//the corresponding GICD_ITARGETSRn number, n, is given by n = m DIV 4
   	   	n = intno / 4U;
		//the offset of the required GICD_ITARGETSR is (0x800 + (4*n))	  
		addr = (uint32_t *)(GICD_BASE + 0x800U + 4U * n);
		//the byte offset of the required Priority field in this register is m MOD 4
		shift = (intno % 4U) * 8U; //8 symbolize 8 bits
		// Get the value in GICD_ITARGETSR for interrupt m
		reg = (*addr) & ~(0xFFU << shift);
		// Specify the CPU in usage. NOTICE: THIS PORT ID BASED ON CPU 3
		*addr = (reg | cpumask << shift);
	}
	/**
	 * @brief Instruction Synchronization Barrier flushes the pipeline in the processor
	 * 
	 * all instructions following the ISB are fetched from cache or memory, after the instruction has been 
	 * completed. It ensures that the effects of context altering operations executed before the ISB instruction 
	 * are visible to the instructions fetched after the ISB.
	 * 
	 * https://www.keil.com/support/man/docs/armasm/armasm_dom1361289871865.htm
	 */
    asm volatile ("isb");

	/* Handler registration */
    InterruptHandlerFunctionTable[intno].fn = fn;
    
	return 0;
}   
/*-----------------------------------------------------------*/

/* EOI notification */
/* 
	end of interrupt.
	it has completed the processing of the specified interrupt
*/
void eoi_notify(uint32_t val)
{
	uint32_t *addr;

	addr = (uint32_t *)(GICC_BASE + 0x10U);
	*addr = val;
    asm volatile ("isb");

	return;
}   
/*-----------------------------------------------------------*/

/*
 * wait_gic_init()
 * To check GIC initialization by Linux
 * 
 * this function was developed by eggman
 */
void wait_gic_init(void)
{
	volatile uint32_t *addr;

	addr = (uint32_t *)(GICD_BASE + 0x00U);

	while (*addr == 0x1U) { /* Wait until Linux disables GICD to set it up */
		;
	}
	while (*addr == 0x0U) { /* Wait until Linux enables GICD again after completing GICD setting up */
		;
	}

	return;
}
/*-----------------------------------------------------------*/
