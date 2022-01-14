#include "interrupt.h"
#include "board.h"

/* Vector table */
INTERRUPT_VECTOR InterruptHandlerFunctionTable[MAX_NUM_IRQS] = {0};

/* Interrupt handler registration */
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
    
	/* GICD_ITARGETSRn (only for SPIs) 
		This field stores the list of target processors for the interrupt. That is, it holds
		the list of CPU interfaces to which the Distributor forwards the interrupt if it is asserted and
		has sufficient priority.
	*/
	//For interrupt ID m, when DIV and MOD are the integer division and modulo operations:
	if (intno >= 32U) {
		//the corresponding GICD_ITARGETSRn number, n, is given by n = m DIV 4
   	   	n = intno / 4U;
		//the offset of the required GICD_ITARGETSR is (0x800 + (4*n))	  
		addr = (uint32_t *)(GICD_BASE + 0x800U + 4U * n);
		//the byte offset of the required Priority field in this register is m MOD 4
		shift = (intno % 4U) * 8U; //8 symbolize 8 bits
		// Get the value in GICD_ITARGETSR for interrupt m
		reg = (*addr) & ~(0xFFU << shift);
		// Specify the CPU in usage
		*addr = (reg | cpumask << shift);
	}
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
