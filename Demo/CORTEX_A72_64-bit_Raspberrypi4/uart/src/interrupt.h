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
 * @brief Notigy the system that a given interrupt is done.
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
 */
typedef struct {
    INTERRUPT_HANDLER fn;
} INTERRUPT_VECTOR;

#endif //_INTERRUPT_H_