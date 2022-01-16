#ifndef __FIFO_MODULE__
#define __FIFO_MODULE__
/**************************************************************************
*					   Definition for a FIFO type.	                      *
***************************************************************************
* 	  The FIFO type can more esealy process information in a array.	      *
*	  A char array is the kernel of FIFO,so the FIFO is evidently limited *
***************************************************************************
* 		typedef struct myfifo											  *
*					Used to handle the FIFO information					  *
***************************************************************************
*			@Routines	                                                  *
*																		  *
*			init_fifo -> 	Initialises the FIFO						  *
*			put_char  -> 	Stores an element in the FIFO				  *
*			get_char	->	Gets an element from the FIFO		          *
*			get_len		->	Return the length of FIFO (number of   	      *
*							characters	who was not read yet)			  *
*			reset_fifo->	clear the FIFO for receiving data starting in *
*									the	beginning of the FIFO			  *
*			fifo_cmp	->	Compares two FIFO, both in length and content*
***************************************************************************
*     @Authors															  *
*					This module was created by Vitor Silva 				  *
*					Was later modified by Francisco Marques      	      *
**************************************************************************/


//#include "main.h"
#include "errono.h"
#include <stddef.h>
#include <stdint.h>

typedef struct myfifo{
	/*index who indicate the next position to write*/
	uint8_t i_w;
	/*index who indicates where is the character to read*/
	uint8_t i_r;
	/*pointer to the array who handle the data */
	uint8_t* p_buff;
	/*indicates the fifo length*/
	uint8_t fifo_len;

} fifo_t;


		/******************************************************
		* Initializes the fifo								  *
		*******************************************************
		*	@Parameters									      *
		* 		a pointer to fifo that should be initialized  *
		*		a pointer to array who will store the data	  *
		*			the fifo length							  *
		*******************************************************
		*	@Returns 									      *
		*				Dont returns nothing				  *
		******************************************************/
extern void init_fifo(fifo_t * p_fifo, uint8_t* p_buff, uint8_t fflen);


		/******************************************************
		* Stores an element in the fifo						  *
		*******************************************************
		*	@Parameters										  *
		* 		-a pointer to fifo that should handle the new *
				  character									  *
		*		-The character who should be stored	          *
		*******************************************************
		*	@Returns 										  *
		*	  0 - if the character was been stored      	  *
		*	 -ENOBUFS	- if the character was not been stored*
		******************************************************/
extern int16_t fifo_put_char(fifo_t * p_fifo, uint8_t c);


	    /******************************************************
 	 	* Delete the fifo last char element 			      *
		*******************************************************
		*	@Parameters										  *
		* 		-a pointer to fifo that should handle the new *
		*  	  	  character									  *
		*******************************************************
		*	@Returns 										  *
		*	  0 - if the character was been stored      	  *
		*	 ENODATA- if hasn't any character to delete       *
		******************************************************/
extern uint16_t fifo_delete_char(fifo_t * p_fifo);
		/******************************************************
		* Gets an element from the fifo					      *
		*******************************************************
		*	@Parameters									      *
		* 			-a pointer to fifo we want read      	  *
		*******************************************************
		*	@Returns 										  *
		*		the character who was readed	  		      *
		*		-ENODATA - if hasn't any character to read	  *
		******************************************************/
extern int16_t fifo_get_char(fifo_t * p_fifo);


		/******************************************************
		* number of characters who was not read yet			  *
		*******************************************************
		*	@Parameters										  *
		* 		-a pointer to fifo we want to know how many	  *
		*		  characters rest to read			          *
		*******************************************************
		*	@Returns 										  *
		*				the number of characters			  *
		******************************************************/
extern uint16_t get_len(fifo_t * p_fifo);


		/******************************************************
		* Reset the fifo									  *
		*******************************************************
		*	@Parameters										  *
		* 			a pointer to fifo that should be reseted  *
		*******************************************************
		*	@Returns 										  *
		*				Dont return nothing					  *
		******************************************************/
extern void reset_fifo(fifo_t* p_fifo);


		/******************************************************
		* Compares two fifos								  *
		*******************************************************
		*	@Parameters										  *
		* 			Two pointers for the fifos who should be  *
		*				compare								  *
		*******************************************************
		*	@Returns 										  *
		*				0 - if the two fifos are equal		  *
		*			!0  - if fifos are not equal              *
		******************************************************/
extern uint16_t fifo_cmp(fifo_t* p_fifo1, fifo_t* p_fifo2);


#endif /* INC_FIFO_H_ */
