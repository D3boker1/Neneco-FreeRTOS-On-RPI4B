
#include "fifo.h"

void init_fifo(fifo_t * p_fifo, uint8_t* p_buff, uint8_t fflen){

	p_fifo->i_w = 0;
	p_fifo->i_r = 0;
	p_fifo->p_buff = p_buff;
	p_fifo->fifo_len = fflen;
	
}

uint16_t get_len(fifo_t * p_fifo){
	/*if((p_fifo->p_buff) == 0)
		return -ENOSYS;
	*/
	return  (p_fifo->i_w - p_fifo->i_r) ;
}



 int16_t fifo_put_char(fifo_t * p_fifo, uint8_t c){
	 uint8_t index;
	 
	if(get_len(p_fifo) >= p_fifo->fifo_len)
		return -ENOBUFS;
	index = p_fifo->i_w & (p_fifo->fifo_len-1);
	p_fifo->p_buff[index] = c;
	p_fifo->i_w++;
	
	return 0;
}

 uint16_t fifo_delete_char(fifo_t * p_fifo){

	if(p_fifo->i_w == 0)
		return ENODATA;

	p_fifo->i_w--;

	return 0;
}


 int16_t fifo_get_char(fifo_t * p_fifo){
	
	 uint8_t index;
	 if( get_len(p_fifo) == 0)
		 return -ENODATA;
	 
	 index  = p_fifo->i_r & (p_fifo->fifo_len-1);
	 p_fifo->i_r++;
	 
	 return  p_fifo->p_buff[index];
}

void reset_fifo(fifo_t* p_fifo)
{
	p_fifo->i_w = 0;
	p_fifo->i_r = 0;
}


 uint16_t fifo_cmp(fifo_t* p_fifo1, fifo_t* p_fifo2)
{	
	uint8_t equal = 1;
	uint8_t i;
	
	if((p_fifo1 == 0) || (p_fifo2 == 0))
			return -EILSEQ;
	
	for (i = 0; i < get_len(p_fifo1); i++)
	{
			equal &= (p_fifo1->p_buff[i + p_fifo1->i_r] == p_fifo2->p_buff[i + p_fifo2->i_r]);
	}
	
	return equal;
}
