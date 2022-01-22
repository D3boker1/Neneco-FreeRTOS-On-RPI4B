/**
 * @file spi.c
 * @author Francisco Marques (fmarques_00@protonmail.com)
 * @brief This module allow the programme to use the spi device 0 on RPI.
 * @version 0.1
 * @date 2021-12-29
 * 
 * @copyright Copyright (c) 2021
 * 
 * This module is currently in development. DO NOT USE IT.
 */

#include "spi.h"

spictl_t spicontrol;


/*uint8_t spi_send_data(const uint8_t data){
    uint32_t free_len = MAX_LEN - uxQueueMessagesWaiting(spicontrol.tx_queue);

    if(free_len != 0){  
        xSemaphoreTake(spicontrol.tx_mux, (portTickType) portMAX_DELAY );
        xQueueSendToBack(spicontrol.tx_queue, &data, (portTickType) portMAX_DELAY);
        xSemaphoreGive(spicontrol.tx_mux);
    }
    else
        return ENOBUFS;

    SPI_CS |= (1 << TA);
    return VALID;

}*/

uint8_t spi_send_data_poll(const uint8_t data_poll){
    //uint32_t reg = 0;

    xSemaphoreTake(spicontrol.tx_mux, (portTickType) portMAX_DELAY );

    SPI_CS |= (1 << TA);

    /*do
    {
        reg = SPI_CS;
        reg = ~((reg >> TXD) & 0x01);
    } while (reg); */ /**< Waits until TxFIFO has space for at least 1 byte*/
    
    while (!((SPI_CS >> TXD) & 0x01)) {}
    

    SPI_FIFO = data_poll;
    asm volatile ("isb");
    
    while (!((SPI_CS >> DONE) & 0x01)) {}
/*
    do{
        reg = SPI_CS;
        reg = ~((reg >> DONE) & 0x01);
    }while(reg);*/ /**< Waits until DONE goes to 1*/  
   
    SPI_CS &= ~(1 << TA);

    xSemaphoreGive(spicontrol.tx_mux);

    return VALID;
}

int16_t spi_receive_data_poll(void){

    int16_t ret = -ENODATA;

    SPI_CS |= (1 << TA);
    
    while (!((SPI_CS >> RXD) & 0x01)) {} /**< Waits until has at least one byte to read*/
    
    xSemaphoreTake(spicontrol.rx_mux, (portTickType) portMAX_DELAY );
    ret = (uint16_t) SPI_FIFO;
    asm volatile ("isb");
    xSemaphoreGive(spicontrol.rx_mux);
    
    while (!((SPI_CS >> DONE) & 0x01)) {}

    SPI_CS &= ~(1 << TA);

    

    return ret;

}

/*int16_t spi_receive_data(void){
    uint16_t data16_recv = -ENODATA;
    uint8_t data_len = uxQueueMessagesWaiting(spicontrol.rx_queue);

    if(data_len != 0){
        xQueueReceive(spicontrol.rx_queue, &data16_recv, (portTickType) portMAX_DELAY);
    }
    
    return data16_recv;

}*/


/*static void spi_isr_handler(void){
    gpio_pin_set(GPIO_42, GPIO_PIN_SET);

    //uint32_t reg = SPI_CS;
    uint8_t data8_send = 0;
    uint8_t data_to_send = uxQueueMessagesWaitingFromISR(spicontrol.tx_queue);

    if((SPI_CS >> DONE) & 0x01){
        if(data_to_send != 0){
            
            xSemaphoreTakeFromISR(spicontrol.tx_mux, NULL );

            xQueueReceiveFromISR(spicontrol.tx_queue, &data8_send, NULL);
            SPI_FIFO = data8_send;
            asm volatile ("isb");
            
            xSemaphoreGiveFromISR(spicontrol.tx_mux, NULL);
            
            data_to_send --;
        }
        else{
            
            SPI_CS &= ~(1 << TA); // Clear the TA flag
        }
        reg = SPI_CS;
        reg = ((reg >> RXD) & 0x01);
        while (reg)
        {
            uint16_t c = (uint16_t) SPI_FIFO;
            xQueueSendToBackFromISR( spicontrol.rx_queue, &c , NULL);
            reg = SPI_CS;
            reg = ((reg >> RXD) & 0x01);
        }
    }

    if((SPI_CS >> RXR) & 1){
        //uint16_t c = (uint16_t) SPI_FIFO;
        //xQueueSendToBackFromISR( spicontrol.rx_queue, &c , NULL);

        while ((SPI_CS >> RXD) & 1)
        {
            uint16_t c = (uint16_t) SPI_FIFO;
            xQueueSendToBackFromISR( spicontrol.rx_queue, &c , NULL);
        }

        if(data_to_send != 0){
            xQueueReceiveFromISR(spicontrol.tx_queue, &data8_send, NULL);
            SPI_FIFO = data8_send;
        }
    }
  
}*/

//uint8_t spi_init(SPI_t spix, SPICS_t csx, SPICPOL_t cpolx, SPICPHA_t cphax){
uint8_t spi_init(SPICS_t csx, SPICPOL_t cpolx, SPICPHA_t cphax){
    uint8_t ret = ENOSYS;

    /**<Initialize the SPI control structure*/
    spicontrol.tx_mux = xSemaphoreCreateMutex();
	//spicontrol.tx_queue = xQueueCreate(MAX_LEN, sizeof (uint8_t));
    spicontrol.rx_mux = xSemaphoreCreateMutex();
	//spicontrol.rx_queue = xQueueCreate(MAX_LEN, sizeof (uint16_t));

    //Initialize the SPI0 (For now) pins
    gpio_pin_init(GPIO_7, ALT0, GPIO_PIN_PULL_NON);
    gpio_pin_init(GPIO_8, ALT0, GPIO_PIN_PULL_NON);
    gpio_pin_init(GPIO_9, ALT0, GPIO_PIN_PULL_NON);
    gpio_pin_init(GPIO_10, ALT0, GPIO_PIN_PULL_NON);
    gpio_pin_init(GPIO_11, ALT0, GPIO_PIN_PULL_NON);

    // Reset the control and status spi register. 
    // Select the chip in use, the polarity and the phase work conditions.
    SPI_CS = 0;
    SPI_CS &= ~((1 << CSPOL) | (1 << CSPOL0) | (1 << CSPOL1) | (1 << CSPOL2));
    SPI_CS |= ((csx << CS) | (cpolx << CPOL) | (cphax << CPHA));

    /**< Set the SPI0 clock divider */
    /**< SCLK = core_clock / CDIV , where core_clock = 500 MHz.*/ 
    SPI_CLK = 204; /**< SCLK ≃ 2,450980 MHz*/

    //Initialize the spi interrupt
    //SPI_CS |= ((1 << INTR) | (1 << INTD));
    //isr_register(IRQ_SPI, SPI_PRIORITY, (0x1U << 0x3U), spi_isr_handler);
    ret = VALID;

    return ret;

}