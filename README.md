# Como usar

1. Copiar a imagem para o cartão sd através do comando dd com a imagem sdcard.img
2. copiar o ficheiro config.txt para a partição boot
3. copiar o ficheiro kernel8.img para a partição boot
4. copiar o ficheiro elf com a app freeRTOS para a partição boot
5. Para correr na rpi:

dcache off

fatload mmc 0 0x28000000 uart.elf

dcache flush

bootelf 0x28000000
