#include <stdio.h>

#include <avr/io.h>
#define WB_ADR  _SFR_IO8(0x1F)
#define WB_DAT  _SFR_IO8(0x1E)

/*
 * Send character c down the UART Tx, wait until tx holding register
 * is empty.
 */
static
int uart_putchar(char c, FILE *stream)
{
 if (c=='\n')
    uart_putchar('\r',stream); /* Terminal emulator friendly \n => \r\n */
 WB_ADR=1; /* Status is reg 1 */
 loop_until_bit_is_set(WB_DAT,0);
 WB_ADR=0; /* Data is reg 0 */
 WB_DAT=c;
 return 0;
}

FILE uart_str=FDEV_SETUP_STREAM(uart_putchar,NULL,_FDEV_SETUP_RW);

int main(int argc, char *argv[])
{
 stdout=&uart_str;

 puts("ATtinyX5 says: hello world!");
 return 0;
}

