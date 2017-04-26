; Basic test for LSR
; Needs ADC, LDI & OUT
   .DEVICE ATtiny22
start:
   ldi  r22,0xA9
   lsr  r22
   ldi  r23,0x00
   adc  r22,r23
   ldi  r18,0xFF
   out  0x17,r18
   out  0x18,r22
loop:
   rjmp  loop
   nop
   nop
