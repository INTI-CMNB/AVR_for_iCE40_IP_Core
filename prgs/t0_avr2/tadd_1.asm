; Basic test for ADD & ADC
; Needs LDI & OUT
   .DEVICE ATtiny22
start:
   ldi  r16,0xE0
   ldi  r17,0x50
   add  r17,r16  ; 0x30 + Cy
   ldi  r16,0x24
   adc  r17,r16  ; 0x30+0x24+Cy=0x55
   ldi  r18,0xFF
   out  0x17,r18
   out  0x18,r17
loop:
   rjmp  loop
   nop
   nop
