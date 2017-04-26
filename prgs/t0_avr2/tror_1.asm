; Basic test for ROR
; Needs ADD, ADC, LDI & OUT
   .DEVICE ATtiny22
start:
   ldi  r23,0xFF
   ldi  r24,0x01
   add  r23,r24  ; Cy=1
   ldi  r25,0x50
   ror  r25      ; 0xA8 Cy=0
   ror  r25      ; 0x54 Cy=0
   adc  r25,r24  ; 0x55
   ldi  r18,0xFF
   out  0x17,r18
   out  0x18,r25
loop:
   rjmp  loop
   nop
   nop
