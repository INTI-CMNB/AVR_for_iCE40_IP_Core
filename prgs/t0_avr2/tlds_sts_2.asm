; Basic test for LDS & STS
; Needs ADD, INC, LDI & OUT
   .DEVICE ATtiny22
start:
   ldi  r16,0x2A
   ldi  r17,0x0
   ldi  r18,0x0
   sts  0x0011,r16
   lds  r18,0x0010
   add  r17,r18
   inc  r17
   ldi  r18,0xFF
   out  0x17,r18
   out  0x18,r17
loop:
   rjmp  loop
   nop
   nop
