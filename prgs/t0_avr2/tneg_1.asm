; Basic test for NEG
; Needs LDI & OUT
   .DEVICE ATtiny22
start:
   ldi  r20,0xAB
   neg  r20
   ldi  r18,0xFF
   out  0x17,r18
   out  0x18,r20
loop:
   rjmp  loop
   nop
   nop
