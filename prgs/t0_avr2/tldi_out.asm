; Test for LDI and OUT
   .DEVICE ATtiny22
start:
   ldi   r16,0xFF
   out  0x17,r16
   ldi   r24,0x55
   out  0x18,r24
loop:
   rjmp  loop
   nop
   nop
