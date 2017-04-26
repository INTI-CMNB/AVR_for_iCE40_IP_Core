; Basic test for AND, OR & EOR
; Needs LDI & OUT
   .DEVICE ATtiny22
start:
   ldi  r16,0xE2
   ldi  r17,0x5F
   ldi  r18,0x14
   ldi  r19,0x03
   and  r16,r17
   or   r16,r18
   eor  r16,r19
   ldi  r18,0xFF
   out  0x17,r18
   out  0x18,r16
loop:
   rjmp  loop
   nop
   nop
