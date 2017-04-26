; Basic test for SWAP & MOV
; Needs LDI & OUT
   .DEVICE ATtiny22
start:
   ldi  r26,0x45
   swap r26
   ori  r26,0x01
   mov  r1,r26
   ldi  r18,0xFF
   out  0x17,r18
   out  0x18,r1
loop:
   rjmp  loop
   nop
   nop
