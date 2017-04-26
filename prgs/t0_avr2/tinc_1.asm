; Basic test for COM
; Needs LDI & OUT
   .DEVICE ATtiny22
start:
   ldi  r21,0x53
   inc  r21
   inc  r21
   inc  r21
   inc  r21
   dec  r21
   dec  r21
   ldi  r18,0xFF
   out  0x17,r18
   out  0x18,r21
loop:
   rjmp  loop
   nop
   nop
