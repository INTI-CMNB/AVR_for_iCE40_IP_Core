; Basic test for SUB & SBC
; Needs LDI & OUT
   .DEVICE ATtiny22
start:
   ldi  r16,0xE0
   ldi  r17,0x50
   sub  r17,r16  ; 0x70 + Cy
   ldi  r16,0x1A
   sbc  r17,r16  ; 0x70-0x1A-Cy=0x55
   ldi  r18,0xFF
   out  0x17,r18
   out  0x18,r17
loop:
   rjmp  loop
   nop
   nop
