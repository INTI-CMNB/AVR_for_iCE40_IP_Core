; Basic test for SUBI, SBCI, ANDI & ORI
; Needs LDI & OUT
   .DEVICE ATtiny22
start:
   ldi  r17,0x50
   subi r17,0xE0  ; 0x70 + Cy
   sbci r17,0x1D  ; 0x70-0x1D-Cy=0x52
   andi r17,0xFD  ; 0x50
   ori  r17,0x05  ; 0x55
   ldi  r18,0xFF
   out  0x17,r18
   out  0x18,r17
loop:
   rjmp  loop
   nop
   nop
