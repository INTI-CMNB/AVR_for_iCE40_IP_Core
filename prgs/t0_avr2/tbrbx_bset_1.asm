; Basic test for BSET, BCLR, BRBS, BRBC, RETI
; Needs RJMP, RCALL, LDI & OUT
   .DEVICE ATtiny22
start:
   ; Set SP
   ldi   r16,0xDF
   out   0x3d,r16
   ldi   r16,0x00
   out   0x3e,r16
   ; Port B is output
   ldi   r18,0xFF
   out   0x17,r18

   ;
   ; Carry
   ;
   sec         ; Cy=1
   brcs  c1    ; Branch if carry set
   rjmp  error
c1:
   brcc  error ; Branch if carry clear

   clc         ; Cy=0
   brcc  c2    ; Branch if carry clear
   rjmp  error
c2:
   brcs  error ; Branch if carry set

   ;
   ; Negative
   ;
   sen         ; N=1
   brmi  n1    ; Branch if minus (N=1)
   rjmp  error
n1:
   brpl  error ; Branch if plus (N=0)

   cln         ; N=0
   brpl  n2    ; Branch if plus (N=0)
   rjmp  error
n2:
   brmi  error ; Branch if minus (N=1)

   ;
   ; Interrupt
   ;
   rcall set_i ; I=1 using RETI
   brie  i1    ; Branch if interrupt enabled
   rjmp  error
i1:
   brid  error ; Branch if interrupt disabled

   cli         ; I=0
   brid  i2    ; Branch if interrupt disabled
   rjmp  error
i2:
   brie  error ; Branch if interrupt enabled

   ldi   r20,0x55
   out   0x18,r20
loop:
   rjmp  loop
error:
   ldi   r20,0xAA
   out   0x18,r20
   rjmp  loop
set_i:
   reti
   nop
   nop
