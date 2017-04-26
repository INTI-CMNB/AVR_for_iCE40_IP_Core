; Basic test for SBRC, SBRS
; Needs RJMP, LDI & OUT
   .DEVICE ATtiny22
start:
   ; Port B is output
   ldi   r18,0xFF
   out   0x17,r18

   ldi   r16,8
   sbrs  r16,3 ; Skip if set
   rjmp  error
   sbrc  r16,3
   rjmp  c1
   rjmp  error
c1:

   ldi   r16,0xF7
   sbrc  r16,3 ; Skip if clear
   rjmp  error
   sbrs  r16,3
   rjmp  c2
   rjmp  error
c2:
   ldi   r20,0x55
   out   0x18,r20
loop:
   rjmp  loop
error:
   ldi   r20,0xAA
   out   0x18,r20
   rjmp  loop
   nop
   nop
