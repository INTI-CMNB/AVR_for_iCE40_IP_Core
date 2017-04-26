; Basic test for RJMP
; Needs RCALL, CPI, BRNE, POP, LDI & OUT
   .DEVICE ATtiny22
   ; Set SP
   ldi   r16,0xDF
   out   0x3d,r16
   ldi   r16,0x00
   out   0x3e,r16
start:
   ldi   r18,0xFF
   out   0x17,r18
   rjmp  adelante
atras:
   rcall aca
aca:
   pop   r16
   cpi   r16,high(aca)
   brne  error
   pop   r16
   cpi   r16,low(aca)
   brne  error

   ldi   r20,0x55
   out   0x18,r20
loop:
   rjmp  loop
error:
   ldi   r20,0xAA
   out   0x18,r20
   rjmp  loop
adelante:
   rjmp  atras
   nop
   nop
