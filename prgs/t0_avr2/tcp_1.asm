; Basic test for CP, CPC, CPI, CPSE
; Needs RJMP, LDI & OUT
   .DEVICE ATtiny22
start:
   ; Port B is output
   ldi   r18,0xFF
   out   0x17,r18

   ldi   r23,45
   ldi   r22,45
   cp    r23,r22
   brne  error
   ldi   r24,45
   cp    r23,r24
   brne  error
   cp    r24,r23
   brne  error
   ldi   r24,44
   cp    r24,r23
   breq  error

   sez
   sec
   cpc   r23,r24
   brne  error

   cpi   r23,45
   brne  error

   cpse  r22,r23
   rjmp  error
   cpse  r23,r24
   rjmp  ok
   rjmp  error
ok:

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
