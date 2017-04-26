; Basic test for PUSH & POP
; Needs CPI, LDS, IN, BRNE, RJMP, LDI & OUT
   .DEVICE ATtiny22
start:
   ldi  r18,0xFF
   out  0x17,r18
   ; Set SP
   ldi   r16,0xDF
   out   0x3D,r16
   ldi   r16,0x00
   out   0x3E,r16

   ldi   r18,0x33
   push  r18

   ldi   r28,1
   in    r19,0x3D
   cpi   r19,0xDE
   brne  error

   ldi   r28,2
   lds   r20,0x00DF
   cpi   r20,0x33
   brne  error

   ldi   r28,3
   ldi   r21,0
   pop   r21
   cpi   r20,0x33
   brne  error

   ldi   r28,4
   in    r19,0x3D
   cpi   r19,0xDF
   brne  error

   ldi   r20,0x55
   out  0x18,r20
loop:
   rjmp  loop
error:
   out   0x18,r28
   rjmp  loop
   nop
   nop
