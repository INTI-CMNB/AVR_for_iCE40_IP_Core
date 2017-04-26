; Basic test for LPM
; Needs ADIW, MOV, LDI & OUT
   .DEVICE ATtiny22
start:
   ldi  r18,0xFF
   out  0x17,r18

   ldi  r31,high(val<<1) ; Initialize Z pointer
   ldi  r30,low(val<<1)
   lpm
   mov  r16,r0          ; r16=0x23
   ; Z++
   adiw r31:r30,1
   lpm
   mov  r17,r0          ; r17=0x78

   sub  r17,r16
   out  0x18,r17
loop:
   rjmp  loop
val:
   .dw 0x7823
   nop
   nop

