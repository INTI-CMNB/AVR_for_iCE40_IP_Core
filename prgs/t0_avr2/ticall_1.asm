; Basic test for ICALL
; Needs ORI, LDI & OUT
   .DEVICE ATtiny22
start:
   ; Set SP
   ldi   r16,0xDF
   out   0x3d,r16
   ldi   r16,0x00
   out   0x3e,r16
   ; Set Z
   ldi   r30,add_more
   ldi   r31,0x00
   ; r20=0x50
   ldi   r20,0x50
   ; Call subroutine to complete r20
   icall
   ldi   r18,0xFF
   out   0x17,r18
   out   0x18,r20
loop:
   rjmp  loop
   ori   r20,0x30
add_more:
   ori   r20,0x5
   ret
   ori   r20,0x10
   nop
   nop
