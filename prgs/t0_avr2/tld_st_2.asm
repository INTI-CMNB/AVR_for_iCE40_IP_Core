; Basic test for LD & ST with inc/dec
; Needs ADD, LDI & OUT
   .DEVICE ATtiny22
start:
   ldi  r26,0x80
   ldi  r27,0x00 ; X=0x0080 RAM
   ldi  r28,19
   ldi  r29,0x00 ; Y=19 r19
   ldi  r16,0x50
   ldi  r17,0x04
   ldi  r18,0x01
   st   X+,r16 ; RAM(0x80)=0x50
   st   X+,r17 ; RAM(0x81)=0x04
   st   X+,r18 ; RAM(0x82)=0x01
   ld   r10,-X ; r10=0x01
   ld   r11,-X ; r11=0x04
   ld   r12,-X ; r12=0x50
   st   -Y,r10 ; r18=0x01
   st   -Y,r11 ; r17=0x04
   st   -Y,r12 ; r16=0x50
   add  r10,r11
   add  r10,r12
   ldi  r18,0xFF
   out  0x17,r18
   out  0x18,r10
loop:
   rjmp  loop
   nop
   nop
