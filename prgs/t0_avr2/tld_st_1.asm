; Basic test for LD & ST (some are LDD and STD in fact)
; Needs ADD, LDI & OUT
   .DEVICE ATtiny22
start:
   ldi  r26,0x80
   ldi  r27,0x00 ; X=0x0080 RAM
   ldi  r28,0x5D
   ldi  r29,0x00 ; Y=0x005D SP Low (I/O 0x3D)
   ldi  r30,0x0F
   ldi  r31,0x00 ; Z=0x000F R15
   ldi  r16,0x50
   st   X,r16 ; RAM
   ldi  r16,0x04
   st   Y,r16 ; I/O (SP) (STD Y+0)
   ldi  r16,0x01
   std  Z+2,r16 ; R17
   ldd  r10,Z+2 ; R10=R17=1
   ld   r11,Y ; R11=SP=4 (LDD r11,Y+0)
   ld   r12,X ; R12=RAM(0x80)=0x50
   add  r12,r11
   add  r12,r10
   ldi  r18,0xFF
   out  0x17,r18
   out  0x18,r12
loop:
   rjmp  loop
   nop
   nop
