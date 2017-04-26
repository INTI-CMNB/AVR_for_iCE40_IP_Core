; Basic test for LDS & STS
; Needs ADD, LDI & OUT
   .DEVICE ATtiny22
start:
   ldi  r16,0x50
   sts  0x0080,r16 ; RAM
   ldi  r16,0x04
   sts  0x005D,r16 ; I/O (SP)
   ldi  r16,0x01
   sts  0x000F,r16 ; R15
   lds  r10,0x000F ; R10=R15=1
   lds  r11,0x005D ; R11=SP=4
   lds  r12,0x0080 ; R12=RAM(0x80)=0x50
   add  r12,r11
   add  r12,r10
   ldi  r18,0xFF
   out  0x17,r18
   out  0x18,r12
loop:
   rjmp  loop
   nop
   nop
