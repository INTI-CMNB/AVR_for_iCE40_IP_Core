   .DEVICE ATtiny22
start:
   rjmp   st2
   reti
st2:
   ; Set SP
   ldi   r16,0xDF
   out  0x3d,r16
   ldi   r16,0x00
   out  0x3e,r16
   ; First RAM address to 0x41
   ldi   r16,0x41
   sts   0x60,r16
   ; r15 to 0x42
   ldi   r16,0x42
   ldi   r17,0xFF ; Just to make sure rr is ok
   sts   0x0F,r16
   ; PortB to 0x43
   ldi   r16,0x43
   sts   0x38,r16
   ; X points to first RAM address
   ldi   r26,0x60
   ldi   r27,0x00
   ; Read [0x60]
   ld    r17,X
   ; r17=0, to be sure
   ldi   r17,0x00
   ; r17=[0x60]
   lds   r17,0x0060
   ; [0x60]=r16
   st    X,r16
   ; [0x60]=r17
   sts   0x0060,r17
   ; r17=r16 using the stack
   push  r16
   pop   r17
   ; Port B bit 4=1
   sbi   0x18,4
   ; Port B bit 4=0
   cbi   0x18,4
   sbis  0x18,4
   push  r16
   sbic  0x18,4
   push  r17
   ldi   r24,35
   ; Testing call
   rcall test1
   rcall test1
   ; IRQ enable
   bset  7
   ; Disable IRQ
   bclr  7
   ldi   r16,0x40
   adiw  r24,10
   ; PortA is output
   ldi   r16,0xFF
   out  0x17,r16
   out  0x18,r24
loop:
   rjmp  loop

test1:
   adiw r24,20
   ret
   nop
   nop
