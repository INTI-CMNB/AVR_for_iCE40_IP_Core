; Basic test for SBIW
; Needs ADD, LDI & OUT
   .DEVICE ATtiny22
start:
   ldi  r26,0x30
   ldi  r27,0x2A
   sbiw r27:r26,5
   add  r26,r27
   ldi  r18,0xFF
   out  0x17,r18
   out  0x18,r26
loop:
   rjmp  loop
   nop
   nop
