; Basic test for RJMP
; Needs ORI, LDI & OUT
   .DEVICE ATtiny22
start:
   ldi   r18,0xFF
   out   0x17,r18
   ldi   r20,0x50
   rjmp  add_more
end:
   out   0x18,r20
loop:
   rjmp  loop
   ori   r20,0x30
add_more:
   ori   r20,0x5
   rjmp  end
   ori   r20,0x10
   nop
   nop
