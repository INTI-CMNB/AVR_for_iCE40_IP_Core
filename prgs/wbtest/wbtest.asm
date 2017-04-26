   .DEVICE ATmega103
start:
   rjmp  st2
   reti
st2:
   ; PortA is output
   ldi  r16,0xFF
   out  0x17,r16

   ; Test WB reg 1
   ; Address 0
   ldi  r16,0x00
   out  0x1F,r16
   ldi  r16,0x55
   out  0x1E,r16
   in   r17,0x1E
   cpse r16,r17
   rjmp error
   ldi  r16,0xAA
   out  0x1E,r16
   in   r17,0x1E
   cpse r16,r17
   rjmp error

   ; Test WB reg 2
   ; Address 1
   ldi  r16,0x01
   sts  0x3F,r16
   ldi  r16,0x55
   sts  0x3E,r16
   lds  r17,0x3E
   com  r16
   cpse r16,r17
   rjmp error
   ldi  r16,0xAA
   sts  0x3E,r16
   lds  r17,0x3E
   com  r16
   cpse r16,r17
   rjmp error

   ; Test WB reg 3
   ; Address 2
   ldi  r16,0x02
   out  0x1F,r16
   ldi  r16,0x55
   out  0x1E,r16
   in   r17,0x1E
   ldi  r18,0x55
   eor  r16,r18
   cpse r16,r17
   rjmp error
   ldi  r16,0xAA
   out  0x1E,r16
   in   r17,0x1E
   eor  r16,r18
   cpse r16,r17
   rjmp error

   ; Success!
   ldi  r16,0x55
   out  0x18,r16
end:
   rjmp end

error:
   ldi  r16,0xAA
   out  0x18,r16
   rjmp end

   nop
   nop

