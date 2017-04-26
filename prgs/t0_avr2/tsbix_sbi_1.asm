; Basic test for SBI, CBI, SBIC, SBIS, IN
; Needs RJMP, LDI & OUT
; IMPORTANT!!! the only addressable registers are PORTB and EEPROM.
;              this test uses a PORTD (0x12) address that doesn't exist in ATtiny22
   .DEVICE ATtiny22
start:
   ; Port B is output
   ldi   r18,0xFF
   out   0x17,r18

   ; Clear SP Low
   ldi   r16,0
   out   0x12,r16

   sbi   0x12,3
   sbis  0x12,3 ; Skip if set
   rjmp  error
   sbic  0x12,3
   rjmp  c1
   rjmp  error
c1:

   cbi   0x12,3
   sbic  0x12,3 ; Skip if clear
   rjmp  error
   sbis  0x12,3
   rjmp  c2
   rjmp  error
c2:
   ldi   r16,2
   out   0x12,r16
   sbi   0x12,0
   cbi   0x12,1
   sbi   0x12,2
   sbi   0x12,4
   sbi   0x12,6
   in    r20,0x12

   out   0x18,r20
loop:
   rjmp  loop
error:
   ldi   r20,0xAA
   out   0x18,r20
   rjmp  loop
   nop
   nop
