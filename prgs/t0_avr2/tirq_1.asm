; Basic test for Interrupts
; IMPORTANT: Uses PORTD (not real in ATtiny2) and asumes b7 is conected to the external interrupt
; Needs ORI, LDI & OUT
   .DEVICE ATtiny22
   rjmp  start
   rjmp  interrupt
start:
   ; Set SP
   ldi   r16,0xDF
   out   0x3d,r16
   ldi   r16,0x00
   out   0x3e,r16

   ; PORTB is output
   ldi   r18,0xFF
   out   0x17,r18

   ; PORTA is output and with 0x80
   ldi   r16,0x80
   out   0x12,r16
   ldi   r16,0xFF
   out   0x11,r16

   ldi   r18,0xAA
   sei   ; The IRQ will be served in the next cycle

   out   0x18,r18 ; r18 should be 0x55 here
loop:
   rjmp  loop
interrupt:
   ldi   r18,0x55
   reti
   nop
   nop
