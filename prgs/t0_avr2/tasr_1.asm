; Basic test for ASR
; Needs ADC, SUB, LDI & OUT
   .DEVICE ATtiny22
start:
   ldi  r25,0xAB
   ldi  r26,0x00
   ldi  r27,0x81
   asr  r25      ; 0xD5
   adc  r25,r26  ; 0xD6
   sub  r25,r27
   ldi  r18,0xFF
   out  0x17,r18
   out  0x18,r25
loop:
   rjmp  loop
   nop
   nop
