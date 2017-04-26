; Basic test for BST, BLD
; Needs RJMP, BRBC, BRBS, BSET, BCLR, LDI & OUT
   .DEVICE ATtiny22
start:
   ; Port B is output
   ldi   r18,0xFF
   out   0x17,r18

   clt             ; T=0
   ldi   r28,8
   bst   r28,3     ; T=1
   brts  ok1
   rjmp  error
ok1:
   ldi   r28,0xF7
   bst   r28,3     ; T=0
   brtc  ok2
   rjmp  error
ok2:
   ldi   r29,0xAA
   set
   bld   r29,6
   bld   r29,4
   bld   r29,2
   bld   r29,0
   clt
   bld   r29,7
   bld   r29,5
   bld   r29,3
   bld   r29,1

   out   0x18,r29
loop:
   rjmp  loop
error:
   ldi   r20,0xAA
   out   0x18,r20
   rjmp  loop
   nop
   nop
