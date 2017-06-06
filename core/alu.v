/***********************************************************************

  ALU (Arithmetic Logic Unit)

  This file is part FPGA Libre project http://fpgalibre.sf.net/

  Description:
  This is an ALU approach for AVR v2.x. Uses 8 operations.

  To Do:
  -

  Author:
  Salvador E. Tropea

------------------------------------------------------------------------------

 Copyright (c) 2016-2017  <salvador en inti.gob.ar>
 Copyright (c) 2016-2017 Instituto Nacional de Tecnología Industrial

 Distributed under the GPL v2 or newer license

------------------------------------------------------------------------------

 Design unit:      ALU
 File name:        alu.v
 Note:             None
 Limitations:      None known
 Errors:           None known
 Library:          avr
 Dependencies:     IEEE.std_logic_1164
                   IEEE.numeric_std
                   avr.Constants
                   avr.Internal
                   avr.Types
 Target FPGA:      iCE40HX4K-TQ144
 Language:         Verilog
 Wishbone:         None
 Synthesis tools:  Lattice iCECube2 2016.02.27810
 Simulation tools: GHDL [Sokcho edition] (0.2x)
 Text editor:      SETEdit 0.5.x

***********************************************************************/

module ALU
   (
    // Operands
    input [7:0] a_i,
    input [7:0] b_i,
    // Input flags
    input c_i, 
    // Operation (only one active at the time)
    input op_add_i, 
    input op_sub_i, 
    input op_and_i, 
    input op_or_i, 
    input op_xor_i, 
    input op_nop_i, 
    input op_shf_i, 
    input op_swp_i, 
    // Result
    output [7:0] s_o,
    // Flags
    output c_o, 
    output h_o, 
    output z_o, 
    output v_o);

wire [7:0] s; // Result
wire c; // Carry
wire h; // Half Carry
wire z; // Zero
wire v; // Overflow
wire [0:0] cin; // Adder/Sub Carry In
// Adder
wire [4:0] add_lo; // Adder Low Nibble
wire [4:0] add_hi; // Adder High Nibble
// Substractor
wire [4:0] sub_lo; // Adder Low Nibble
wire [4:0] sub_hi; // Adder High Nibble

assign cin[0]=c_i;
///////////////////////
// Adder: A+B        //
///////////////////////
// Low nibble
assign add_lo={1'b0,a_i[3:0]}+b_i[3:0]+cin;
// High nibble
assign add_hi={1'b0,a_i[7:4]}+b_i[7:4]+add_lo[4:4];
///////////////////////
// Substractor: A-B  //
///////////////////////
// Low nibble
assign sub_lo={1'b0,a_i[3:0]}-b_i[3:0]-cin;
// High nibble
assign sub_hi={1'b0,a_i[7:4]}-b_i[7:4]-sub_lo[4:4];

// Outputs
// Result
assign s=({add_hi[3:0],add_lo[3:0]} & {8{op_add_i}}) |
         ({sub_hi[3:0],sub_lo[3:0]} & {8{op_sub_i}}) |
         ({c_i,a_i[7:1]}            & {8{op_shf_i}}) |
         ((a_i & b_i)               & {8{op_and_i}}) |
         ((a_i | b_i)               & {8{op_or_i}})  |
         ((a_i ^ b_i)               & {8{op_xor_i}}) |
         ({a_i[3:0]&a_i[7:4]}       & {8{op_swp_i}}) |
         (a_i                       & {8{op_nop_i}});
assign s_o=s;
// Zero flag
assign z=s==8'b0;
assign z_o=z;
// Carry flag
assign c=(add_hi[4] & op_add_i) | // Cy from the high nibble
         (sub_hi[4] & op_sub_i) | // Borrow from the high nibble
         (a_i[0]    & op_shf_i);
assign c_o=c;
// Half carry flag
assign h=(add_lo[4] & op_add_i) | // Cy from the low nibble
         (sub_lo[4] & op_sub_i);   // Borrow from the low nibble
assign h_o=h;
// Overflow flag
assign v=(((a_i[7] & b_i[7] & ~add_hi[3]) | // (-)+(-)=(+)
          (~a_i[7] & ~b_i[7] & add_hi[3]))  // (+)+(+)=(-)
          & op_add_i) |
         (((a_i[7] & ~b_i[7] & ~sub_hi[3]) | // (-)-(+)=(+)
          (~a_i[7] & b_i[7] & sub_hi[3]))    // (+)-(-)=(-)
          & op_sub_i) |
         ((s[7] ^ s[0]) & op_shf_i);
assign v_o=v;

endmodule // ALU

