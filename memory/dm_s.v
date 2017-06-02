/***********************************************************************

  Single Port RAM that maps to an FPGA BRAM

  This file is part FPGA Libre project http://fpgalibre.sf.net/

  Description:
  This is a data memory for the AVR. It maps to a Lattice or Xilinx
  BRAM, most probably also works for ALTERA, Microsemi, etc.

  To Do:
  -

  Author:
    - Salvador E. Tropea, salvador inti.gob.ar

------------------------------------------------------------------------------

 Copyright (c) 2008-2017 Salvador E. Tropea <salvador inti.gob.ar>
 Copyright (c) 2008-2017 Instituto Nacional de Tecnología Industrial

 Distributed under the GPL v2 or newer license

------------------------------------------------------------------------------

 Design unit:      SinglePortRAM
 File name:        dm_s.v
 Note:             None
 Limitations:      None known
 Errors:           None known
 Library:          avr
 Dependencies:     IEEE.std_logic_1164
                   IEEE.numeric_std
 Target FPGA:      Spartan 3 (XC3S1500-4-FG456)
                   iCE40HX4K-TQ144
 Language:         Verilog
 Wishbone:         No
 Synthesis tools:  Xilinx Release 9.2.03i - xst J.39
                   Lattice iCECube2 2016.02.27810
 Simulation tools: GHDL [Sokcho edition] (0.2x)
 Text editor:      SETEdit 0.5.x

***********************************************************************/

module SinglePortRAM
  #(
    parameter FALL_EDGE=0,
    parameter WORD_SIZE=8, // Word Size
    parameter ADDR_W=12)   // Address Width
   (
    input                  clk_i,
    input                  we_i,
    input  [ADDR_W-1:0]    addr_i,
    input  [WORD_SIZE-1:0] d_i,
    output [WORD_SIZE-1:0] d_o);

reg [ADDR_W-1:0] addr_r;
reg [WORD_SIZE-1:0] ram[0:(2**ADDR_W)-1];

generate
if (!FALL_EDGE)
   begin : use_rising_edge
   always @(posedge clk_i)
   begin : do_ram
     if (we_i)
        ram[addr_i] <= d_i;
     addr_r <= addr_i;
   end // do_ram
   end // use_rising_edge
else
   begin : use_falling_edge
   always @(negedge clk_i)
   begin : do_ram
     if (we_i)
        ram[addr_i] <= d_i;
     addr_r <= addr_i;
   end // do_ram
   end // use_falling_edge
endgenerate

assign d_o=ram[addr_r];

endmodule // SinglePortRAM

