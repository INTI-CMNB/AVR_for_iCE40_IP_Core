/***********************************************************************

  Internal I/O registers for the AVR core

  This file is part FPGA Libre project http://fpgalibre.sf.net/

  Description:
  Implements the Status Register, Stack Pointer and RAM Page Z I/O
  registers. They are the "internal" I/O Regs.
  It also includes the I/O data bus mux. External I/O regs uses
  e_data_i, this is shared with the external RAM data bus.

  To Do:
  -

  Author:
  Salvador E. Tropea

------------------------------------------------------------------------------

 Copyright (c) 2008-2017  <salvador en inti.gob.ar>
 Copyright (c) 2008-2017 Instituto Nacional de Tecnología Industrial

 Distributed under the GPL v2 or newer license

------------------------------------------------------------------------------

 Design unit:      IORegFile
 File name:        io_reg_file.v
 Note:             None
 Limitations:      None known
 Errors:           None known
 Library:          avr
 Dependencies:     IEEE.std_logic_1164
                   IEEE.numeric_std
                   avr.Constants
 Target FPGA:      iCE40HX4K-TQ144
 Language:         Verilog
 Wishbone:         None
 Synthesis tools:  Lattice iCECube2 2016.02.27810
 Simulation tools: GHDL [Sokcho edition] (0.2x)
 Text editor:      SETEdit 0.5.x

***********************************************************************/

`include "../core/avr_constants.v"

module IORegFile
  #(
    parameter SP_W=16,     // Stack pointer size
    parameter ENA_RAMPZ=0) // RAMPZ enable
   (
    //Clock and reset
    input clk_i, 
    input ena_i, 
    input rst_i, 
    // I/O write
    input  [5:0] adr_i,    // I/O Address
    input        we_i,     // I/O Write Enable
    input        re_i,     // I/O Read Enable
    input  [7:0] data_i,   // I/O Data In
    output [7:0] data_o,   // I/O Data Out
    input  [7:0] e_data_i, // External data (I/O-RAM)
    // Status Register
    input  [7:0] sreg_i,   // SREG in
    output [7:0] sreg_o,   // SREG out
    input  [7:0] sreg_we_i,// Flags write enable
    // Stack Pointer
    output [SP_W-1:0] sp_o,     // SP
    input             sp_pop_i, // Stack Pointer 1=Pop(+1) 0=Push(-1)
    input             sp_we_i,  // Stack Pointer write enable (dec/inc)
    // RAM Page Z Select
    output [7:0] rampz_o); // RAM Page Z

localparam SP_TOP=SP_W<9 ? 8 : SP_W-1;
localparam DATA_TOP=SP_W<9 ? 0 : SP_W-9;

reg [7:0] sreg_r=0;
reg rampz_r=0;
wire [7:0] rampz;
reg [SP_W-1:0] sp_r=0;
`define spl_r sp_r[7:0]
wire [7:0] spl;
wire [7:0] sph;

integer i;
always @(posedge clk_i)
begin : do_regs
  if (rst_i)
     begin
     sreg_r  <= 0;
     sp_r    <= 0;
     rampz_r <= 0;
     end
  else if (ena_i)
     begin
     if (we_i)
        begin
        // Write to I/O File Registers (Data Bus)
        if (adr_i==`SREG_ADDRESS)
           sreg_r <= data_i;
        else if (adr_i==`SPL_ADDRESS)
           `spl_r <= data_i;
        else if (adr_i==`SPH_ADDRESS && SP_W>8)
           sp_r[SP_TOP:8] <= data_i[DATA_TOP:0];
        else if (adr_i==`RAMPZ_ADDRESS && ENA_RAMPZ)
           rampz_r <= data_i[0];
        end
     else // we_i
        begin
        // Directly from the control logic
        // Status Register (ALU Flags and SREG bit operations)
        for (i=7; i>=0; i=i-1)
            if (sreg_we_i[i])
               sreg_r[i] <= sreg_i[i];
        // Stack Pointer (inc/dec -> push, pop, call, ret, etc.)
        if (sp_we_i)
           begin
           if (sp_pop_i)
              sp_r <= sp_r+1; // Pop
           else
              sp_r <= sp_r-1; // Push
           end // sp_we_i
        end // else we_i
     end // ena_i
end // do_regs
assign rampz={7'b0,rampz_r};

assign sph=SP_W==16 ? sp_r[15:8] : (SP_W>8 ? {{16-SP_W{1'b0}},sp_r[SP_W-1:8]} : 0);

// Individual registers
assign sreg_o =sreg_r;
assign sp_o   =sp_r;
assign rampz_o=rampz;
assign spl    =`spl_r;
// I/O Data Bus Mux, provides our registers or the external data
// Note: re_i is used to differentiate RAM and I/O reads
assign data_o=adr_i==`SPL_ADDRESS   && re_i              ? spl : (
              adr_i==`SPH_ADDRESS   && re_i && SP_W>8    ? sph : (
              adr_i==`SREG_ADDRESS  && re_i              ? sreg_r : (
              adr_i==`RAMPZ_ADDRESS && re_i && ENA_RAMPZ ? rampz : e_data_i)));

endmodule // IORegFile

