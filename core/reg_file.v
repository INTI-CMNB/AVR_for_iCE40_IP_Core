/***********************************************************************

  Registers File

  This file is part FPGA Libre project http://fpgalibre.sf.net/

  Description:
  This is a synchronous implementation, specifically oriented to the
  iCE40 resources.
  Is a dual-port memory : (Read-first)
  * Port A
    - 8 bits R/W
    - 16 bits R/W
  * Port B
    - 8 bits R/W

  To Do:
  -

  Author:
  Salvador E. Tropea

------------------------------------------------------------------------------

 Copyright (c) 2016-2017  <salvador en inti.gob.ar>
 Copyright (c) 2016-2017 Instituto Nacional de Tecnología Industrial

 Distributed under the GPL v2 or newer license

------------------------------------------------------------------------------

 Design unit:      RegisterFile
 File name:        reg_file.v
 Note:             None
 Limitations:      None known
 Errors:           None known
 Library:          avr
 Dependencies:     IEEE.std_logic_1164
                   IEEE.numeric_std
 Target FPGA:      iCE40HX4K-TQ144
 Language:         Verilog
 Wishbone:         None
 Synthesis tools:  Lattice iCECube2 2016.02.27810
 Simulation tools: GHDL [Sokcho edition] (0.2x)
 Text editor:      SETEdit 0.5.x

***********************************************************************/


module RegisterFile
   (
    input         clk_i,
    // Main 8 bits port
    input         rd_we_i,
    input  [ 4:0] rd_adr_i,
    input  [ 7:0] rd_i,
    output [ 7:0] rd_o,
    // Main 16 bits port
    input         rd16_we_i,
    input  [15:0] rd16_i,
    output [15:0] rd16_o,
    // Secondary 8 bits read port
    input  [ 4:0] rr_adr_i,
    output [ 7:0] rr_o);

reg [7:0] ram_0[0:15]; // Even
reg [7:0] ram_1[0:15]; // Odd
// Port used to read Rr
reg [7:0] ramB_0[0:15]; // Even
reg [7:0] ramB_1[0:15]; // Odd

wire [3:0] adr16;
reg  [3:0] adr16_r;
reg        adr_lsb_r;
wire [3:0] adr16B;
reg  [3:0] adr16B_r;
reg        adr_lsbB_r;
wire       we0, we1;
wire [7:0] r0i;
wire [7:0] r1i;

// Address for 16x16 registers
assign adr16 =rd_adr_i[4:1];
assign adr16B=rr_adr_i[4:1];
// Decode which RAM/s will have WE asserted
assign we0=rd16_we_i || (rd_we_i && !rd_adr_i[0]);
assign we1=rd16_we_i || (rd_we_i &&  rd_adr_i[0]);
// Muxes for 16/8 bits writes
assign r0i=rd16_we_i ? rd16_i[ 7:0] : rd_i;
assign r1i=rd16_we_i ? rd16_i[15:8] : rd_i;

always @(posedge clk_i)
begin : do_mem
  if (we0)
     begin
     ram_0[adr16]  <= r0i;
     // Mirror
     ramB_0[adr16] <= r0i;
     end
  if (we1)
     begin
     ram_1[adr16]  <= r1i;
     // Mirror
     ramB_1[adr16] <= r1i;
     end
  adr16_r    <= adr16;
  adr_lsb_r  <= rd_adr_i[0];
  // Secondary read port
  adr16B_r   <= adr16B;
  adr_lsbB_r <= rr_adr_i[0];
end // do_mem

assign rd16_o={ram_1[adr16_r],ram_0[adr16_r]};
assign rd_o=adr_lsb_r ? ram_1[adr16_r] : ram_0[adr16_r];
// Secondary read port
assign rr_o=adr_lsbB_r ? ramB_1[adr16B_r] : ramB_0[adr16B_r];

endmodule // RegisterFile

