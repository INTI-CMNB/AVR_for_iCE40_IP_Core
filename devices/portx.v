/***********************************************************************

  I/O Port Device

  This file is part FPGA Libre project http://fpgalibre.sf.net/

  Description:
  Simple I/O pins device. Takes some ideas from O.C. AVR Core.

  To Do:
  -

  Author:
  Salvador E. Tropea

------------------------------------------------------------------------------

 Copyright (c) 2008-2017  <salvador en inti.gob.ar>
 Copyright (c) 2008-2017 Instituto Nacional de Tecnología Industrial

 Distributed under the GPL v2 or newer license

------------------------------------------------------------------------------

 Design unit:      IOPort
 File name:        portx.v
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

module IOPort
  #(
    parameter NUMBER=0,  // A=0, B=1, etc.
    parameter ENA_OUT=1, // Enable outputs
    parameter ENA_IN=1,  // Enable inputs
    parameter BITS=8)    // How many bits are implemented
   (
    // AVR Control
    input        clk_i,
    input        rst_i,
    input        ena_i,
    input  [5:0] adr_i,
    input  [7:0] data_i,
    output [7:0] data_o, 
    input        re_i,
    input        we_i,
    output       selected_o,
    // External connection
    input  [BITS-1:0] port_i,
    output [BITS-1:0] port_o,
    output [BITS-1:0] port_oe_o);

`include "../core/avr_ports.v"

   // Registers
reg [BITS-1:0] data_r; // Data to output
reg [BITS-1:0] ddr_r;  // Data Direction
reg [BITS-1:0] inp_r;  // Data from input
reg [BITS-1:0] inp0_r; // Filtered inputs
// Address decoding
wire data_sel;
wire dir_sel;
wire inp_sel;

assign data_sel=adr_i==PortAddress(NUMBER) && ENA_OUT;
assign dir_sel =adr_i==PortDDR(NUMBER)     && ENA_OUT && ENA_IN;
assign inp_sel =adr_i==PortPin(NUMBER)     && ENA_IN;

assign selected_o=(data_sel | dir_sel | inp_sel) & re_i;

always @(posedge clk_i)
begin : do_regs
  if (rst_i)
     begin
     data_r <= 0;
     ddr_r  <= 0;
     inp_r  <= 0;
     end
  else // rst_i
     begin
     inp_r <= inp0_r; // 2nd stage sync.
     // Write to registers
     if (we_i && ena_i)
        begin
        if (data_sel)
           data_r <= data_i[BITS-1:0];
        if (dir_sel)
           ddr_r <= data_i[BITS-1:0];
        end // we_i and ena_i
     end // else rst_i
end // do_regs

// DFF Falling Edge register
always @(negedge clk_i)
begin : dff_sync
  inp0_r <= port_i;
end // dff_sync

assign data_o[BITS-1:0]=(data_r & {BITS{data_sel}}) |
                        (ddr_r  & {BITS{dir_sel}})  |
                        (inp_r  & {BITS{inp_sel}});

// Input/Output connection
genvar i;
generate
// Bidirectional ports
if (ENA_OUT && ENA_IN)
   begin : is_in_out
   assign port_oe_o=ddr_r;
   assign port_o=data_r;
   end
else
// Output ports
if (ENA_OUT && !ENA_IN)
   begin : is_out
   assign port_oe_o={BITS{1'b1}};
   assign port_o=data_r;
   end
else
// Input ports
if (!ENA_OUT && ENA_IN)
   begin : is_in
   assign port_oe_o={BITS{1'b0}};
   assign port_o={BITS{1'b0}};
   end
endgenerate

endmodule // IOPort

