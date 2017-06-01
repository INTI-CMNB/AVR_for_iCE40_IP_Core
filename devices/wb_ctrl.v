/***********************************************************************

  WISHBONE bridge control

  This file is part FPGA Libre project http://fpgalibre.sf.net/

  Description:
  This is a very simple WISHBONE bridge peripheral. It contains 2
  registers: WB_ADDRESS and WB_DATA. The CPU must load the desired
  address in WB_ADDDRESS and then operate on WB_DATA. This will be
  translated into a WISHBONE transaction. The ena_o output is used
  to introduce wait states. Note this peripheral doesn't have ena_i
  this signal should go directly to all WISHBONE peripherals.

  To Do:
  -

  Author:
    - Salvador E. Tropea, salvador en inti.gob.ar

------------------------------------------------------------------------------

 Copyright (c) 2009-2017 Salvador E. Tropea <salvador en inti.gob.ar>
 Copyright (c) 2009-2017 Instituto Nacional de Tecnología Industrial

 Distributed under the GPL v2 or newer license

------------------------------------------------------------------------------

 Design unit:      WBControl
 File name:        wb_ctrl.v
 Note:             None
 Limitations:      None known
 Errors:           None known
 Library:          avr
 Dependencies:     IEEE.std_logic_1164
                   IEEE.numeric_std
 Target FPGA:      iCE40HX4K-TQ144
 Language:         Verilog
 Wishbone:         MASTER (rev B.3)
 Synthesis tools:  Lattice iCECube2 2016.02.27810
 Simulation tools: GHDL [Sokcho edition] (0.2x)
 Text editor:      SETEdit 0.5.x

------------------------------------------------------------------------------

 Wishbone Datasheet

  1 Revision level                      B.3
  2 Type of interface                   MASTER
  3 Defined signal names                RST_I => rst_i
                                        CLK_I => clk_i
                                        ADR_O => wb_adr_o
                                        DAT_I => wb_dat_i
                                        DAT_O => wb_dat_o
                                        WE_O  => wb_we_o
                                        STB_O => wb_stb_o
                                        ACK_I => wb_ack_i
  4 ERR_I                               Unsupported
  5 RTY_I                               Unsupported
  6 TAGs                                None
  7 Port size                           8-bit
  8 Port granularity                    8-bit
  9 Maximum operand size                8-bit
 10 Data transfer ordering              N/A
 11 Data transfer sequencing            Undefined
 12 Constraints on the CLK_I signal     None

 Notes: SEL_O isn't needed because size==granularity

***********************************************************************/

`include "../core/avr_constants.v"

module WBControl
  #(
    parameter [5:0] ADDR_REG=`WB_ADDRESS,
    parameter [5:0] DATA_REG=`WB_DATA)
   (
    // AVR Control
    input        rst_i,  // Reset
    input        clk_i,  // Clock
    output       ena_o,
    // I/O Bus
    input  [5:0] adr_i,
    input  [7:0] data_i,
    output [7:0] data_o, 
    input        re_i,
    input        we_i,
    output       selected_o,
    // WISHBONE side
    output [7:0] wb_adr_o,  // I/O Address
    output [7:0] wb_dat_o,  // Data Bus output
    input  [7:0] wb_dat_i,  // Data Bus input
    output       wb_stb_o,  // Strobe output
    output       wb_we_o,   // Write Enable output
    input        wb_ack_i); // Acknowledge input

reg  [7:0] addr_r;
wire addr_sel;
wire data_sel;
wire strobe;

assign addr_sel  =adr_i==ADDR_REG;
assign data_sel  =adr_i==DATA_REG;
assign selected_o=addr_sel | data_sel;
assign wb_dat_o  =data_i;
assign data_o    =(addr_r   & {8{addr_sel}}) |
                  (wb_dat_i & {8{data_sel}});
assign strobe    =data_sel & (re_i | we_i);
assign wb_stb_o  =strobe;
assign wb_we_o   =data_sel & we_i;
assign ena_o     =!strobe || wb_ack_i;
assign wb_adr_o  =addr_r;

always @(posedge clk_i)
begin : do_addr_r
  if (rst_i)
     addr_r <= 0;
  else if (addr_sel && we_i)
     addr_r <= data_i;
end process do_addr_r;

endmodule // WBControl

