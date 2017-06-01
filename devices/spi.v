/***********************************************************************

  SPI_Dev [Master only]

  This file is part FPGA Libre project http://fpgalibre.sf.net/

  Description:
  SPI implementation. Not 100% but enough for the Arduino API.

  To Do:
  -

  Author:
    - Salvador E. Tropea, salvador en inti.gob.ar

------------------------------------------------------------------------------

 Copyright (c) 2009-2017 Salvador E. Tropea <salvador en inti.gob.ar>
 Copyright (c) 2009-2017 Instituto Nacional de Tecnología Industrial

 Distributed under the GPL v2 or newer license

------------------------------------------------------------------------------

 Design unit:      SPI_Dev
 File name:        spi.v
 Note:             None
 Limitations:      None known
 Errors:           None known
 Library:          avr
 Dependencies:     IEEE.std_logic_1164
                   IEEE.numeric_std
                   SPI.Devices
                   avr.Constants
 Target FPGA:      iCE40HX4K-TQ144
 Language:         Verilog
 Wishbone:         None
 Synthesis tools:  Lattice iCECube2 2016.02.27810
 Simulation tools: GHDL [Sokcho edition] (0.2x)
 Text editor:      SETEdit 0.5.x

***********************************************************************/

module SPI_Dev
  #(
    parameter ENABLE=1,
    parameter WCOL_ENABLE=0)
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
    // IRQ control
    output       irq_req_o,
    input        irq_ack_i,
    // External connection
    output       mux_en_o,
    // SPI
    input        clk2x_i,
    output       sclk_o,
    input        miso_i,
    output       mosi_o);

// Registers
reg  [7:0] spc_r=0;
wire [7:0] sps;
// Address decoding
wire spcr_sel; // Control
wire spsr_sel; // Status
wire spdr_sel; // Data
// SPI core signals
wire start;
wire [7:0] rx_data;
wire busy;
wire irq_req;
wire irq_ack;
// SPI core configuration
`define cpol spc_r[3]   // Clock Polarity
`define dord spc_r[5]   // Data Order
`define cpha spc_r[2]   // Clock Phase
`define spr  spc_r[1:0] // Rate
reg  spi2x_r=0; // 2x rate
// Rate generator
wire spi_clk; // SPI clock enable
reg  spi_ena_p; // just logic
reg  spi_ena_2;
wire ena_div1;
wire ena_div4;
wire ena_div16;
wire ena_div32;
wire ena_div2x;
reg  [4:0] cnt_r=0;
// Other signals
`define spif irq_req  // Interrupt Flag
`define spie spc_r[7] // Interrupt Enable
// SPI Enable
`define spe  spc_r[6]
reg  wcol_r=0; // Write Collision

`include "../core/avr_constants.v"

assign spcr_sel=adr_i==SPCR_ADDRESS && ENABLE;
assign spsr_sel=adr_i==SPSR_ADDRESS && ENABLE;
assign spdr_sel=adr_i==SPDR_ADDRESS && ENABLE;

assign selected_o=(spcr_sel | spsr_sel | spdr_sel) & (re_i | we_i);

// SPE: SPI enable
assign mux_en_o=`spe;
// The IRQ flag is in the SPI device
assign irq_req_o=`spif && `spie;
// The IRQ is ack'd reading SPDR or by CPU ack
// Note: writing to the register also clears the flag
// TODO: I think we must check a read from SPSR
assign irq_ack=irq_ack_i | (spdr_sel & (re_i | we_i) & ena_i);
// Start Tx
assign start=spdr_sel & we_i & ena_i;

SPI_Master #(.DATA_W(8)) SPI_Core
   (// System
    .clk_i(clk2x_i), .rst_i(rst_i), .ena_i(spi_clk),
    // Interface
    .start_i(start), .tx_i(data_i), .rx_o(rx_data),
    .busy_o(busy), .irq_o(irq_req), .ack_i(irq_ack),
    // Mode options
    .cpol_i(`cpol), .dord_i(`dord), .cpha_i(`cpha),
    // SPI
    .sclk_o(sclk_o), .miso_i(miso_i), .mosi_en_o(open),
    .mosi_o(mosi_o));

always @(posedge clk_i)
begin : do_regs
  if (rst_i)
     begin
     spc_r   <= 0;
     wcol_r  <= 0;
     spi2x_r <= 0;
     end
  else // rst_i
     begin
     // CPU Write to registers
     if (we_i && ena_i)
        begin
        if (spcr_sel)
           spc_r <= data_i;
        if (spsr_sel)
           spi2x_r <= data_i[0];
        end
     // WCOL flag
     if (WCOL_ENABLE)
        begin
        // Write Collision
        if (start && busy)
           wcol_r <= 1;
        else if (spdr_sel && re_i && ena_i)
           // Reading SPDR this flag is cleared
           // TODO: I think we must check a read from SPSR
           wcol_r <= 0;
        end // WCOL_ENABLE
     end // else rst_i
end // do_regs

assign sps={`spif,wcol_r,5'b0,spi2x_r};
assign data_o=(spc_r   & {8{spcr_sel}}) |
              (sps     & {8{spsr_sel}}) |
              (rx_data & {8{spdr_sel}});

// Rate generator
// Clock divider /2 to /32
always @(posedge clk2x_i)
begin : do_cnt_r
  if (rst_i || start)
     cnt_r <= 0;
  else
     cnt_r <= cnt_r+1;
end // do_cnt_r

// SPR selection
assign ena_div1 =1;
assign ena_div4 =cnt_r[1:0]==2'b11;
assign ena_div16=cnt_r[3:0]==4'b1111;
assign ena_div32=cnt_r[4:0]==5'b11111;

always @(`spr or ena_div1 or ena_div4 or ena_div16 or ena_div32)
begin
  case (`spr)
    2'b00:   spi_ena_p=ena_div1;  // Direct
    2'b01:   spi_ena_p=ena_div4;  // /4
    2'b10:   spi_ena_p=ena_div16; // /16
    default: spi_ena_p=ena_div32; // "11" 1/32
  endcase
end

// Extra /2
always @(posedge clk2x_i)
begin : do_spi_clk_2
  if (rst_i || start)
     spi_ena_2 <= 0;
  else if (spi_ena_p)
     spi_ena_2 <= !spi_ena_2;
end // do_spi_clk_2
assign ena_div2x=spi_ena_2 && spi_ena_p;

// SPI2X selector
assign spi_clk=spi2x_r ? spi_ena_p : ena_div2x;

endmodule // SPI_Dev

