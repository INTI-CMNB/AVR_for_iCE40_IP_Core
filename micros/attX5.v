/***********************************************************************

  AVR 2.5 CPU with some aspects of ATtinyX5 CPUs

  This file is part FPGA Libre project http://fpgalibre.sf.net/

  Description:
  ATtiny25/45/85 style CPU. Not exactly the same, just the same
  memory layout and instruction set.
  It also implements the SPM instruction.
  Ports B, C and D are available.

  To Do:
  -

  Author:
  Salvador E. Tropea

------------------------------------------------------------------------------

 Copyright (c) 2016-2017  <salvador en inti.gob.ar>
 Copyright (c) 2016-2017 Instituto Nacional de Tecnología Industrial

 Distributed under the GPL v2 or newer license

------------------------------------------------------------------------------

 Design unit:      ATtX5
 File name:        attX5.v
 Note:             None
 Limitations:      None known
 Errors:           None known
 Library:          avr
 Dependencies:     IEEE.std_logic_1164
                   IEEE.numeric_std
                   avr.Core
                   avr.Constants
                   avr.Types
                   avr.Devices
                   avr.Memory
 Target FPGA:      iCE40HX4K-TQ144
 Language:         Verilog
 Wishbone:         None
 Synthesis tools:  Lattice iCECube2 2016.02.27810
 Simulation tools: GHDL [Sokcho edition] (0.2x)
 Text editor:      SETEdit 0.5.x

***********************************************************************/

module ATtX5
  #(
    parameter ENA_PORTB=1,    // Include PortB
    parameter ENA_PORTC=0,    // Include PortC (experimental)
    parameter ENA_PORTD=0,    // Include PortD (experimental)
    parameter ENA_WB=1,       // Include the WISHBONE bridge
    parameter ENA_IRQ_CTRL=0, // Include the ext. irq. control
    parameter ENA_DEBUG=0,    // Enable debug interface
    parameter ENA_SPM=0,      // Implement the SPM instruction (AVR4)
    parameter ENA_AVR25=1,    // Enable AVR 2.5 instructions
    parameter PORTB_SIZE=5,   // PORTB implemented bits
    parameter PORTC_SIZE=6,   // PORTC implemented bits
    parameter PORTD_SIZE=8,   // PORTD implemented bits
    parameter ENA_SPI=0,      // Include SPI
    parameter RAM_ADDR_W=8,   // tn25: 7 45: 8 85: 9 (128 to 512 b)
    parameter RESET_JUMP=0)   // Address of the reset vector
   (
    input clk_i, 
    input clk2x_i, 
    input rst_i, 
    input ena_i, // CPU clock enable
    // Ports
    input  [PORTB_SIZE-1:0] portb_i,
    input  [PORTC_SIZE-1:0] portc_i,
    input  [PORTD_SIZE-1:0] portd_i,
    output [PORTB_SIZE-1:0] portb_o,
    output [PORTC_SIZE-1:0] portc_o,
    output [PORTD_SIZE-1:0] portd_o,
    // Program Memory
    output [15:0] pc_o,     // PROM address
    input  [15:0] inst_i,   // PROM data
    output [15:0] inst_o,   // PROM data
    output        pgm_we_o, // PROM WE
    // External device interrupts (external UART, Timer, etc.)
    input  [2:0]  dev_irq_i,
    output [2:0]  dev_ack_o,
    // External PIN interrupts
    input  [1:0]  pin_irq_i,
    // WISHBONE
    output [7:0]  wb_adr_o, // I/O Address
    output [7:0]  wb_dat_o, // Data Bus output
    input  [7:0]  wb_dat_i, // Data Bus input
    output        wb_stb_o, // Strobe output
    output        wb_we_o,  // Write Enable output
    input         wb_ack_i, // Acknowledge input
    // SPI
    output        spi_ena_o,
    output        sclk_o,
    input         miso_i,
    output        mosi_o,
    // Debug
    input         dbg_stop_i, // Stop request
    output [15:0] dbg_pc_o,
    output [15:0] dbg_inst_o,
    output [15:0] dbg_inst2_o,
    output        dbg_exec_o,
    output        dbg_is32_o,
    output        dbg_stopped_o, // CPU is stopped
    // Debug used for Test_ALU_1_TB
    input         dbg_rf_fake_i,
    input  [7:0]  dbg_rr_data_i,
    input  [7:0]  dbg_rd_data_i,
    output [7:0]  dbg_rd_data_o,
    output        dbg_rd_we_o,
    output        dbg_cyc_last_o); // Last cycle in the instruction

localparam EXT_BUS_W=6; // 6 peripherals
localparam IRQ_NUM=5;   // Number of IRQ lines
localparam ID_W=3;      // Width of the ID
// i.e. 512 => 9 bits, but from 0x60 to 0x25F => 10 bits
localparam ADR_W=RAM_ADDR_W+1;

// I/O registers
wire [5:0] io_adr;
wire       io_re;
wire       io_we;
// Data bus
reg  [7:0] core_din; // Logic
wire [7:0] core_dout;
// Interrupts
wire [IRQ_NUM-1:0] irq_lines;
wire [IRQ_NUM-1:0] irq_acks;
// RAM
wire [7:0]            ram_datao;
wire [RAM_ADDR_W:0]   ram_adr;
wire [RAM_ADDR_W-1:0] ram_adr_2;
wire                  ram_we;
wire                  ram_re;
// I/O mux
wire [7:0]           io_out[0:EXT_BUS_W-1];
wire [EXT_BUS_W-1:0] io_out_en;
// WISHBONE
wire wb_go_on;
wire cpu_ena;
// Watchdog
wire cpu_rst;

AVRCore
   #(
     .ID_W(ID_W), .IRQ_LINES(IRQ_NUM),
     .ENA_RAMPZ(0), .SP_W(ADR_W), .RAM_ADR_W(ADR_W), .ENA_AVR3(0),
     .ENA_DEBUG(ENA_DEBUG), .ENA_SPM(ENA_SPM), .RESET_JUMP(RESET_JUMP),
     .ENA_AVR4(0), .ENA_AVR25(ENA_AVR25))
   (
     //Clock and reset
     .clk_i(clk_i), .ena_i(cpu_ena), .rst_i(rst_i),
     // Program Memory
     .pc_o(pc_o), .inst_i(inst_i), .inst_o(inst_o), .pgm_we_o(pgm_we_o),
     // I/O control
     .io_adr_o(io_adr), .io_re_o(io_re), .io_we_o(io_we),
     // Data memory control
     .ram_adr_o(ram_adr), .ram_re_o(ram_re), .ram_we_o(ram_we),
     // Data paths
     .io_data_i(core_din), .data_o(core_dout),
     .ram_data_i(ram_datao),
     // Interrupts
     .irq_lines_i(irq_lines), .irq_ok_o(), .irq_acks_o(irq_acks),
     .irq_ena_o(),
     // Sleep Control
     .sleep_o(),
     //Watchdog
     .wdr_o(),
     // Debug
     .dbg_stop_i(dbg_stop_i),
     .dbg_pc_o(dbg_pc_o),
     .dbg_inst_o(dbg_inst_o),
     .dbg_inst2_o(dbg_inst2_o),
     .dbg_exec_o(dbg_exec_o),
     .dbg_is32_o(dbg_is32_o),
     .dbg_stopped_o(dbg_stopped_o),
     // Debug used for Test_ALU_1_TB
     .dbg_rf_fake_i(dbg_rf_fake_i),
     .dbg_rr_data_i(dbg_rr_data_i),
     .dbg_rd_data_i(dbg_rd_data_i),
     .dbg_rd_data_o(dbg_rd_data_o),
     .dbg_rd_we_o(dbg_rd_we_o),
     .dbg_cyc_last_o(dbg_cyc_last_o));
assign cpu_ena=wb_go_on & ena_i;
assign cpu_rst=rst_i;

reg [7:0] do;
integer i;
//always @(io_out_en or io_out)
always @*
begin : do_mux
  do=0;
  for (i=EXT_BUS_W-1; i>=0; i=i-1)
      if (io_out_en[i])
         do=io_out[i];
  core_din <= do;
end // do_mux

// Used IRQ lines (add 2 for the "vector number"), address=(irq+1)*2
// 0 External PIN INT0
// 1 External PIN INT1
// 2 External Device Interrupt 0
// 3 External Device Interrupt 1
// 4 External Device Interrupt 2 (16 bits Timer)
assign dev_ack_o=irq_acks[4:2];

// Used io_out_en
// 0 PortB
// 1 PortC
// 2 WB bridge
// 3 IRQ Ctrl
// 4 PortD for experimental use
// 5 SPI

///////////
// PortB //
///////////
generate
if (ENA_PORTB)
   begin : portb_impl
   IOPort #(.NUMBER(1), .BITS(PORTB_SIZE)) portb_comp
      (// AVR Control
       .clk_i(clk_i),
       .rst_i(rst_i),
       .ena_i(ena_i),
       .adr_i(io_adr),
       .data_i(core_dout),
       .data_o(io_out[0]),
       .re_i(io_re),
       .we_i(io_we),
       .selected_o(io_out_en[0]),
       // External connection
       .port_i(portb_i),
       .port_o(portb_o));
   end
else
   begin : portb_not_impl
   assign portb_o={PORTB_SIZE{1'bZ}};
   end
endgenerate
// ************************************************

///////////
// PortC //
///////////
generate
if (ENA_PORTC)
   begin : portc_impl
   IOPort #(.NUMBER(2), .BITS(PORTC_SIZE)) portc_comp
      (// AVR Control
       .clk_i(clk_i),
       .rst_i(rst_i),
       .ena_i(ena_i),
       .adr_i(io_adr),
       .data_i(core_dout),
       .data_o(io_out[1]),
       .re_i(io_re),
       .we_i(io_we),
       .selected_o(io_out_en[1]),
       // External connection
       .port_i(portc_i),
       .port_o(portc_o));
   end
else
   begin : portc_not_impl
   assign portc_o={PORTC_SIZE{1'bZ}};
   end
endgenerate
// ************************************************

///////////
// PortD //
///////////
generate
if (ENA_PORTD)
   begin : portd_impl
   IOPort #(.NUMBER(3), .BITS(PORTD_SIZE)) portd_comp
      (// AVR Control
       .clk_i(clk_i),
       .rst_i(rst_i),
       .ena_i(ena_i),
       .adr_i(io_adr),
       .data_i(core_dout),
       .data_o(io_out[4]),
       .re_i(io_re),
       .we_i(io_we),
       .selected_o(io_out_en[4]),
       // External connection
       .port_i(portd_i),
       .port_o(portd_o));
   end
else
   begin : portd_not_impl
   assign portd_o={PORTD_SIZE{1'bZ}};
   end
endgenerate
// ************************************************

/////////////////////
// WISHBONE bridge //
/////////////////////
generate
if (ENA_WB)
   begin : WB_Impl
   WBControl wb_bridge
      (// AVR Control
       .rst_i(rst_i),
       .clk_i(clk_i),
       .ena_o(wb_go_on),
       // I/O Bus
       .adr_i(io_adr),
       .data_i(core_dout),
       .data_o(io_out[2]),
       .re_i(io_re),
       .we_i(io_we),
       .selected_o(io_out_en[2]),
       // WISHBONE side
       .wb_adr_o(wb_adr_o), .wb_dat_o(wb_dat_o), .wb_dat_i(wb_dat_i),
       .wb_stb_o(wb_stb_o), .wb_we_o(wb_we_o),  .wb_ack_i(wb_ack_i));
   end
endgenerate

/////////////////////////////////
// External Interrupts Control //
/////////////////////////////////
IRQCtrl #(.ENABLE(ENA_IRQ_CTRL)) irq_ctrl
   (// AVR Control
    .clk_i(clk_i),
    .rst_i(rst_i),
    .ena_i(ena_i),
    .adr_i(io_adr),
    .data_i(core_dout),
    .data_o(io_out[3]),
    .re_i(io_re),
    .we_i(io_we),
    .selected_o(io_out_en[3]),
    // External connection
     // 3 peripherals
    .dev_irq_i(dev_irq_i),
     // 2 PINs
    .pin_irq_i(pin_irq_i),
    .irq_ack_i(irq_acks[1:0]),
     // 5 lines for the CPU
    .ext_irq_o(irq_lines));

/////////
// SPI //
/////////
generate
if (ENA_SPI)
   begin : SPI_Impl
   SPI_Dev SPI_Device
      (// AVR Control
       .clk_i(clk_i),
       .rst_i(rst_i),
       .ena_i(ena_i),
       .adr_i(io_adr),
       .data_i(core_dout),
       .data_o(io_out[5]),
       .re_i(io_re),
       .we_i(io_we),
       .selected_o(io_out_en[5]),
       // IRQ control. Currently disabled
       .irq_req_o(),
       .irq_ack_i(0),
       // External connection
       .mux_en_o(spi_ena_o),
       // SPI
       .clk2x_i(clk2x_i),
       .sclk_o(sclk_o),
       .miso_i(miso_i),
       .mosi_o(mosi_o));
   end
else
   begin : SPI_Not_Impl
   assign spi_ena_o=0;
   assign sclk_o   =0;
   assign mosi_o   =0;
   end
endgenerate

////////////
// Memory //
////////////
// Memory configuration (DM 128/256/512x8)
SinglePortRAM #(.WORD_SIZE(8), .ADDR_W(RAM_ADDR_W)) DRAM_Inst
   (
    .clk_i(clk_i), .we_i(ram_we),
    .addr_i(ram_adr_2),
    .d_i(core_dout), .d_o(ram_datao));
// ram_adr valid values range from 0x60 to 0x0DF we use the 7 LSBs
// ram_adr valid values range from 0x60 to 0x15F we use the 8 LSBs
// ram_adr valid values range from 0x60 to 0x25F we use the 9 LSBs
assign ram_adr_2=ram_adr[RAM_ADDR_W-1:0];

endmodule // ATtX5

