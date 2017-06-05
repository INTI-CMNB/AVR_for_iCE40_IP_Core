/***********************************************************************

  AVR CPU core (CPU+Satus+Stack Pointer)

  This file is part FPGA Libre project http://fpgalibre.sf.net/

  Description:
  This joins all the CPU with some basic I/O registers, like the
  status register and the stack pointer.

  To Do:
  -

  Author:
  Salvador E. Tropea

------------------------------------------------------------------------------

 Copyright (c) 2008-2017  <salvador en inti.gob.ar>
 Copyright (c) 2008-2017 Instituto Nacional de Tecnología Industrial

 Distributed under the GPL v2 or newer license

------------------------------------------------------------------------------

 Design unit:      AVRCore
 File name:        core.v
 Note:             None
 Limitations:      None known
 Errors:           None known
 Library:          avr
 Dependencies:     IEEE.std_logic_1164
                   IEEE.numeric_std
                   avr.Internal
                   avr.Types
 Target FPGA:      iCE40HX4K-TQ144
 Language:         Verilog
 Wishbone:         None
 Synthesis tools:  Lattice iCECube2 2016.02.27810
 Simulation tools: GHDL [Sokcho edition] (0.2x)
 Text editor:      SETEdit 0.5.x

***********************************************************************/

module AVRCore
  #(
    parameter SP_W=16,       // Stack pointer size (8:16)
    parameter RAM_ADR_W=16,  // RAM address width (8:16)
    parameter ENA_RAMPZ=0,   // RAMPZ enable
    parameter ID_W=5,        // Width of the IRQ ID
    parameter ENA_AVR25=0,   // Enable AVR25 instructions (MOVW/LPM Rd,Z)
    parameter ENA_AVR3=1,    // Enable AVR3 instructions
    parameter ENA_AVR4=0,    // Enable AVR4 instructions
    parameter ENA_SPM=0,     // Enable SPM instructions
    parameter ENA_DEBUG=0,   // Enable debug interface
    parameter RESET_JUMP=0,  // Address of the reset vector
    parameter IRQ_LINES=23)  // Number of IRQ lines
   (
    // Clock and reset
    input  clk_i,
    input  ena_i,
    input  rst_i,
    // Program Memory
    output [15:0] pc_o,
    input  [15:0] inst_i,
    output [15:0] inst_o, 
    output        pgm_we_o,
    // I/O control
    output [5:0] io_adr_o,
    output       io_re_o,
    output       io_we_o,
    input  [7:0] io_data_i,
    // Data memory control
    output [RAM_ADR_W-1:0] ram_adr_o, 
    output                 ram_re_o,
    output                 ram_we_o,
    input  [7:0]           ram_data_i,
    // Data paths
    output [7:0] data_o,
    // Interrupt
    input  [IRQ_LINES-1:0] irq_lines_i,
    output [IRQ_LINES-1:0] irq_acks_o, 
    output                 irq_ok_o,
    output                 irq_ena_o,  // SREG(I)
    // Sleep Instruction
    output sleep_o, 
    // Watchdog Reset Instruction
    output wdr_o, 
    // Debug
    input         dbg_stop_i,  // Stop request
    output [15:0] dbg_pc_o,
    output [15:0] dbg_inst_o, 
    output [15:0] dbg_inst2_o, 
    output        dbg_exec_o,
    output        dbg_is32_o,
    output        dbg_stopped_o,  // CPU is stopped
    // Debug used for Test_ALU_1_TB
    input         dbg_rf_fake_i,
    input  [7:0]  dbg_rr_data_i,
    input  [7:0]  dbg_rd_data_i,
    output [7:0]  dbg_rd_data_o,
    output        dbg_rd_we_o,
    output        dbg_cyc_last_o); // Last cycle in the instruction

// I/O Registers File
// Input Data after the IORegFile block
wire [7:0] io_datai;
// Output Data (also used internally)
wire [7:0] datao;
// I/O control (also used internally)
wire [5:0] io_adr;
wire       io_we;
wire       io_re;
// I/O Registers
wire [7:0] new_sreg;
wire [7:0] sreg;
wire [7:0] sreg_we;
wire [SP_W-1:0] sp;
wire [7:0] rampz;
wire       sp_pop;
wire       sp_we;

CPU
   #(.ENA_DEBUG(ENA_DEBUG), .RAM_ADR_W(RAM_ADR_W),
     .SP_W(SP_W), .ENA_AVR4(ENA_AVR4), .ENA_AVR3(ENA_AVR3),
     .ENA_SPM(ENA_SPM), .RESET_JUMP(RESET_JUMP),
     .ENA_AVR25(ENA_AVR25), .IRQ_ID_W(ID_W),
     .IRQ_LINES(IRQ_LINES))
   CPU
   (// Clock and reset
    .clk_i(clk_i),
    .ena_i(ena_i),
    .rst_i(rst_i),
    // Program memory
    .pc_o(pc_o),
    .inst_i(inst_i),
    .inst_o(inst_o),
    .pgm_we_o(pgm_we_o),
    // I/O control
    .io_adr_o(io_adr),
    .io_re_o(io_re),
    .io_we_o(io_we),
    .io_data_i(io_datai),
    // Data memory control
    .ram_adr_o(ram_adr_o),
    .ram_re_o(ram_re_o),
    .ram_we_o(ram_we_o),
    .ram_data_i(ram_data_i),
    // Data paths
    .data_o(datao),
    // Interrupt
    .irq_lines_i(irq_lines_i),
    .irq_acks_o(irq_acks_o),
    //Sleep 
    .sleep_o(sleep_o),
    .irq_ok_o(irq_ok_o),
    //Watchdog
    .wdr_o(wdr_o),
    // I/O register file interface
    .sreg_o(new_sreg),
    .sreg_i(sreg),
    .sreg_we_o(sreg_we),
    .sp_i(sp),
    .sp_pop_o(sp_pop),
    .sp_we_o(sp_we),
    .rampz_i(rampz),
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

IORegFile #(.ENA_RAMPZ(ENA_RAMPZ), .SP_W(SP_W)) IORegs_Inst
   (// Clock and reset
    .clk_i(clk_i),
    .ena_i(ena_i),
    .rst_i(rst_i),
    // I/O Bus
    .adr_i(io_adr),
    .we_i(io_we),
    .re_i(io_re),
    .data_i(datao),
    .data_o(io_datai),    // Local data bus
    .e_data_i(io_data_i), // External data bus
    // Status Register
    .sreg_i(new_sreg),
    .sreg_o(sreg),
    .sreg_we_i(sreg_we),
    // Stack Pointer
    .sp_o(sp),
    .sp_pop_i(sp_pop),
    .sp_we_i(sp_we),
    // RAM Page Z
    .rampz_o(rampz));

// Outputs
assign io_adr_o=io_adr;
assign io_we_o =io_we;
assign io_re_o =io_re;
assign data_o  =datao;
// Sleep support
assign irq_ena_o=sreg[7]; // I flag

endmodule // AVRCore

