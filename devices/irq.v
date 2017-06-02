/***********************************************************************

  External Interrupt Control stuff [GICR style for Lattuino]

  This file is part FPGA Libre project http://fpgalibre.sf.net/

  Description:
  Implements the external interrupts control registers: GICR, GIFR
  and some bits from MCUCR. Used by ATmega8, ATtiny22 and similar
  AVRs. Note that ATmega103 uses EIMSK and other registers for the
  same purpose.@p
  Note that when this module is disabled the external irq lines at
  the input are copied to the output (irqs are active high). But when
  enabled, and irqs by level are selected, the external irq lines are
  negated (irqs are active low).@p
  This file contains some extensions specific for Lattuino:
  GICR/GIFR:
  B7/B6/B5 External Devices
  B4       ExtINT1
  B3       ExtINT0
  MCUCR: B3/B2 ExtINT1 Mode B1/B0 ExtINT0 Mode@p

  To Do:
  -

  Author:
    - Salvador E. Tropea, salvador en inti.gob.ar

------------------------------------------------------------------------------

 Copyright (c) 2009-2017 Salvador E. Tropea <salvador en inti.gob.ar>
 Copyright (c) 2009-2017 Instituto Nacional de Tecnología Industrial

 Distributed under the GPL v2 or newer license

------------------------------------------------------------------------------

 Design unit:      IRQCtrl
 File name:        irq.v
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

module IRQCtrl
  #(
    parameter ENABLE=1)
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
    // 3 peripherals
    input  [2:0] dev_irq_i,
    // 2 PINs
    input  [1:0] pin_irq_i,
    input  [1:0] irq_ack_i,
    // 5 lines for the CPU
    output [4:0] ext_irq_o);

// Registers
reg [4:0] gic_r=0;  // B7/B6+B5/B4/B3
reg [1:0] gif_r=0;  // B7/B6
reg [3:0] mcuc_r=0; // B3..B0
reg [1:0] ext_irq_r=0; // Previous IRQ
`define isc0_r mcuc_r[1:0]
`define isc1_r mcuc_r[3:2]
// Address decoding
wire gicr_sel;
wire gifr_sel;
wire mcucr_sel;

assign gicr_sel =adr_i==`GICR_ADDRESS  && ENABLE;
assign gifr_sel =adr_i==`GIFR_ADDRESS  && ENABLE;
assign mcucr_sel=adr_i==`MCUCR_ADDRESS && ENABLE;

assign selected_o=(gicr_sel | gifr_sel | mcucr_sel) & (re_i | we_i);

always @(posedge clk_i)
begin : do_regs
  ext_irq_r <= pin_irq_i; // Previous state
  if (rst_i)
     begin
     gic_r  <= 0;
     gif_r  <= 0;
     mcuc_r <= 0;
     end
  else // rst_i
     begin
     // General Interrupt Flag irq ack
     gif_r <= gif_r & ~irq_ack_i;
     // Write to registers
     if (we_i && ena_i)
        begin
        // Global Interrupt Control [0x3B]
        if (gicr_sel)
           gic_r <= data_i[7:3];
        // General Interrupt Flag [0x3A]
        // Writing 1 clear the flag
        if (gifr_sel)
           gif_r <= gif_r & ~data_i[4:3];
        // MCU Control [0x35]
        if (mcucr_sel)
           begin
           `isc0_r <= data_i[1:0];
           `isc1_r <= data_i[3:2];
           end
        end // we_i && ena_i
     // General Interrupt Flag logic
     case (`isc0_r)
       2'b01: // By change
          if (ext_irq_r[0]!=pin_irq_i[0])
             gif_r[0] <= 1;
       2'b10: // Falling edge
          if (ext_irq_r[0] && !pin_irq_i[0])
             gif_r[0] <= 1;
       2'b11: // Rising edge
          if (!ext_irq_r[0] && pin_irq_i[0])
             gif_r[0] <= 1;
       default: // "00", By level, this flag is always 0
          gif_r[0] <= 0;
     endcase
     case (`isc1_r)
       2'b01: // By change
          if (ext_irq_r[1]!=pin_irq_i[1])
             gif_r[1] <= 1;
       2'b10: // Falling edge
          if (ext_irq_r[1] && !pin_irq_i[1])
             gif_r[1] <= 1;
       2'b11: // Rising edge
          if (!ext_irq_r[1] && pin_irq_i[1])
             gif_r[1] <= 1;
       default: // "00", By level, this flag is always 0
          gif_r[1] <= 0;
     endcase
     end // else rst_i
end // do_regs

assign ext_irq_o[0]=!ENABLE ? pin_irq_i[0] : (
                    `isc0_r==2'b00 ? ~pin_irq_i[0] & gic_r[0] :
                                      gif_r[0] & gic_r[0]);
assign ext_irq_o[1]=!ENABLE ? pin_irq_i[1] : (
                    `isc1_r==2'b00 ? ~pin_irq_i[1] & gic_r[1] :
                                      gif_r[1] & gic_r[1]);
assign ext_irq_o[4:2]=ENABLE ? dev_irq_i & gic_r[4:2] : dev_irq_i;

assign data_o=({gic_r,3'b000}           & {8{gicr_sel}}) |
              ({dev_irq_i,gif_r,3'b000} & {8{gifr_sel}}) |
              ({4'b0000,mcuc_r}         & {8{mcucr_sel}});
endmodule // IRQCtrl

