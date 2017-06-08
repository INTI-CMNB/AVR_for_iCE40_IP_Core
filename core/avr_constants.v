/***********************************************************************

  AVR CPU core constants, types and prototypes

  This file is part FPGA Libre project http://fpgalibre.sf.net/

  Description:
  Package for the CPU core

  To Do:
  -

  Author:
  Salvador E. Tropea

------------------------------------------------------------------------------

 Copyright (c) 2016-2017  <salvador en inti.gob.ar>
 Copyright (c) 2016-2017 Instituto Nacional de Tecnología Industrial

 Distributed under the GPL v2 or newer license

------------------------------------------------------------------------------

 Design unit:      Constants (Package)
 File name:        avr_constants.v
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

// I/O Register File (Internal)
`define RAMPZ_ADDRESS  6'h3B
`define SPL_ADDRESS    6'h3D
`define SPH_ADDRESS    6'h3E
`define SREG_ADDRESS   6'h3F
// Interrupt Control
// ATmega8, ATtiny22
`define GICR_ADDRESS   6'h3B
`define GIFR_ADDRESS   6'h3A
// ATmega103
`define EIMSK_ADDRESS  6'h39
`define EIFR_ADDRESS   6'h38
`define EICR_ADDRESS   6'h3A
// UART
`define UDR_ADDRESS    6'h0C
`define UBRR_ADDRESS   6'h09
`define USR_ADDRESS    6'h0B
`define UCR_ADDRESS    6'h0A
// Timer/Counter
`define TCCR0_ADDRESS  6'h33
`define TCCR1A_ADDRESS 6'h2F
`define TCCR1B_ADDRESS 6'h2E
`define TCCR2_ADDRESS  6'h25
`define ASSR_ADDRESS   6'h30
`define TIMSK_ADDRESS  6'h37
`define TIFR_ADDRESS   6'h36
`define TCNT0_ADDRESS  6'h32
`define TCNT2_ADDRESS  6'h24
`define OCR0_ADDRESS   6'h31
`define OCR2_ADDRESS   6'h23
`define TCNT1H_ADDRESS 6'h2D
`define TCNT1L_ADDRESS 6'h2C
`define OCR1AH_ADDRESS 6'h2B
`define OCR1AL_ADDRESS 6'h2A
`define OCR1BH_ADDRESS 6'h29
`define OCR1BL_ADDRESS 6'h28
`define ICR1AH_ADDRESS 6'h27
`define ICR1AL_ADDRESS 6'h26
// Service module
`define MCUCR_ADDRESS  6'h35
`define MCUSR_ADDRESS  6'h34
`define XDIV_ADDRESS   6'h3C
// EEPROM 
`define EEARH_ADDRESS  6'h1F
`define EEARL_ADDRESS  6'h1E
`define EEDR_ADDRESS   6'h1D
`define EECR_ADDRESS   6'h1C
// WISHBONE extension, added by SET.
// Uses the EEPROM space.
`define WB_ADDRESS     6'h1F
`define WB_DATA        6'h1E
// WISHBONE extension, added by David Caruso.
// Uses the ADC space, internal wishbone for internal peripherals
`define WB_DATA_INT    6'h05
// SPI
`define SPCR_ADDRESS   6'h0D
`define SPSR_ADDRESS   6'h0E
`define SPDR_ADDRESS   6'h0F
// PORTA
`define PORTA_ADDRESS  6'h1B
`define DDRA_ADDRESS   6'h1A
`define PINA_ADDRESS   6'h19
// PORTB
`define PORTB_ADDRESS  6'h18
`define DDRB_ADDRESS   6'h17
`define PINB_ADDRESS   6'h16
// PORTC
`define PORTC_ADDRESS  6'h15
`define DDRC_ADDRESS   6'h14
`define PINC_ADDRESS   6'h13
// PORTD
`define PORTD_ADDRESS  6'h12
`define DDRD_ADDRESS   6'h11
`define PIND_ADDRESS   6'h10
// PORTE
`define PORTE_ADDRESS  6'h03
`define DDRE_ADDRESS   6'h02
`define PINE_ADDRESS   6'h01
// PORTF (Input only)
`define PINF_ADDRESS   6'h00
// Analog to digital converter
`define ADCL_ADDRESS   6'h04
`define ADCH_ADDRESS   6'h05
`define ADCSR_ADDRESS  6'h06
`define ADMUX_ADDRESS  6'h07
// Analog comparator
`define ACSR_ADDRESS   6'h08
// Watchdog
`define WDTCR_ADDRESS  6'h21

`define LD_X 2'b11
`define LD_Y 2'b10
`define LD_Z 2'b00
`define LD_POST_INC 2'b01
`define LD_PRE_DEC  2'b10


