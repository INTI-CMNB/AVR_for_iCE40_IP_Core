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
localparam [5:0] RAMPZ_ADDRESS  =6'h3B;
localparam [5:0] SPL_ADDRESS    =6'h3D;
localparam [5:0] SPH_ADDRESS    =6'h3E;
localparam [5:0] SREG_ADDRESS   =6'h3F;
// Interrupt Control
// ATmega8, ATtiny22
localparam [5:0] GICR_ADDRESS   =6'h3B;
localparam [5:0] GIFR_ADDRESS   =6'h3A;
// ATmega103
localparam [5:0] EIMSK_ADDRESS  =6'h39;
localparam [5:0] EIFR_ADDRESS   =6'h38;
localparam [5:0] EICR_ADDRESS   =6'h3A;
// UART
localparam [5:0] UDR_ADDRESS    =6'h0C;
localparam [5:0] UBRR_ADDRESS   =6'h09;
localparam [5:0] USR_ADDRESS    =6'h0B;
localparam [5:0] UCR_ADDRESS    =6'h0A;
// Timer/Counter
localparam [5:0] TCCR0_ADDRESS  =6'h33;
localparam [5:0] TCCR1A_ADDRESS =6'h2F;
localparam [5:0] TCCR1B_ADDRESS =6'h2E;
localparam [5:0] TCCR2_ADDRESS  =6'h25;
localparam [5:0] ASSR_ADDRESS   =6'h30;
localparam [5:0] TIMSK_ADDRESS  =6'h37;
localparam [5:0] TIFR_ADDRESS   =6'h36;
localparam [5:0] TCNT0_ADDRESS  =6'h32;
localparam [5:0] TCNT2_ADDRESS  =6'h24;
localparam [5:0] OCR0_ADDRESS   =6'h31;
localparam [5:0] OCR2_ADDRESS   =6'h23;
localparam [5:0] TCNT1H_ADDRESS =6'h2D;
localparam [5:0] TCNT1L_ADDRESS =6'h2C;
localparam [5:0] OCR1AH_ADDRESS =6'h2B;
localparam [5:0] OCR1AL_ADDRESS =6'h2A;
localparam [5:0] OCR1BH_ADDRESS =6'h29;
localparam [5:0] OCR1BL_ADDRESS =6'h28;
localparam [5:0] ICR1AH_ADDRESS =6'h27;
localparam [5:0] ICR1AL_ADDRESS =6'h26;
// Service module
localparam [5:0] MCUCR_ADDRESS  =6'h35;
localparam [5:0] MCUSR_ADDRESS  =6'h34;
localparam [5:0] XDIV_ADDRESS   =6'h3C;
// EEPROM 
localparam [5:0] EEARH_ADDRESS  =6'h1F;
localparam [5:0] EEARL_ADDRESS  =6'h1E;
localparam [5:0] EEDR_ADDRESS   =6'h1D;
localparam [5:0] EECR_ADDRESS   =6'h1C;
// WISHBONE extension, added by SET.
// Uses the EEPROM space.
localparam [5:0] WB_ADDRESS     =6'h1F;
localparam [5:0] WB_DATA        =6'h1E;
// WISHBONE extension, added by David Caruso.
// Uses the ADC space, internal wishbone for internal peripherals
localparam [5:0] WB_DATA_INT    =6'h05;
// SPI
localparam [5:0] SPCR_ADDRESS   =6'h0D;
localparam [5:0] SPSR_ADDRESS   =6'h0E;
localparam [5:0] SPDR_ADDRESS   =6'h0F;
// PORTA
localparam [5:0] PORTA_ADDRESS  =6'h1B;
localparam [5:0] DDRA_ADDRESS   =6'h1A;
localparam [5:0] PINA_ADDRESS   =6'h19;
// PORTB
localparam [5:0] PORTB_ADDRESS  =6'h18;
localparam [5:0] DDRB_ADDRESS   =6'h17;
localparam [5:0] PINB_ADDRESS   =6'h16;
// PORTC
localparam [5:0] PORTC_ADDRESS  =6'h15;
localparam [5:0] DDRC_ADDRESS   =6'h14;
localparam [5:0] PINC_ADDRESS   =6'h13;
// PORTD
localparam [5:0] PORTD_ADDRESS  =6'h12;
localparam [5:0] DDRD_ADDRESS   =6'h11;
localparam [5:0] PIND_ADDRESS   =6'h10;
// PORTE
localparam [5:0] PORTE_ADDRESS  =6'h03;
localparam [5:0] DDRE_ADDRESS   =6'h02;
localparam [5:0] PINE_ADDRESS   =6'h01;
// PORTF (Input only)
localparam [5:0] PINF_ADDRESS   =6'h00;
// Analog to digital converter
localparam [5:0] ADCL_ADDRESS   =6'h04;
localparam [5:0] ADCH_ADDRESS   =6'h05;
localparam [5:0] ADCSR_ADDRESS  =6'h06;
localparam [5:0] ADMUX_ADDRESS  =6'h07;
// Analog comparator
localparam [5:0] ACSR_ADDRESS   =6'h08;
// Watchdog
localparam [5:0] WDTCR_ADDRESS  =6'h21;


