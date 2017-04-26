------------------------------------------------------------------------------
----                                                                      ----
----  AVR CPUs prototypes                                                 ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  Package for the microcontrollers                                    ----
----                                                                      ----
----  To Do:                                                              ----
----  -                                                                   ----
----                                                                      ----
----  Author:                                                             ----
----  Salvador E. Tropea                                                  ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Copyright (c) 2016-2017  <salvador en inti.gob.ar>                   ----
---- Copyright (c) 2016-2017 Instituto Nacional de Tecnología Industrial  ----
----                                                                      ----
---- Distributed under the GPL v2 or newer license                        ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Design unit:      Micros (Package)                                   ----
---- File name:        micros_pkg.vhdl                                    ----
---- Note:             None                                               ----
---- Limitations:      None known                                         ----
---- Errors:           None known                                         ----
---- Library:          avr                                                ----
---- Dependencies:     IEEE.std_logic_1164                                ----
----                   IEEE.numeric_std                                   ----
----                   avr.Types                                          ----
---- Target FPGA:      iCE40HX4K-TQ144                                    ----
---- Language:         VHDL                                               ----
---- Wishbone:         None                                               ----
---- Synthesis tools:  Lattice iCECube2 2016.02.27810                     ----
---- Simulation tools: GHDL [Sokcho edition] (0.2x)                       ----
---- Text editor:      SETEdit 0.5.x                                      ----
----                                                                      ----
------------------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
library avr;
use avr.Types.all;

package Micros is
   component ATtX5 is
      generic(
         ENA_TC0       : boolean:=true;   -- Include the Timer/Counter 0
         ENA_PORTB     : boolean:=true;   -- Include PortB
         ENA_PORTC     : boolean:=false;  -- Include PortC (experimental)
         ENA_PORTD     : boolean:=false;  -- Include PortD (experimental)
         ENA_WB        : boolean:=true;   -- Include the WISHBONE bridge
         ENA_IRQ_CTRL  : boolean:=false;  -- Include the ext. irq. control
         ENA_DEBUG     : boolean:=false;  -- Enable debug interface
         ENA_SPM       : boolean:=false;  -- Implement the SPM instruction (AVR4)
         PORTB_SIZE    : positive:=5;     -- PORTB implemented bits
         PORTC_SIZE    : positive:=6;     -- PORTC implemented bits
         PORTD_SIZE    : positive:=8;     -- PORTD implemented bits
         ENA_SPI       : boolean:=false;  -- Include SPI
         RF_ENA_RST    : boolean:=false;  -- Register File Reset Enable
         RAM_ADDR_W    : positive:=8;     -- tn25: 7 45: 8 85: 9 (128 to 512 b)
         RESET_JUMP    : natural:=0);     -- Address of the reset vector
      port(
         clk_i      : in    std_logic;
         clk2x_i    : in    std_logic;
         rst_i      : in    std_logic;
         ena_i      : in    std_logic:='1'; -- CPU clock enable
         tmr_ena_i  : in    std_logic:='1'; -- T/C clock enable
         -- Ports
         portb_i    : in    std_logic_vector(PORTB_SIZE-1 downto 0):=(others => '0');
         portc_i    : in    std_logic_vector(PORTC_SIZE-1 downto 0):=(others => '0');
         portd_i    : in    std_logic_vector(PORTD_SIZE-1 downto 0):=(others => '0');
         portb_o    : out   std_logic_vector(PORTB_SIZE-1 downto 0);
         portc_o    : out   std_logic_vector(PORTC_SIZE-1 downto 0);
         portd_o    : out   std_logic_vector(PORTD_SIZE-1 downto 0);
         -- Program Memory
         pc_o       : out   unsigned(15 downto 0); -- PROM address
         inst_i     : in    std_logic_vector(15 downto 0); -- PROM data
         inst_o     : out   std_logic_vector(15 downto 0); -- PROM data
         pgm_we_o   : out   std_logic; -- PROM WE
         -- External device interrupts (external UART, Timer, etc.)
         dev_irq_i  : in    std_logic_vector(2 downto 0):="000";
         dev_ack_o  : out   std_logic_vector(2 downto 0);
         -- External PIN interrupts
         pin_irq_i  : in    std_logic_vector(1 downto 0):="00";
         -- WISHBONE
         wb_adr_o   : out   std_logic_vector(7 downto 0); -- I/O Address
         wb_dat_o   : out   std_logic_vector(7 downto 0); -- Data Bus output
         wb_dat_i   : in    std_logic_vector(7 downto 0):="00000000"; -- Data Bus input
         wb_stb_o   : out   std_logic;  -- Strobe output
         wb_we_o    : out   std_logic;  -- Write Enable output
         wb_ack_i   : in    std_logic:='1'; -- Acknowledge input
         -- SPI
         spi_ena_o  : out   std_logic;
         sclk_o     : out   std_logic;
         miso_i     : in    std_logic:='0';
         mosi_o     : out   std_logic;
         -- Debug
         dbg_o      : out debug_o_t;  -- Debug status
         dbg_i      : in  debug_i_t:=DEBUG_I_INIT); -- Debug control
   end component ATtX5;
end package Micros;

