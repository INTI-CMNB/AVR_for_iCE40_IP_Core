------------------------------------------------------------------------------
----                                                                      ----
----  AVR Devices prototypes                                              ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  Package for the internal peripherals                                ----
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
---- Design unit:      Devices (Package)                                  ----
---- File name:        dev_pkg.vhdl                                       ----
---- Note:             None                                               ----
---- Limitations:      None known                                         ----
---- Errors:           None known                                         ----
---- Library:          avr                                                ----
---- Dependencies:     IEEE.std_logic_1164                                ----
----                   IEEE.numeric_std                                   ----
----                   avr.Constants                                      ----
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
use avr.Constants.all;

package Devices is
   component IOPort is
      generic(
         NUMBER      : natural:=0;     -- A=0, B=1, etc.
         ENA_OUT     : std_logic:='1'; -- Enable outputs
         ENA_IN      : std_logic:='1'; -- Enable inputs
         BITS        : positive:=8);   -- How many bits are implemented
      port(
         -- AVR Control
         clk_i      : in   std_logic;
         rst_i      : in   std_logic;
         ena_i      : in   std_logic;
         adr_i      : in   unsigned(5 downto 0);
         data_i     : in   std_logic_vector(7 downto 0);
         data_o     : out  std_logic_vector(7 downto 0);
         re_i       : in   std_logic;
         we_i       : in   std_logic;
         selected_o : out  std_logic;
         -- External connection
         port_i     : in   std_logic_vector(BITS-1 downto 0);
         port_o     : out  std_logic_vector(BITS-1 downto 0));
   end component IOPort;

   component WBControl is
      generic(
         ADDR_REG   : unsigned(5 downto 0):=WB_ADDRESS;
         DATA_REG   : unsigned(5 downto 0):=WB_DATA);
      port(
         -- AVR Control
         rst_i      : in   std_logic; -- Reset
         clk_i      : in   std_logic; -- Clock
         ena_o      : out  std_logic;
         -- I/O Bus
         adr_i      : in   unsigned(5 downto 0);
         data_i     : in   std_logic_vector(7 downto 0);
         data_o     : out  std_logic_vector(7 downto 0);
         re_i       : in   std_logic;
         we_i       : in   std_logic;
         selected_o : out  std_logic;
         -- WISHBONE side
         wb_adr_o   : out   std_logic_vector(7 downto 0); -- I/O Address
         wb_dat_o   : out   std_logic_vector(7 downto 0); -- Data Bus output
         wb_dat_i   : in    std_logic_vector(7 downto 0):="00000000"; -- Data Bus input
         wb_stb_o   : out   std_logic; -- Strobe output
         wb_we_o    : out   std_logic; -- Write Enable output
         wb_ack_i   : in    std_logic  -- Acknowledge input
           );
   end component WBControl;

   component IRQCtrl is
      generic(
         ENABLE : std_logic:='1');
      port(
         -- AVR Control
         clk_i      : in   std_logic;
         rst_i      : in   std_logic;
         ena_i      : in   std_logic;
         adr_i      : in   unsigned(5 downto 0);
         data_i     : in   std_logic_vector(7 downto 0);
         data_o     : out  std_logic_vector(7 downto 0);
         re_i       : in   std_logic;
         we_i       : in   std_logic;
         selected_o : out  std_logic;
         -- External connection
          -- 3 peripherals
         dev_irq_i  : in   std_logic_vector(2 downto 0);
          -- 2 PINs
         pin_irq_i  : in   std_logic_vector(1 downto 0);
         irq_ack_i  : in   std_logic_vector(1 downto 0);
          -- 5 lines for the CPU
         ext_irq_o  : out  std_logic_vector(4 downto 0));
   end component IRQCtrl;

   component SPI_Dev is
      generic(
         ENABLE      : std_logic:='1';
         WCOL_ENABLE : std_logic:='0');
      port(
         -- AVR Control
         clk_i      : in   std_logic;
         rst_i      : in   std_logic;
         ena_i      : in   std_logic;
         adr_i      : in   unsigned(5 downto 0);
         data_i     : in   std_logic_vector(7 downto 0);
         data_o     : out  std_logic_vector(7 downto 0);
         re_i       : in   std_logic;
         we_i       : in   std_logic;
         selected_o : out  std_logic;
         -- IRQ control
         irq_req_o  : out  std_logic;
         irq_ack_i  : in   std_logic:='0';
         -- External connection
         mux_en_o   : out  std_logic;
         -- SPI
         clk2x_i    : in   std_logic;
         sclk_o     : out  std_logic;
         miso_i     : in   std_logic;
         mosi_o     : out  std_logic);
   end component SPI_Dev;
end package Devices;
