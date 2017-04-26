------------------------------------------------------------------------------
----                                                                      ----
----  WISHBONE bridge control                                             ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  This is a very simple WISHBONE bridge peripheral. It contains 2     ----
----  registers: WB_ADDRESS and WB_DATA. The CPU must load the desired    ----
----  address in WB_ADDDRESS and then operate on WB_DATA. This will be    ----
----  translated into a WISHBONE transaction. The ena_o output is used    ----
----  to introduce wait states. Note this peripheral doesn't have ena_i   ----
----  this signal should go directly to all WISHBONE peripherals.         ----
----                                                                      ----
----  To Do:                                                              ----
----  -                                                                   ----
----                                                                      ----
----  Author:                                                             ----
----    - Salvador E. Tropea, salvador en inti.gob.ar                     ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Copyright (c) 2009 Salvador E. Tropea <salvador en inti.gob.ar>      ----
---- Copyright (c) 2009 Instituto Nacional de Tecnología Industrial       ----
----                                                                      ----
---- Distributed under the GPL v2 or newer license                        ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Design unit:      WBControl (RTL) (Entity and architecture)          ----
---- File name:        wb_ctrl.vhdl                                       ----
---- Note:             None                                               ----
---- Limitations:      None known                                         ----
---- Errors:           None known                                         ----
---- Library:          avr                                                ----
---- Dependencies:     IEEE.std_logic_1164                                ----
----                   IEEE.numeric_std                                   ----
---- Target FPGA:      iCE40HX4K-TQ144                                    ----
---- Language:         VHDL                                               ----
---- Wishbone:         MASTER (rev B.3)                                   ----
---- Synthesis tools:  Lattice iCECube2 2016.02.27810                     ----
---- Simulation tools: GHDL [Sokcho edition] (0.2x)                       ----
---- Text editor:      SETEdit 0.5.x                                      ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Wishbone Datasheet                                                   ----
----                                                                      ----
----  1 Revision level                      B.3                           ----
----  2 Type of interface                   MASTER                        ----
----  3 Defined signal names                RST_I => rst_i                ----
----                                        CLK_I => clk_i                ----
----                                        ADR_O => wb_adr_o             ----
----                                        DAT_I => wb_dat_i             ----
----                                        DAT_O => wb_dat_o             ----
----                                        WE_O  => wb_we_o              ----
----                                        STB_O => wb_stb_o             ----
----                                        ACK_I => wb_ack_i             ----
----  4 ERR_I                               Unsupported                   ----
----  5 RTY_I                               Unsupported                   ----
----  6 TAGs                                None                          ----
----  7 Port size                           8-bit                         ----
----  8 Port granularity                    8-bit                         ----
----  9 Maximum operand size                8-bit                         ----
---- 10 Data transfer ordering              N/A                           ----
---- 11 Data transfer sequencing            Undefined                     ----
---- 12 Constraints on the CLK_I signal     None                          ----
----                                                                      ----
---- Notes: SEL_O isn't needed because size==granularity                  ----
----                                                                      ----
------------------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

library avr;
use avr.Constants.all;

entity WBControl is
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
end entity WBControl;

architecture RTL of WBControl is
   signal addr_r   : std_logic_vector(7 downto 0);
   signal addr_sel : std_logic;
   signal data_sel : std_logic;
   signal strobe   : std_logic;
begin
   addr_sel   <= '1' when adr_i=ADDR_REG else '0';
   data_sel   <= '1' when adr_i=DATA_REG else '0';
   selected_o <= addr_sel or data_sel;
   wb_dat_o   <= data_i;
   data_o     <= (addr_r and addr_sel) or
                 (wb_dat_i and data_sel);
   strobe     <= data_sel and (re_i or we_i);
   wb_stb_o   <= strobe;
   wb_we_o    <= data_sel and we_i;
   ena_o      <= '0' when strobe='1' and wb_ack_i='0' else '1';
   wb_adr_o   <= addr_r;

   do_addr_r:
   process (clk_i)
   begin
      if rising_edge(clk_i) then
         if rst_i='1' then
            addr_r <= (others => '0');
         elsif addr_sel='1' and we_i='1' then
            addr_r <= data_i;
         end if;
      end if;
   end process do_addr_r;
end architecture RTL; -- Entity: WBControl

