------------------------------------------------------------------------------
----                                                                      ----
----  I/O Port Device                                                     ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  Simple I/O pins device. Takes some ideas from O.C. AVR Core.        ----
----                                                                      ----
----  To Do:                                                              ----
----  -                                                                   ----
----                                                                      ----
----  Author:                                                             ----
----  Salvador E. Tropea                                                  ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Copyright (c) 2008-2017  <salvador en inti.gob.ar>                   ----
---- Copyright (c) 2008-2017 Instituto Nacional de Tecnología Industrial  ----
----                                                                      ----
---- Distributed under the GPL v2 or newer license                        ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Design unit:      IOPort(RTL) (Entity and architecture)              ----
---- File name:        portx.vhdl                                         ----
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

entity IOPort is
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
end entity IOPort;

architecture RTL of IOPort is
   -- Registers
   signal data_r   : std_logic_vector(port_i'range); -- Data to output
   signal ddr_r    : std_logic_vector(port_i'range); -- Data Direction
   signal inp_r    : std_logic_vector(port_i'range); -- Data from input
   signal inp0_r   : std_logic_vector(port_i'range); -- Filtered inputs
   -- Address decoding
   signal data_sel : std_logic;
   signal dir_sel  : std_logic;
   signal inp_sel  : std_logic;
begin
   data_sel <= '1' when adr_i=PPORT_TABLE(NUMBER).data and ENA_OUT='1' else '0';
   dir_sel  <= '1' when adr_i=PPORT_TABLE(NUMBER).ddr and ENA_OUT='1' and ENA_IN='1' else '0';
   inp_sel  <= '1' when adr_i=PPORT_TABLE(NUMBER).inp and ENA_IN='1' else '0';

   selected_o <= (data_sel or dir_sel or inp_sel) and re_i;

   do_regs:
   process (clk_i)
   begin
      if rising_edge(clk_i) then
         if rst_i='1' then
            data_r <= (others => '0');
            ddr_r  <= (others => '0');
            inp_r  <= (others => '0');
         else
            inp_r  <= inp0_r; -- 2nd stage sync.
            -- Write to registers
            if we_i='1' and ena_i='1' then
               if data_sel='1' then
                  data_r <= data_i(BITS-1 downto 0);
               end if;
               if dir_sel='1' then
                  ddr_r <= data_i(BITS-1 downto 0);
               end if;
            end if; -- we_i='1' and ena_i='1'
         end if; -- else rst_i='1'
      end if; -- rising_edge(clk_i)
   end process do_regs;

   -- DFF Falling Edge register
   dff_sync:
   process (clk_i)
   begin
      if falling_edge(clk_i) then
         inp0_r <= port_i;
      end if;
   end process dff_sync;

   data_o(BITS-1 downto 0) <= (data_r and data_sel) or
                              (ddr_r  and dir_sel) or
                              (inp_r  and inp_sel);

   -- Input/Output connection

   -- Bidirectional ports
   is_in_out:
   if ENA_OUT='1' and ENA_IN='1' generate
      do_hi_z:
      for i in port_o'range generate
          port_o(i) <= data_r(i) when ddr_r(i)='1' else 'Z';
      end generate do_hi_z;
   end generate is_in_out;

   -- Output ports
   is_out:
   if ENA_OUT='1' and ENA_IN='0' generate
      port_o <= data_r;
   end generate is_out;

   -- Input ports
   is_in:
   if ENA_OUT='0' and ENA_IN='1' generate
      port_o <= (others => 'Z');
   end generate is_in;
end architecture RTL; -- Entity: IOPort

