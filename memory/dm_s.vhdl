------------------------------------------------------------------------------
----                                                                      ----
----  Single Port RAM that maps to an FPGA BRAM                           ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  This is a data memory for the AVR. It maps to a Lattice or Xilinx   ----
----  BRAM, most probably also works for ALTERA, Microsemi, etc.          ----
----                                                                      ----
----  To Do:                                                              ----
----  -                                                                   ----
----                                                                      ----
----  Author:                                                             ----
----    - Salvador E. Tropea, salvador inti.gob.ar                        ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Copyright (c) 2008 Salvador E. Tropea <salvador inti.gob.ar>         ----
---- Copyright (c) 2008 Instituto Nacional de Tecnología Industrial       ----
----                                                                      ----
---- Distributed under the GPL v2 or newer license                        ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Design unit:      SinglePortRAM(Xilinx) (Entity and architecture)    ----
---- File name:        dm_s.vhdl                                          ----
---- Note:             None                                               ----
---- Limitations:      None known                                         ----
---- Errors:           None known                                         ----
---- Library:          avr                                                ----
---- Dependencies:     IEEE.std_logic_1164                                ----
----                   IEEE.numeric_std                                   ----
---- Target FPGA:      Spartan 3 (XC3S1500-4-FG456)                       ----
----                   iCE40HX4K-TQ144                                    ----
---- Language:         VHDL                                               ----
---- Wishbone:         No                                                 ----
---- Synthesis tools:  Xilinx Release 9.2.03i - xst J.39                  ----
----                   Lattice iCECube2 2016.02.27810                     ----
---- Simulation tools: GHDL [Sokcho edition] (0.2x)                       ----
---- Text editor:      SETEdit 0.5.x                                      ----
----                                                                      ----
------------------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity SinglePortRAM is
   generic(
      FALL_EDGE    : boolean:=false;
      WORD_SIZE    : integer:=8;   -- Word Size
      ADDR_W       : integer:=12); -- Address Width
   port(
      clk_i   : in  std_logic;
      we_i    : in  std_logic;
      addr_i  : in  std_logic_vector(ADDR_W-1 downto 0);
      d_i     : in  std_logic_vector(WORD_SIZE-1 downto 0);
      d_o     : out std_logic_vector(WORD_SIZE-1 downto 0));
end entity SinglePortRAM;

architecture Xilinx of SinglePortRAM is
   type ram_t is array(natural range 0 to (2**ADDR_W)-1) of std_logic_vector(WORD_SIZE-1 downto 0);
   signal addr_r  : std_logic_vector(ADDR_W-1 downto 0);
   signal ram     : ram_t;
begin

   use_rising_edge:
   if not FALL_EDGE generate
     do_ram:
     process (clk_i)
     begin
        if rising_edge(clk_i) then
           if we_i='1' then
              ram(to_integer(unsigned(addr_i))) <= d_i;
           end if;
           addr_r <= addr_i;
        end if;
     end process do_ram;
   end generate use_rising_edge;

   use_falling_edge:
   if FALL_EDGE generate
     do_ram:
     process (clk_i)
     begin
        if falling_edge(clk_i) then
           if we_i='1' then
              ram(to_integer(unsigned(addr_i))) <= d_i;
           end if;
           addr_r <= addr_i;
        end if;
     end process do_ram;
   end generate use_falling_edge;

   d_o <= ram(to_integer(unsigned(addr_r)));

end architecture Xilinx; -- Entity: SinglePortRAM

