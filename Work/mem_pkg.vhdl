------------------------------------------------------------------------------
----                                                                      ----
----  RAM memory prototypes                                               ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  Package for the RAM memory                                          ----
----                                                                      ----
----  To Do:                                                              ----
----  -                                                                   ----
----                                                                      ----
----  Author:                                                             ----
----  Salvador E. Tropea                                                  ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Copyright (c) 2017  <salvador en inti.gob.ar>                        ----
---- Copyright (c) 2017 Instituto Nacional de Tecnología Industrial       ----
----                                                                      ----
---- Distributed under the GPL v2 or newer license                        ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Design unit:      Memory (Package)                                   ----
---- File name:        mem_pkg.vhdl                                       ----
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

package Memory is
   component SinglePortRAM is
      generic(
         FALL_EDGE    : std_logic:='0';
         WORD_SIZE    : integer:=8;   -- Word Size
         ADDR_W       : integer:=12); -- Address Width
      port(
         clk_i   : in  std_logic;
         we_i    : in  std_logic;
         addr_i  : in  std_logic_vector(ADDR_W-1 downto 0);
         d_i     : in  std_logic_vector(WORD_SIZE-1 downto 0);
         d_o     : out std_logic_vector(WORD_SIZE-1 downto 0));
   end component SinglePortRAM;
end package Memory;

