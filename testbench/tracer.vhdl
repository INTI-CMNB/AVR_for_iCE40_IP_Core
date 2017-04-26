------------------------------------------------------------------------------
----                                                                      ----
----  Tracer                                                              ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  Used to print an ejecution trace. Use avr-disa tool to decode the   ----
----  opcodes (into mnemonics).                                           ----
----                                                                      ----
----  To Do:                                                              ----
----  -                                                                   ----
----                                                                      ----
----  Author:                                                             ----
----    - Salvador E. Tropea, salvador inti.gob.ar                        ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Copyright (c) 2009 Salvador E. Tropea <salvador inti.gob.ar>         ----
---- Copyright (c) 2009 Instituto Nacional de Tecnología Industrial       ----
----                                                                      ----
---- Distributed under the GPL v2 or newer license                        ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Design unit:      Tracer(Simulator) (Entity and architecture)        ----
---- File name:        tracer.vhdl                                        ----
---- Note:             None                                               ----
---- Limitations:      None known                                         ----
---- Errors:           None known                                         ----
---- Library:          avr                                                ----
---- Dependencies:     IEEE.std_logic_1164                                ----
----                   IEEE.numeric_std                                   ----
----                   avr.Types                                          ----
----                   utils.StdIO                                        ----
----                   utils.Str                                          ----
---- Target FPGA:      N/A                                                ----
---- Language:         VHDL                                               ----
---- Wishbone:         No                                                 ----
---- Synthesis tools:  N/A                                                ----
---- Simulation tools: GHDL [Sokcho edition] (0.2x)                       ----
---- Text editor:      SETEdit 0.5.x                                      ----
----                                                                      ----
------------------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
library avr;
use avr.Types.all;
library utils;
use utils.StdIO.all;
use utils.Str.all;

entity Tracer is
   port(
      clk_i     : in  std_logic;
      rst_i     : in  std_logic;
      ena_i     : in  std_logic:='1';
      dbg_i     : in  debug_o_t);
end entity Tracer;

architecture Simulator of Tracer is
   signal rst_r : std_logic:='0';
begin
   do_trace:
   process (clk_i)
   begin
      if rising_edge(clk_i)  then
         if rst_i='1' then
            rst_r <= '1';
         else
            rst_r <= '0';
            if rst_r='0' and dbg_i.exec='1' and ena_i='1' and dbg_i.stopped='0' then
               if dbg_i.is32 then
                  outwrite("0x"&hstr(dbg_i.pc)&" 0x"&hstr(dbg_i.inst)&" 0x"&hstr(dbg_i.inst2));
               else
                  outwrite("0x"&hstr(dbg_i.pc)&" 0x"&hstr(dbg_i.inst));
               end if;
            end if; -- rst_r='0' and dbg_i.exec='1' and ena_i='1'
         end if; -- else rst_i='1'
      end if; -- rising_edge(clk_i)
   end process do_trace;
end architecture Simulator; -- Entity: Tracer

