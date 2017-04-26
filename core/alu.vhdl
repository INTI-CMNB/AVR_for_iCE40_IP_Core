------------------------------------------------------------------------------
----                                                                      ----
----  ALU (Arithmetic Logic Unit)                                         ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  This is an ALU approach for AVR v2.x. Uses 8 operations.            ----
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
---- Design unit:      ALU(RTL) (Entity and architecture)                 ----
---- File name:        alu.vhdl                                           ----
---- Note:             None                                               ----
---- Limitations:      None known                                         ----
---- Errors:           None known                                         ----
---- Library:          avr                                                ----
---- Dependencies:     IEEE.std_logic_1164                                ----
----                   IEEE.numeric_std                                   ----
----                   avr.Constants                                      ----
----                   avr.Internal                                       ----
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
use avr.Internal.all;
use avr.Types.all;
use avr.Constants.all;

entity ALU is
   port(
      -- Operands
      a_i      : in  unsigned(7 downto 0);
      b_i      : in  unsigned(7 downto 0);
      -- Input flags
      c_i      : in  std_logic;
      -- Operation (only one active at the time)
      op_add_i : in  std_logic;
      op_sub_i : in  std_logic;
      op_and_i : in  std_logic;
      op_or_i  : in  std_logic;
      op_xor_i : in  std_logic;
      op_nop_i : in  std_logic;
      op_shf_i : in  std_logic;
      op_swp_i : in  std_logic;
      -- Result
      s_o      : out unsigned(7 downto 0);
      -- Flags
      c_o      : out std_logic;
      h_o      : out std_logic;
      z_o      : out std_logic;
      v_o      : out std_logic);
end entity ALU;

architecture RTL of ALU is
   signal s       : unsigned(7 downto 0); -- Result
   signal c       : std_logic; -- Carry
   signal h       : std_logic; -- Half Carry
   signal z       : std_logic; -- Zero
   signal v       : std_logic; -- Overflow
   signal cin     : unsigned(0 downto 0); -- Adder/Sub Carry In
   -- Adder
   signal add_lo  : unsigned(4 downto 0); -- Adder Low Nibble
   signal add_hi  : unsigned(4 downto 0); -- Adder High Nibble
   -- Substractor
   signal sub_lo  : unsigned(4 downto 0); -- Adder Low Nibble 
   signal sub_hi  : unsigned(4 downto 0); -- Adder High Nibble
begin
   cin(0) <= c_i;
   -----------------------
   -- Adder: A+B        --
   -----------------------
   -- Low nibble
   add_lo  <= resize(a_i(3 downto 0),5)+b_i(3 downto 0)+cin;
   -- High nibble
   add_hi  <= resize(a_i(7 downto 4),5)+b_i(7 downto 4)+add_lo(4 downto 4);
   -----------------------
   -- Substractor: A-B  --
   -----------------------
   -- Low nibble
   sub_lo  <= resize(a_i(3 downto 0),5)-b_i(3 downto 0)-cin;
   -- High nibble
   sub_hi  <= resize(a_i(7 downto 4),5)-b_i(7 downto 4)-sub_lo(4 downto 4);

   -- Outputs
   -- Result
   s <= (add_hi(3 downto 0)&add_lo(3 downto 0) and op_add_i) or
        (sub_hi(3 downto 0)&sub_lo(3 downto 0) and op_sub_i) or
        (c_i&a_i(7 downto 1)                   and op_shf_i) or
        ((a_i and b_i)                         and op_and_i) or
        ((a_i or  b_i)                         and op_or_i)  or
        ((a_i xor b_i)                         and op_xor_i) or
        (a_i(3 downto 0)&a_i(7 downto 4)       and op_swp_i) or
        (a_i                                   and op_nop_i);
   s_o <= s;
   -- Zero flag
   z   <= '1' when s=x"00" else '0';
   z_o <= z;
   -- Carry flag
   c   <= (add_hi(4) and op_add_i) or -- Cy from the high nibble
          (sub_hi(4) and op_sub_i) or -- Borrow from the high nibble
          (a_i(0)    and op_shf_i);
   c_o <= c;
   -- Half carry flag
   h   <= (add_lo(4) and op_add_i) or -- Cy from the low nibble
          (sub_lo(4) and op_sub_i);   -- Borrow from the low nibble
   h_o <= h;
   -- Overflow flag
   v   <= (((a_i(7) and b_i(7) and not(add_hi(3))) or   -- (-)+(-)=(+)
           (not(a_i(7)) and not(b_i(7)) and add_hi(3))) -- (+)+(+)=(-)
           and op_add_i) or
          (((a_i(7) and not(b_i(7)) and not(sub_hi(3))) or -- (-)-(+)=(+)
           (not(a_i(7)) and b_i(7) and sub_hi(3)))         -- (+)-(-)=(-)
           and op_sub_i) or
          ((s(7) xor s(0)) and op_shf_i);
   v_o <= v;
end architecture RTL; -- Entity: ALU

