------------------------------------------------------------------------------
----                                                                      ----
----  Single Port ROM that maps to a Xilinx/Lattice BRAM                  ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  This is a program memory for the AVR. It maps to a Xilinx BRAM      ----
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
---- Distributed under the BSD license                                    ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Design unit:      SinglePortPM(Xilinx) (Entity and architecture)     ----
---- File name:        pm_s.in.vhdl (template used)                       ----
---- Note:             None                                               ----
---- Limitations:      None known                                         ----
---- Errors:           None known                                         ----
---- Library:          work                                               ----
---- Dependencies:     IEEE.std_logic_1164                                ----
---- Target FPGA:      Spartan 3 (XC3S1500-4-FG456)                       ----
---- Language:         VHDL                                               ----
---- Wishbone:         No                                                 ----
---- Synthesis tools:  Xilinx Release 9.2.03i - xst J.39                  ----
---- Simulation tools: GHDL [Sokcho edition] (0.2x)                       ----
---- Text editor:      SETEdit 0.5.x                                      ----
----                                                                      ----
------------------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity helloPM is
   generic(
      WORD_SIZE  : integer:=16;  -- Word Size
      FALL_EDGE  : boolean:=false;-- Ram clock falling edge
      ADDR_W     : integer:=13); -- Address Width
   port(
      clk_i   : in  std_logic;
      addr_i  : in  std_logic_vector(ADDR_W-1 downto 0);
      data_o  : out std_logic_vector(WORD_SIZE-1 downto 0));
end entity helloPM;

architecture Xilinx of helloPM is
   constant ROM_SIZE : natural:=2**ADDR_W;
   type rom_t is array(natural range 0 to ROM_SIZE-1) of std_logic_vector(WORD_SIZE-1 downto 0);
   signal addr_r  : std_logic_vector(ADDR_W-1 downto 0);

   signal rom : rom_t :=
(
     0 => x"c00e",
     1 => x"c026",
     2 => x"c025",
     3 => x"c024",
     4 => x"c023",
     5 => x"c022",
     6 => x"c021",
     7 => x"c020",
     8 => x"c01f",
     9 => x"c01e",
    10 => x"c01d",
    11 => x"c01c",
    12 => x"c01b",
    13 => x"c01a",
    14 => x"c019",
    15 => x"2411",
    16 => x"be1f",
    17 => x"edcf",
    18 => x"bfcd",
    19 => x"e010",
    20 => x"e6a0",
    21 => x"e0b0",
    22 => x"eeee",
    23 => x"e0f0",
    24 => x"c002",
    25 => x"9005",
    26 => x"920d",
    27 => x"38aa",
    28 => x"07b1",
    29 => x"f7d9",
    30 => x"e020",
    31 => x"e8aa",
    32 => x"e0b0",
    33 => x"c001",
    34 => x"921d",
    35 => x"39a0",
    36 => x"07b2",
    37 => x"f7e1",
    38 => x"d015",
    39 => x"c04d",
    40 => x"cfd7",
    41 => x"308a",
    42 => x"f049",
    43 => x"e091",
    44 => x"bb9f",
    45 => x"9bf0",
    46 => x"cffe",
    47 => x"ba1f",
    48 => x"bb8e",
    49 => x"e080",
    50 => x"e090",
    51 => x"9508",
    52 => x"e091",
    53 => x"bb9f",
    54 => x"9bf0",
    55 => x"cffe",
    56 => x"ba1f",
    57 => x"e09d",
    58 => x"bb9e",
    59 => x"cfef",
    60 => x"e680",
    61 => x"e090",
    62 => x"9390",
    63 => x"008d",
    64 => x"9380",
    65 => x"008c",
    66 => x"e68e",
    67 => x"e090",
    68 => x"d003",
    69 => x"e080",
    70 => x"e090",
    71 => x"9508",
    72 => x"930f",
    73 => x"931f",
    74 => x"93cf",
    75 => x"93df",
    76 => x"91e0",
    77 => x"008c",
    78 => x"91f0",
    79 => x"008d",
    80 => x"8123",
    81 => x"ff21",
    82 => x"c01b",
    83 => x"01ec",
    84 => x"e000",
    85 => x"e010",
    86 => x"9189",
    87 => x"9160",
    88 => x"008c",
    89 => x"9170",
    90 => x"008d",
    91 => x"01db",
    92 => x"9618",
    93 => x"91ed",
    94 => x"91fc",
    95 => x"9719",
    96 => x"2388",
    97 => x"f031",
    98 => x"9509",
    99 => x"2b89",
   100 => x"f389",
   101 => x"ef0f",
   102 => x"ef1f",
   103 => x"cfee",
   104 => x"e08a",
   105 => x"9509",
   106 => x"2b89",
   107 => x"f411",
   108 => x"01c8",
   109 => x"c002",
   110 => x"ef8f",
   111 => x"ef9f",
   112 => x"91df",
   113 => x"91cf",
   114 => x"911f",
   115 => x"910f",
   116 => x"9508",
   117 => x"94f8",
   118 => x"cfff",
   119 => x"0000",
   120 => x"0300",
   121 => x"0000",
   122 => x"0000",
   123 => x"0029",
   124 => x"0000",
   125 => x"0000",
   126 => x"5441",
   127 => x"6974",
   128 => x"796e",
   129 => x"3558",
   130 => x"7320",
   131 => x"7961",
   132 => x"3a73",
   133 => x"6820",
   134 => x"6c65",
   135 => x"6f6c",
   136 => x"7720",
   137 => x"726f",
   138 => x"646c",
   139 => x"0021",
others => x"0000"

);
begin

   use_rising_edge:
   if not FALL_EDGE generate
      do_rom:
      process (clk_i)
      begin
         if rising_edge(clk_i)then
            addr_r <= addr_i;
         end if;
      end process do_rom;
  end generate use_rising_edge;

  use_falling_edge:
  if FALL_EDGE generate
      do_rom:
      process (clk_i)
      begin
         if falling_edge(clk_i)then
            addr_r <= addr_i;
         end if;
      end process do_rom;
  end generate use_falling_edge;

  data_o <= rom(to_integer(unsigned(addr_r)));

end architecture Xilinx; -- Entity: helloPM

