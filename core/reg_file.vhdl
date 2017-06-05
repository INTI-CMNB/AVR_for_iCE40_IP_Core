------------------------------------------------------------------------------
----                                                                      ----
----  Registers File                                                      ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  This is a synchronous implementation, specifically oriented to the  ----
----  iCE40 resources.                                                    ----
----  Is a dual-port memory : (Read-first)                                ----
----  * Port A                                                            ----
----    - 8 bits R/W                                                      ----
----    - 16 bits R/W                                                     ----
----  * Port B                                                            ----
----    - 8 bits R/W                                                      ----
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
---- Design unit:      RegisterFile(Behavioral) (Entity and architecture) ----
---- File name:        reg_file.vhdl                                      ----
---- Note:             None                                               ----
---- Limitations:      None known                                         ----
---- Errors:           None known                                         ----
---- Library:          avr                                                ----
---- Dependencies:     IEEE.std_logic_1164                                ----
----                   IEEE.numeric_std                                   ----
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

entity RegisterFile is
   port(
      clk_i    : in  std_logic;
      -- Main 8 bits port
      rd_we_i  : in  std_logic;
      rd_adr_i : in  unsigned(4 downto 0);
      rd_i     : in  std_logic_vector( 7 downto 0);
      rd_o     : out std_logic_vector( 7 downto 0);
      -- Main 16 bits port
      rd16_we_i: in  std_logic;
      rd16_i   : in  unsigned(15 downto 0);
      rd16_o   : out unsigned(15 downto 0);
      -- Secondary 8 bits read port
      rr_adr_i : in  unsigned(4 downto 0);
      rr_o     : out std_logic_vector(7 downto 0));
end entity RegisterFile;

architecture Behavioral of RegisterFile is
   type ramType is array (0 to 15) of std_logic_vector(7 downto 0);
   signal ram_0 : ramType:=(others => (others => '0')); -- Even
   signal ram_1 : ramType:=(others => (others => '0')); -- Odd
   -- Port used to read Rr
   signal ramB_0 : ramType:=(others => (others => '0')); -- Even
   signal ramB_1 : ramType:=(others => (others => '0')); -- Odd

   signal adr16      : unsigned(3 downto 0);
   signal adr16_r    : unsigned(3 downto 0);
   signal adr_lsb_r  : std_logic;
   signal adr16B     : unsigned(3 downto 0);
   signal adr16B_r   : unsigned(3 downto 0);
   signal adr_lsbB_r : std_logic;
   signal we0, we1   : std_logic;
   signal r0i        : std_logic_vector(7 downto 0);
   signal r1i        : std_logic_vector(7 downto 0);
begin
   -- Address for 16x16 registers
   adr16  <= rd_adr_i(4 downto 1);
   adr16B <= rr_adr_i(4 downto 1);
   -- Decode which RAM/s will have WE asserted
   we0 <= '1' when rd16_we_i='1' or (rd_we_i='1' and rd_adr_i(0)='0') else '0';
   we1 <= '1' when rd16_we_i='1' or (rd_we_i='1' and rd_adr_i(0)='1') else '0';
   -- Muxes for 16/8 bits writes
   r0i <= std_logic_vector(rd16_i( 7 downto 0)) when rd16_we_i='1' else rd_i;
   r1i <= std_logic_vector(rd16_i(15 downto 8)) when rd16_we_i='1' else rd_i;
   process (clk_i)
   begin
     if rising_edge(clk_i) then
        if we0='1' then
           ram_0(to_integer(adr16)) <= r0i;
           -- Mirror
           ramB_0(to_integer(adr16)) <= r0i;
        end if;
        if we1='1' then
           ram_1(to_integer(adr16)) <= r1i;
           -- Mirror
           ramB_1(to_integer(adr16)) <= r1i;
        end if;
        adr16_r <= adr16;
        adr_lsb_r <= rd_adr_i(0);
        -- Secondary read port
        adr16B_r <= adr16B;
        adr_lsbB_r <= rr_adr_i(0);
     end if;
   end process;

   rd16_o <= unsigned(ram_1(to_integer(adr16_r)))&unsigned(ram_0(to_integer(adr16_r)));
   rd_o   <= ram_1(to_integer(adr16_r)) when adr_lsb_r='1' else
             ram_0(to_integer(adr16_r));
   -- Secondary read port
   rr_o <= ramB_1(to_integer(adr16B_r)) when adr_lsbB_r='1' else
           ramB_0(to_integer(adr16B_r));
end architecture Behavioral; -- Entity: RegisterFile

