------------------------------------------------------------------------------
----                                                                      ----
----  Internal I/O registers for the AVR core                             ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  Implements the Status Register, Stack Pointer and RAM Page Z I/O    ----
----  registers. They are the "internal" I/O Regs.                        ----
----  It also includes the I/O data bus mux. External I/O regs uses       ----
----  e_data_i, this is shared with the external RAM data bus.            ----
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
---- Design unit:      IORegFile(RTL) (Entity and architecture)           ----
---- File name:        io_reg_file.vhdl                                   ----
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

entity IORegFile is
   generic(
      SP_W       : integer range 8 to 16:=16;  -- Stack pointer size
      ENA_RAMPZ  : std_logic:='0'); -- RAMPZ enable
   port(
      --Clock and reset
      clk_i     : in  std_logic;
      ena_i     : in  std_logic;
      rst_i     : in  std_logic;
      -- I/O write
      adr_i     : in  unsigned(5 downto 0); -- I/O Address
      we_i      : in  std_logic;            -- I/O Write Enable
      re_i      : in  std_logic;            -- I/O Read Enable
      data_i    : in  std_logic_vector(7 downto 0); -- I/O Data In
      data_o    : out std_logic_vector(7 downto 0); -- I/O Data Out
      e_data_i  : in  std_logic_vector(7 downto 0); -- External data (I/O-RAM)
      -- Status Register
      sreg_i    : in  std_logic_vector(7 downto 0); -- SREG in
      sreg_o    : out std_logic_vector(7 downto 0); -- SREG out
      sreg_we_i : in  std_logic_vector(7 downto 0); -- Flags write enable
      -- Stack Pointer
      sp_o      : out unsigned(SP_W-1 downto 0); -- SP
      sp_pop_i  : in  std_logic; -- Stack Pointer 1=Pop(+1) 0=Push(-1)
      sp_we_i   : in  std_logic; -- Stack Pointer write enable (dec/inc)
      -- RAM Page Z Select
      rampz_o   : out std_logic_vector(7 downto 0)); -- RAM Page Z
end entity IORegFile;

architecture RTL of IORegFile is
   signal sreg_r  : std_logic_vector(7 downto 0):=(others => '0');
   signal sp_r    : unsigned(SP_W-1 downto 0):=(others => '0');
   signal rampz_r : std_logic:='0';
   signal rampz   : std_logic_vector(7 downto 0);
   alias  spl_r   : unsigned(7 downto 0) is sp_r( 7 downto 0);
   signal spl     : std_logic_vector(7 downto 0);
   signal sph     : std_logic_vector(7 downto 0);
begin
   do_regs:
   process (clk_i)
      variable SP_TOP   : integer;
      variable DATA_TOP : integer;
   begin
      SP_TOP:=SP_W-1;
      if SP_TOP<8 then
         SP_TOP:=8;
      end if;
      DATA_TOP:=SP_W-9;
      if DATA_TOP<0 then
         DATA_TOP:=0;
      end if;
      if rising_edge(clk_i) then
         if rst_i='1' then
            sreg_r  <= (others => '0');
            sp_r    <= (others => '0');
            rampz_r <= '0';
         elsif ena_i='1' then
            if we_i='1' then
               -- Write to I/O File Registers (Data Bus)
               if adr_i=SREG_ADDRESS then
                  sreg_r <= data_i;
               elsif adr_i=SPL_ADDRESS then
                  spl_r <= unsigned(data_i);
               elsif adr_i=SPH_ADDRESS and SP_W>8 then
                  sp_r(SP_TOP downto 8) <= unsigned(data_i(DATA_TOP downto 0));
               elsif adr_i=RAMPZ_ADDRESS and ENA_RAMPZ='1' then
                  rampz_r <= data_i(0);
               end if;
            else
               -- Directly from the control logic
               -- Status Register (ALU Flags and SREG bit operations)
               for i in sreg_r'range loop
                   if sreg_we_i(i)='1' then
                      sreg_r(i) <= sreg_i(i);
                   end if;
               end loop;
               -- Stack Pointer (inc/dec -> push, pop, call, ret, etc.)
               if sp_we_i='1' then
                  if sp_pop_i='1' then
                     sp_r <= sp_r+1; -- Pop
                  else
                     sp_r <= sp_r-1; -- Push
                  end if;
               end if; -- sp_we_i='1'
            end if; -- else we_i='1'
         end if; -- elsif ena_i='1'
      end if; -- rising_edge(clk_i)
   end process do_regs;
   rampz <= "0000000"&rampz_r;

   do_sph:
   process (sp_r)
      variable SP_MSB : integer;
      variable SP_LSB : integer;
   begin
      -- Here we are just avoiding an invalid range. It can't be used even in
      -- code protected by if or if generate, so we must fix the value.
      SP_MSB:=SP_W-1;
      SP_LSB:=8;
      if SP_MSB<8 then
         SP_MSB:=0;
         SP_LSB:=0;
      end if;
      if SP_W>=8 then
         sph <= std_logic_vector(resize(sp_r(SP_MSB downto SP_LSB),8));
      end if;
   end process do_sph;

   -- Individual registers
   sreg_o  <= sreg_r;
   sp_o    <= sp_r;
   rampz_o <= rampz;
   spl     <= std_logic_vector(spl_r);
   -- I/O Data Bus Mux, provides our registers or the external data
   -- Note: re_i is used to differentiate RAM and I/O reads
   data_o  <= spl    when adr_i=SPL_ADDRESS   and re_i='1' else
              sph    when adr_i=SPH_ADDRESS   and re_i='1' and SP_W>8 else
              sreg_r when adr_i=SREG_ADDRESS  and re_i='1' else
              rampz  when adr_i=RAMPZ_ADDRESS and re_i='1' and ENA_RAMPZ='1' else
              e_data_i;
end architecture RTL; -- Entity: IORegFile

