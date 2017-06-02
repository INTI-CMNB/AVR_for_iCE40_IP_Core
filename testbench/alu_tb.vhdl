------------------------------------------------------------------------------
----                                                                      ----
----  Testbench for the AVR's ALU                                         ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  Reads the data and references from gen_alu_test and verifies the    ----
----  ALU behavior.                                                       ----
----                                                                      ----
----  To Do:                                                              ----
----  -                                                                   ----
----                                                                      ----
----  Author:                                                             ----
----    - Salvador E. Tropea, salvador inti.gob.ar                        ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Copyright (c) 2008-2017 Salvador E. Tropea <salvador inti.gob.ar>    ----
---- Copyright (c) 2008-2017 Instituto Nacional de Tecnología Industrial  ----
----                                                                      ----
---- Distributed under the GPL v2 or newer license                        ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Design unit:      ALU_TB(Simulator) (Entity and architecture)        ----
---- File name:        alu_tb.vhdl                                        ----
---- Note:             None                                               ----
---- Limitations:      None known                                         ----
---- Errors:           None known                                         ----
---- Library:          work                                               ----
---- Dependencies:     IEEE.std_logic_1164                                ----
----                   std.textio                                         ----
----                   utils.StdIO                                        ----
----                   utils.Str                                          ----
----                   avr.Internal                                       ----
----                   avr.Types                                          ----
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

library std;
use std.textio.all;

library utils;
use utils.StdIO.all;
use utils.Str.all;

library avr;
use avr.Internal.all;
use avr.Types.all;

entity ALU_TB is
end entity ALU_TB;

architecture Simulator of ALU_TB is
   constant CLK_FREQ   : positive:=25; -- 25 MHz clock
   constant CLK_S_PER  : time:=1 us/(2.0*real(CLK_FREQ)); -- Clock semi period
   constant SP_W       : positive:=8;
   constant SP_0       : unsigned(SP_W-1 downto 0):=(others => '0');

   -- ALU signals
   signal opcode    : std_logic_vector(15 downto 0);
   signal rr0       : std_logic_vector(7 downto 0);
   signal rr        : std_logic_vector(7 downto 0):=(others => '0');
   signal rd0       : std_logic_vector(7 downto 0);
   signal rd        : std_logic_vector(7 downto 0):=(others => '0');
   signal cur_sreg  : std_logic_vector(7 downto 0):=(others => '0');
   signal sreg_mask : std_logic_vector(7 downto 0);
   signal ci0       : std_logic:='0';
   signal zi0       : std_logic:='0';
   signal ci        : std_logic:='0';
   signal zi        : std_logic:='0';
   -- Data Output
   signal res       : std_logic_vector(7 downto 0);
   signal res_rdy   : std_logic;
   signal sreg_res  : std_logic_vector(7 downto 0);
   -- Flags
   alias  c         : std_logic is sreg_res(0);
   alias  z         : std_logic is sreg_res(1);
   alias  n         : std_logic is sreg_res(2);
   alias  v         : std_logic is sreg_res(3);
   alias  s         : std_logic is sreg_res(4);
   alias  h         : std_logic is sreg_res(5);
   alias  c_we      : std_logic is sreg_mask(0);
   alias  z_we      : std_logic is sreg_mask(1);
   alias  n_we      : std_logic is sreg_mask(2);
   alias  v_we      : std_logic is sreg_mask(3);
   alias  s_we      : std_logic is sreg_mask(4);
   alias  h_we      : std_logic is sreg_mask(5);

   -- Test
   signal inst     : string(1 to 6);
   signal ref0     : std_logic_vector(7 downto 0);
   signal ref      : std_logic_vector(7 downto 0);
   -- Flags
   signal cref0    : std_logic;
   signal zref0    : std_logic;
   signal nref0    : std_logic;
   signal vref0    : std_logic;
   signal sref0    : std_logic;
   signal href0    : std_logic;
   signal cref     : std_logic:='X';
   signal zref     : std_logic;
   signal nref     : std_logic;
   signal vref     : std_logic;
   signal sref     : std_logic;
   signal href     : std_logic;
   --
   signal clk      : std_logic:='0';
   signal nclk     : std_logic:='1';
   signal end_test : boolean:=false;
   signal reset    : std_logic:='1';
   --
   signal dbgo_rd_we    : std_logic;
   signal dbgo_cyc_last : std_logic;
begin
   do_clock:
   process
   begin
      clk  <= '0';
      nclk <= '1';
      wait for CLK_S_PER;
      clk  <= '1';
      nclk <= '0';
      wait for CLK_S_PER;
      if end_test then
         outwrite("* End of test");
         wait;
      end if;
   end process do_clock;

   do_reset:
   process
   begin
      wait until rising_edge(clk);
      reset <= '0';
   end process do_reset;

   micro : entity avr.CPU
      generic map(ENA_DEBUG => '1', SP_W => SP_W)
      port map(
         -- Clock and reset
         clk_i => clk, ena_i => '1', rst_i => reset,
         -- Program memory
         pc_o => open, inst_i => opcode,
         -- I/O control
         io_adr_o => open, io_re_o => open, io_we_o => open,
         -- Data memory control
         ram_adr_o => open, ram_re_o => open, ram_we_o => open,
         -- Data paths
         io_data_i => (others => '0'), data_o => open,
         ram_data_i => (others => '0'),
         -- Interrupt
         irq_lines_i => (others => '0'), irq_acks_o => open,
         irq_ok_o => open,
         -- Sleep
         sleep_o => open,
         --Watchdog
         wdr_o => open,
         -- I/O register file interface
         sreg_o => sreg_res, sreg_i => cur_sreg,
         sreg_we_o => sreg_mask, sp_i => SP_0,
         sp_pop_o => open, sp_we_o => open,
         rampz_i => (others => '0'),
         -- Debug
         dbg_stop_i => '0',
         dbg_rd_data_i => rd,
         dbg_rr_data_i => rr,
         dbg_rf_fake_i => '1',
         dbg_rd_data_o => res,
         dbg_rd_we_o => dbgo_rd_we,
         dbg_cyc_last_o => dbgo_cyc_last);
   res_rdy <= dbgo_rd_we or dbgo_cyc_last;

   cur_sreg(0) <= ci;
   cur_sreg(1) <= zi;

   do_test:
   process
      variable l      : line;
      variable lo     : line;
      variable sep    : string(1 to 1);
      variable v_inst : string(1 to 6);
      variable o_inst : string(1 to 6);
      variable sv_aux : std_logic_vector(7 downto 0);
      variable rd_aux : std_logic_vector(7 downto 0);
      variable op_aux : std_logic_vector(15 downto 0);
      variable flag   : std_logic;
      variable cnt    : integer:=0;
      variable dif    : boolean;
   begin
      outwrite("* ALU testbench");
      wait until reset='0';
      wait until rising_edge(clk);
      while not(endfile(input)) loop
         cnt:=cnt+1;
         readline(input,l);
         o_inst:=v_inst;
         read(l,v_inst);
         inst <= v_inst;
         read(l,sep);
         read(l,op_aux);
         opcode <= op_aux;
         read(l,sep);
         read(l,rd_aux);
         dif:=rd/=rd_aux;
         rd <= rd_aux;
         read(l,sep);
         read(l,sv_aux);
         rr <= sv_aux;
         read(l,sep);
         read(l,flag);
         ci <= flag;
         read(l,sep);
         read(l,flag);
         zi <= flag;
         if dif or cref='X' then
            outwrite("Cin,Zin,Rd: "&std_logic'image(ci)&","&
                     std_logic'image(zi)&","&
                     integer'image(to_integer(unsigned(rd_aux))));
         end if;
         read(l,sep);
         read(l,sv_aux);
         ref <= sv_aux;
         read(l,sep);
         read(l,flag);
         href <= flag;
         read(l,sep);
         read(l,flag);
         sref <= flag;
         read(l,sep);
         read(l,flag);
         vref <= flag;
         read(l,sep);
         read(l,flag);
         nref <= flag;
         read(l,sep);
         read(l,flag);
         zref <= flag;
         read(l,sep);
         read(l,flag);
         cref <= flag;
         wait until rising_edge(clk) and res_rdy='1' for CLK_S_PER*20;
         if cref/='X' then
            if res/=ref or (href/='-' and h/=href) or (sref/='-' and s/=sref) or
               (vref/='-' and v/=vref) or (nref/='-' and n/=nref) or
               (zref/='-' and z/=zref) or (cref/='-' and c/=cref) or
               (href='-' and h_we='1') or (sref='-' and s_we='1') or
               (vref='-' and v_we='1') or (nref='-' and n_we='1') or
               (zref='-' and z_we='1') or (cref='-' and c_we='1') or
               res_rdy='0' then
               write(lo,inst&" Rd: 0x"&hstr(rd)&" Rr: 0x"&hstr(rr)&
                     " Result: 0x"&hstr(res)&" Reference: 0x"&hstr(ref)&
                     " Cin: "&std_logic'image(ci)&" Zin: "&std_logic'image(zi));
               writeline(output,lo);
               write(lo,"H:"&std_logic'image(h)&std_logic'image(href)&"|"&
                        "S:"&std_logic'image(s)&std_logic'image(sref)&"|"&
                        "V:"&std_logic'image(v)&std_logic'image(vref)&"|"&
                        "N:"&std_logic'image(n)&std_logic'image(nref)&"|"&
                        "Z:"&std_logic'image(z)&std_logic'image(zref)&"|"&
                        "C:"&std_logic'image(c)&std_logic'image(cref)&"|"&
                        "Flag:'Result''Reference'");
               writeline(output,lo);
            end if;
            --wait until rising_edge(clk);
            assert res_rdy='1' report "Time-out" severity failure;
            assert res=ref report "Wrong output" severity failure;
            assert h=href or href='-' report "Wrong H" severity failure;
            assert s=sref or sref='-' report "Wrong S" severity failure;
            assert v=vref or vref='-' report "Wrong V" severity failure;
            assert n=nref or nref='-' report "Wrong N" severity failure;
            assert z=zref or zref='-' report "Wrong Z" severity failure;
            assert c=cref or cref='-' report "Wrong C" severity failure;
            assert href/='-' or h_we='0' report "Invalid write to H" severity failure;
            assert sref/='-' or s_we='0' report "Invalid write to S" severity failure;
            assert vref/='-' or v_we='0' report "Invalid write to V" severity failure;
            assert nref/='-' or n_we='0' report "Invalid write to N" severity failure;
            assert zref/='-' or z_we='0' report "Invalid write to Z" severity failure;
            assert cref/='-' or c_we='0' report "Invalid write to C" severity failure;
         end if; -- cref/='X'
      end loop;
      outwrite("* Successful, "&integer'image(cnt)&" combinations tested");
      end_test <= true;
      wait;
   end process do_test;

end architecture Simulator; -- Entity: ALU_TB

