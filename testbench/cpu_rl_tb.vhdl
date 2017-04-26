------------------------------------------------------------------------------
----                                                                      ----
----  Testbench for the AVR.                                              ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  This testbench load a lot of small programs and tests them.         ----
----                                                                      ----
----  To Do:                                                              ----
----  -                                                                   ----
----                                                                      ----
----  Author:                                                             ----
----    - Salvador E. Tropea, salvador inti.gob.ar                        ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Copyright (c) 2009-2016 Salvador E. Tropea <salvador inti.gob.ar>    ----
---- Copyright (c) 2009-2016 Instituto Nacional de Tecnología Industrial  ----
----                                                                      ----
---- Distributed under the GPL v2 or newer license                        ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Design unit:      CPU_RL_Tb(Simulator) (Entity and architecture)     ----
---- File name:        cpu_rl_tb.vhdl                                     ----
---- Note:             None                                               ----
---- Limitations:      None known                                         ----
---- Errors:           None known                                         ----
---- Library:          work                                               ----
---- Dependencies:     IEEE.std_logic_1164                                ----
----                   IEEE.numeric_std                                   ----
----                   avr.Micros                                         ----
----                   avr.Types                                          ----
----                   avr.Internal                                       ----
----                   avr.Memory                                         ----
----                   avr.Constants                                      ----
----                   mems.Devices                                       ----
----                   utils.StdIO                                        ----
----                   utils.Str                                          ----
----                   utils.StdLib                                       ----
----                   wb_handler.WishboneTB                              ----
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
use avr.Internal.all;
use avr.Micros.all;
use avr.Types.all;
use avr.Memory.all;
use avr.Constants.all;
library utils;
use utils.StdIO.all;
use utils.Str.all;
use utils.StdLib.all;
library wb_handler;
use wb_handler.WishboneTB.all;
library mems;
use mems.Devices.all;

entity CPU_RL_Tb is
end entity CPU_RL_Tb;

architecture Simulator of CPU_RL_Tb is
   constant SHOW_TRACE : boolean:=false;
   constant CLK_FREQ   : positive:=25; -- 25 MHz clock
   constant CLK_S_PER  : time:=1 us/(2.0*real(CLK_FREQ)); -- Clock semi period
   -- Maximum number of iterations before assuming the CPU hanged.
   constant MAX_ITERATION: integer:=10000;
   constant DATA_RAM_W   : positive:=8;    -- Data RAM address width (256)
   constant PORTB_SIZE   : positive:=8;-- 5
   constant ENA_PORTB    : boolean:=true;

   signal clk          : std_logic;
   signal nclk         : std_logic;
   signal rst          : std_logic:='1';
   signal iteration    : integer:=0;
   signal time_out     : std_logic;

   signal pc           : unsigned(15 downto 0); -- PROM address
   signal adr_pc       : std_logic_vector(15 downto 0); -- PROM address
   signal inst         : std_logic_vector(15 downto 0); -- PROM data
   signal inst_w       : std_logic_vector(15 downto 0); -- PROM data
   signal pgm_we       : std_logic;
   signal irqs         : std_logic:='0';
   signal portb        : std_logic_vector(PORTB_SIZE-1 downto 0);
   signal portd        : std_logic_vector(7 downto 0);

   signal wb_adr       : std_logic_vector(7 downto 0); -- I/O Address
   signal wb_dato      : std_logic_vector(7 downto 0); -- Data Bus output
   signal wb_dati      : std_logic_vector(7 downto 0):="00000000"; -- Data Bus input
   signal wb_stb       : std_logic;  -- Strobe output
   signal wb_we        : std_logic;  -- Write Enable output
   signal wb_ack       : std_logic:='1'; -- Acknowledge input

   signal hex_file     : string(6 downto 1); -- Name of the .hex file to load
   signal load         : std_logic:='0';
   signal loaded       : std_logic;
   signal end_test     : std_logic:='0';

   signal dbg          : debug_o_t;
   signal ena_trace    : std_logic;

   signal ram_we       : std_logic;
   signal ram_adr      : std_logic_vector(15 downto 0);
   signal ram_din      : std_logic_vector(7 downto 0);
   signal ram_datao    : std_logic_vector(7 downto 0);

   -- I/O registers
   signal io_adr    : unsigned(5 downto 0);
   signal io_re     : std_logic;
   signal io_we     : std_logic;
   -- Input Data after the IORegFile block
   signal io_datai    : std_logic_vector(7 downto 0);
   -- I/O Registers
   signal new_sreg    : std_logic_vector(7 downto 0);
   signal sreg        : std_logic_vector(7 downto 0);
   signal sreg_we     : std_logic_vector(7 downto 0);
   signal sp          : unsigned(8-1 downto 0);
   --signal rampz       : std_logic_vector(7 downto 0);
   signal sp_pop      : std_logic;
   signal sp_we       : std_logic;

   -- I/O mux
   constant EXT_BUS_W : positive:=1; -- 5 peripherals
   signal io_out    : ext_mux_t(0 to EXT_BUS_W-1):=(others => (others => '0'));
   signal io_out_en : std_logic_vector(EXT_BUS_W-1 downto 0):=(others => '0');
   signal core_din : std_logic_vector(7 downto 0);

   procedure Reset(signal rst: out std_logic) is
   begin
      rst <= '1';
      wait for 3.25*CLK_S_PER;
      rst <= '0';
   end procedure Reset;

begin
   micro : entity avr.ATtX5
      generic map(PORTB_SIZE => 8, ENA_DEBUG => true, ENA_PORTD => true, ENA_SPM => true)
      port map(
         rst_i => rst, clk_i => clk, clk2x_i => clk,
         pc_o => pc, inst_i => inst, inst_o => inst_w, pgm_we_o => pgm_we,
         pin_irq_i(0) => portd(7), pin_irq_i(1) => '0',
         portb_o => portb, dbg_o => dbg, portd_o => portd,
         -- WISHBONE
         wb_adr_o => wb_adr, wb_dat_o => wb_dato, wb_dat_i => wb_dati,
         wb_stb_o => wb_stb, wb_we_o  => wb_we,   wb_ack_i => wb_ack);

   portd(7) <= 'L';

   -- Program memory (2Kx16)
   -- Note: we don't reset it so it can address the first memory position while
   -- the CPU is reseted.
   adr_pc <= std_logic_vector(pc);
   rom: ROMLoader
      generic map(
         ADDR_W => 11, DATA_W => 16, SIZE => 2048)
      port map(
         addr_i => adr_pc(10 downto 0), data_o => inst, ck_i => clk,
         data_i => inst_w, we_i => pgm_we,
         rst_i => rst, start_load_i => load, end_load_o => loaded,
         the_file_i => hex_file);

   dummy : WB_Dummy
      port map(
         wb_clk_i => clk,     wb_rst_i => rst,   wb_adr_i => wb_adr,
         wb_dat_i => wb_dato, wb_dat_o => wb_dati, wb_we_i  => wb_we,
         wb_stb_i => wb_stb,  wb_ack_o => wb_ack);

   the_clock : SimpleClock
      generic map(FREQUENCY => CLK_FREQ*1e6)
      port map(
         clk_o => clk, nclk_o => nclk, stop_i => end_test);

   the_tracer : entity work.Tracer
      port map(
         clk_i => clk, rst_i => rst, dbg_i => dbg, ena_i => ena_trace);
   ena_trace <= '1' when SHOW_TRACE else '0';

   -- Iteration counter
   iter_counter:
   process (clk, rst)
   begin
      if rst='1' then
         iteration <= 0;
      elsif rising_edge(clk) then
         iteration <= iteration+1;
      end if;
      time_out <= '0';
      if iteration>=MAX_ITERATION then
         time_out <= '1';
      end if;
   end process iter_counter;

   -----------------------------
   -- Loop to load the tests. --
   -----------------------------
   verify:
   process
      variable num : string(1 to 36):="123456789abcdefghijklmnopqrstuvwxyz-";
      variable test_num : integer;
   begin
      outwrite("* Starting AVR tests");
      for test_num in 1 to 34 loop
          rst <= '1';
          hex_file <= "x" & num(test_num) & ".hex";
          outwrite(" + Loading ROM x" & num(test_num) & ".hex");
          load <= '1';
          wait until loaded='1';
          load <= '0';
          Reset(rst);
    
          -- First event: when we set it as output
          wait until time_out='1' or portb'event;
          assert time_out='0' report "Time out in test "&integer'image(test_num) severity failure;
          -- Second event: a real value
          wait until time_out='1' or portb'event;
          assert time_out='0' report "Time out in test "&integer'image(test_num) severity failure;
    
          if portb/=x"55" then
             report "Error in test "&integer'image(test_num)&" (x"&num(test_num)&
                    ".hex): Port A: 0x"&hstr(portb)&" PC: 0x"&hstr(pc) severity failure;
          end if;

          outwrite(" + Success detected after "&integer'image(iteration)&" clocks");
      end loop;
      outwrite("* End of test");

      end_test <= '1';
      wait;
   end process verify;

end architecture Simulator; -- Entity: CPU_RL_Tb

