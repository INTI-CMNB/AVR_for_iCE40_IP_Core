------------------------------------------------------------------------------
----                                                                      ----
----  Testbench for the AVR, ATtX5 configuration.                         ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  Tests hello6.c using an ATtinyX5. It transmits an string using      ----
----  the WISHBONE miniuart.                                              ----
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
---- Design unit:      ATtX5_Tb(Behave) (Entity and architecture)         ----
---- File name:        ATtX5_Tb.vhdl                                      ----
---- Note:             None                                               ----
---- Limitations:      None known                                         ----
---- Errors:           None known                                         ----
---- Library:          work                                               ----
---- Dependencies:     IEEE.std_logic_1164                                ----
----                   IEEE.numeric_std                                   ----
----                   avr.Micros                                         ----
----                   avr.Types                                          ----
----                   work.PrgMems                                       ----
----                   utils.StdIO                                        ----
----                   utils.Str                                          ----
----                   utils.StdLib                                       ----
----                   miniuart.UART                                      ----
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
use avr.Micros.all;
use avr.Types.all;
library work;
use work.PrgMems.all;
library utils;
use utils.StdIO.all;
use utils.Str.all;
use utils.StdLib.all;
library miniuart;
use miniuart.UART.all;
library wb_handler;
use wb_handler.WishboneTB.all;

entity ATtX5_Tb is
end entity ATtX5_Tb;

architecture Behave of ATtX5_Tb is
   constant F_CLK      : natural:=333e3;
   constant BAUD_RATE  : natural:=115200;
   constant BRDIVISOR  : natural:=natural(real(F_CLK)/real(BAUD_RATE)/4.0+0.5);
   constant REF_MSG    : string(1 to 27):="ATtinyX5 says: hello world!";
   constant PERIOD     : time:=1 sec/real(F_CLK);

   signal clk          : std_logic;
   signal nclk         : std_logic;
   signal reset        : std_logic:='1';
   signal stop         : std_logic:='0';

   signal wb_adr       : std_logic_vector(7 downto 0); -- I/O Address
   signal wb_dato      : std_logic_vector(7 downto 0); -- Data Bus output
   signal wb_dati      : std_logic_vector(7 downto 0):="00000000"; -- Data Bus input
   signal wb_stb       : std_logic;  -- Strobe output
   signal wb_we        : std_logic;  -- Write Enable output
   signal wb_ack       : std_logic:='1'; -- Acknowledge input
   signal adr          : std_logic_vector(0 downto 0);

   -- UART used to debug the reception
   signal rx_irq       : std_logic;
   signal wbi          : wb_bus_i_type;
   signal wbo          : wb_bus_o_type;

   signal rs232_tx     : std_logic;
   signal rs232_rx     : std_logic;
   signal pc           : unsigned(15 downto 0); -- PROM address
   signal pcsv         : std_logic_vector(10 downto 0);
   signal inst         : std_logic_vector(15 downto 0); -- PROM data
   signal dbg          : debug_o_t;
begin
   micro : entity avr.ATtX5
      generic map(
         ENA_TC0 => false,   ENA_PORTB => false, ENA_WB    => true,
         ENA_DEBUG => true)
      port map(
         rst_i => reset, clk_i => clk, clk2x_i => clk,
         pc_o => pc, inst_i => inst, dbg_o => dbg,
         -- WISHBONE
         wb_adr_o => wb_adr, wb_dat_o => wb_dato, wb_dat_i => wb_dati,
         wb_stb_o => wb_stb, wb_we_o  => wb_we,   wb_ack_i => wb_ack);

   pcsv <= std_logic_vector(pc(10 downto 0));
   -- Program memory (2Kx16)
   PM_Inst : HelloPM
      generic map(
         WORD_SIZE => 16, ADDR_W => 11)
      port map(
         clk_i => clk, addr_i => pcsv, data_o => inst);

   adr <= wb_adr(0 downto 0);
   the_uart : UART_C
     generic map(BRDIVISOR => BRDIVISOR) -- Baud rate divisor
     port map(
        -- Wishbone signals
        wb_clk_i => clk,     wb_rst_i => reset,   wb_adr_i => adr,
        wb_dat_i => wb_dato, wb_dat_o => wb_dati, wb_we_i  => wb_we,
        wb_stb_i => wb_stb,  wb_ack_o => wb_ack,
        -- Process signals
        inttx_o  => open,    intrx_o => open,     br_clk_i => '1',
        txd_pad_o => rs232_tx, rxd_pad_i => rs232_rx);

   debug_uart : UART_C
     generic map(BRDIVISOR => BRDIVISOR) -- Baud rate divisor
     port map(
        -- Wishbone signals
        wb_clk_i => clk,       wb_rst_i => reset,     wb_adr_i => wbo.adr(0 downto 0),
        wb_dat_i => wbo.dati,  wb_dat_o => wbi.dato,  wb_we_i  => wbo.we,
        wb_stb_i => wbo.stb,   wb_ack_o => wbi.ack,
        -- Process signals
        inttx_o  => open,      intrx_o => rx_irq,     br_clk_i => '1',
        txd_pad_o => rs232_rx, rxd_pad_i => rs232_tx);
   wbi.clk <= clk;
   wbi.rst <= reset;

   the_tracer : entity work.Tracer
      port map(
         clk_i => clk, rst_i => reset, dbg_i => dbg);

   the_clock : SimpleClock
      generic map(FREQUENCY => F_CLK, RESET_TM => 1.2)
      port map(
         clk_o => clk, nclk_o => nclk, rst_o => reset, stop_i => stop);

   do_receive:
   process
      variable s : string(1 to 80);
      variable c : character:=' ';
      variable v : integer:=0;
      variable i : integer:=1;
   begin
      while v/=10 loop
         wait until rx_irq='1' for 4500*PERIOD;
         assert rx_irq='1'
            report "Reception time-out"
            severity failure;
         WBRead(UART_RX,wbi,wbo);
         v:=to_integer(unsigned(wbi.dato));
         c:=character'val(v);
         s(i):=c;
         i:=i+1;
         report "Recibido "&c&" ("&integer'image(v)&")" severity note;
      end loop;
      report "Linea completa: "&s severity note;
      assert i=30 report "La cantidad de caracteres no coindice con lo esperado" severity failure;
      assert s(1 to 27)=REF_MSG report "El mensaje no coincide" severity failure;
      stop <= '1';
      report "Mensaje recibido con exito!" severity note;
      wait;
   end process do_receive;
end architecture Behave; -- Entity: ATtX5_Tb

