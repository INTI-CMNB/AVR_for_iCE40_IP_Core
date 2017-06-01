------------------------------------------------------------------------------
----                                                                      ----
----  SPI_Dev [Master only]                                               ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  SPI implementation. Not 100% but enough for the Arduino API.        ----
----                                                                      ----
----  To Do:                                                              ----
----  -                                                                   ----
----                                                                      ----
----  Author:                                                             ----
----    - Salvador E. Tropea, salvador en inti.gob.ar                     ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Copyright (c) 2009-2017 Salvador E. Tropea <salvador en inti.gob.ar> ----
---- Copyright (c) 2009-2017 Instituto Nacional de Tecnología Industrial  ----
----                                                                      ----
---- Distributed under the GPL v2 or newer license                        ----
----                                                                      ----
------------------------------------------------------------------------------
----                                                                      ----
---- Design unit:      SPI_Dev(RTL) (Entity and architecture)             ----
---- File name:        spi.vhdl                                           ----
---- Note:             None                                               ----
---- Limitations:      None known                                         ----
---- Errors:           None known                                         ----
---- Library:          avr                                                ----
---- Dependencies:     IEEE.std_logic_1164                                ----
----                   IEEE.numeric_std                                   ----
----                   SPI.Devices                                        ----
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
library SPI;
use SPI.Devices.all;
library avr;
use avr.Constants.all;

entity SPI_Dev is
   generic(
      ENABLE      : std_logic:='1';
      WCOL_ENABLE : std_logic:='0');
   port(
      -- AVR Control
      clk_i      : in   std_logic;
      rst_i      : in   std_logic;
      ena_i      : in   std_logic;
      adr_i      : in   unsigned(5 downto 0);
      data_i     : in   std_logic_vector(7 downto 0);
      data_o     : out  std_logic_vector(7 downto 0);
      re_i       : in   std_logic;
      we_i       : in   std_logic;
      selected_o : out  std_logic;
      -- IRQ control
      irq_req_o  : out  std_logic;
      irq_ack_i  : in   std_logic:='0';
      -- External connection
      mux_en_o   : out  std_logic;
      -- SPI
      clk2x_i    : in   std_logic;
      sclk_o     : out  std_logic;
      miso_i     : in   std_logic;
      mosi_o     : out  std_logic);
end entity SPI_Dev;

architecture RTL of SPI_Dev is
   -- Registers
   signal spc_r     : std_logic_vector(7 downto 0):=x"00";
   signal sps       : std_logic_vector(7 downto 0);
   -- Address decoding
   signal spcr_sel  : std_logic; -- Control
   signal spsr_sel  : std_logic; -- Status
   signal spdr_sel  : std_logic; -- Data
   -- SPI core signals
   signal start     : std_logic;
   signal rx_data   : std_logic_vector(7 downto 0);
   signal busy      : std_logic;
   signal irq_req   : std_logic;
   signal irq_ack   : std_logic;
   -- SPI core configuration
   alias  cpol      : std_logic is spc_r(3); -- Clock Polarity
   alias  dord      : std_logic is spc_r(5); -- Data Order
   alias  cpha      : std_logic is spc_r(2); -- Clock Phase
   alias  spr       : std_logic_vector(1 downto 0) is spc_r(1 downto 0); -- Rate
   signal spi2x_r   : std_logic:='0'; -- 2x rate
   -- Rate generator
   signal spi_clk   : std_logic; -- SPI clock enable
   signal spi_ena_p : std_logic;
   signal spi_ena_2 : std_logic;
   signal ena_div1  : std_logic;
   signal ena_div4  : std_logic;
   signal ena_div16 : std_logic;
   signal ena_div32 : std_logic;
   signal ena_div2x : std_logic;
   signal cnt_r     : unsigned(4 downto 0):="00000";
   -- Other signals
   alias  spif      : std_logic is irq_req;  -- Interrupt Flag
   alias  spie      : std_logic is spc_r(7); -- Interrupt Enable
   alias  spe       : std_logic is spc_r(6); -- SPI Enable
   signal wcol_r    : std_logic:='0'; -- Write Collision
begin
   spcr_sel <= '1' when adr_i=SPCR_ADDRESS and ENABLE='1' else '0';
   spsr_sel <= '1' when adr_i=SPSR_ADDRESS and ENABLE='1' else '0';
   spdr_sel <= '1' when adr_i=SPDR_ADDRESS and ENABLE='1' else '0';

   selected_o <= (spcr_sel or spsr_sel or spdr_sel) and (re_i or we_i);

   -- SPE: SPI enable
   mux_en_o <= spe;
   -- The IRQ flag is in the SPI device
   irq_req_o <= spif and spie;
   -- The IRQ is ack'd reading SPDR or by CPU ack
   -- Note: writing to the register also clears the flag
   -- TODO: I think we must check a read from SPSR
   irq_ack <= irq_ack_i or
              (spdr_sel and (re_i or we_i) and ena_i);
   -- Start Tx
   start <= spdr_sel and we_i and ena_i;

   SPI_Core : SPI_Master
      generic map(DATA_W => 8)
      port map(
         -- System
         clk_i => clk2x_i, rst_i => rst_i, ena_i => spi_clk,
         -- Interface
         start_i => start, tx_i => data_i, rx_o => rx_data,
         busy_o => busy, irq_o => irq_req, ack_i => irq_ack,
         -- Mode options
         cpol_i => cpol, dord_i => dord, cpha_i => cpha,
         -- SPI
         sclk_o => sclk_o, miso_i => miso_i, mosi_en_o => open,
         mosi_o => mosi_o);

   do_regs:
   process (clk_i)
   begin
      if rising_edge(clk_i) then
         if rst_i='1' then
            spc_r <= (others => '0');
            wcol_r  <= '0';
            spi2x_r <= '0';
         else
            -- CPU Write to registers
            if we_i='1' and ena_i='1' then
               if spcr_sel='1' then
                  spc_r <= data_i;
               end if;
               if spsr_sel='1' then
                  spi2x_r <= data_i(0);
               end if;
            end if;
            -- WCOL flag
            if WCOL_ENABLE='1' then
               -- Write Collision
               if start='1' and busy='1' then
                  wcol_r <= '1';
               elsif spdr_sel='1' and re_i='1' and ena_i='1' then
                  -- Reading SPDR this flag is cleared
                  -- TODO: I think we must check a read from SPSR
                  wcol_r <= '0';
               end if;
            end if; -- WCOL_ENABLE
         end if; -- else rst_i='1'
      end if; -- rising_edge(clk_i)
   end process do_regs;

   sps <= spif&wcol_r&"00000"&spi2x_r;
   data_o <= (spc_r   and spcr_sel) or
             (sps     and spsr_sel) or
             (rx_data and spdr_sel);

   -- Rate generator
   -- Clock divider /2 to /32
   do_cnt_r:
   process (clk2x_i)
   begin
      if rising_edge(clk2x_i) then
         if rst_i='1' or start='1' then
            cnt_r <= (others => '0');
         else
            cnt_r <= cnt_r+1;
         end if;
      end if;
   end process do_cnt_r;

   -- SPR selection
   ena_div1  <= '1';
   ena_div4  <= '1' when cnt_r(1 downto 0)="11" else '0';
   ena_div16 <= '1' when cnt_r(3 downto 0)="1111" else '0';
   ena_div32 <= '1' when cnt_r(4 downto 0)="11111" else '0';

   with spr select spi_ena_p <=
        ena_div1  when "00", -- Direct
        ena_div4  when "01", -- /4
        ena_div16 when "10", -- /16
        ena_div32 when others; -- "11" 1/32

   -- Extra /2
   do_spi_clk_2:
   process (clk2x_i)
   begin
      if rising_edge(clk2x_i) then
         if rst_i='1' or start='1' then
            spi_ena_2 <= '0';
         elsif spi_ena_p='1' then
            spi_ena_2 <= not(spi_ena_2);
         end if;
      end if;
   end process do_spi_clk_2;
   ena_div2x <= '1' when (spi_ena_2 and spi_ena_p)='1' else '0';

   -- SPI2X selector
   spi_clk <= spi_ena_p when spi2x_r='1' else ena_div2x;
end architecture RTL; -- Entity: SPI_Dev

