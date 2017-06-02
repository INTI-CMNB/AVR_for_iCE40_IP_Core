------------------------------------------------------------------------------
----                                                                      ----
----  AVR 2.5 CPU with some aspects of ATtinyX5 CPUs                      ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  ATtiny25/45/85 style CPU. Not exactly the same, just the same       ----
----  memory layout and instruction set.                                  ----
----  It also implements the SPM instruction.                             ----
----  Ports B, C and D are available.                                     ----
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
---- Design unit:      ATtX5(Struct) (Entity and architecture)            ----
---- File name:        attX5.vhd                                          ----
---- Note:             None                                               ----
---- Limitations:      None known                                         ----
---- Errors:           None known                                         ----
---- Library:          avr                                                ----
---- Dependencies:     IEEE.std_logic_1164                                ----
----                   IEEE.numeric_std                                   ----
----                   avr.Core                                           ----
----                   avr.Constants                                      ----
----                   avr.Types                                          ----
----                   avr.Devices                                        ----
----                   avr.Memory                                         ----
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
use avr.Core.all;
use avr.Constants.all;
use avr.Types.all;
use avr.Devices.all;
use avr.Memory.all;

entity ATtX5 is
   generic(
      ENA_TC0       : boolean:=true;   -- Include the Timer/Counter 0
      ENA_PORTB     : boolean:=true;   -- Include PortB
      ENA_PORTC     : boolean:=false;  -- Include PortC (experimental)
      ENA_PORTD     : boolean:=false;  -- Include PortD (experimental)
      ENA_WB        : boolean:=true;   -- Include the WISHBONE bridge
      ENA_IRQ_CTRL  : std_logic:='0';  -- Include the ext. irq. control
      ENA_DEBUG     : boolean:=false;  -- Enable debug interface
      ENA_SPM       : boolean:=false;  -- Implement the SPM instruction (AVR4)
      ENA_AVR25     : boolean:=true;   -- Enable AVR 2.5 instructions
      PORTB_SIZE    : positive:=5;     -- PORTB implemented bits
      PORTC_SIZE    : positive:=6;     -- PORTC implemented bits
      PORTD_SIZE    : positive:=8;     -- PORTD implemented bits
      ENA_SPI       : boolean:=false;  -- Include SPI
      RF_ENA_RST    : boolean:=false;  -- Register File Reset Enable
      RAM_ADDR_W    : positive:=8;     -- tn25: 7 45: 8 85: 9 (128 to 512 b)
      RESET_JUMP    : natural:=0);     -- Address of the reset vector
   port(
      clk_i      : in    std_logic;
      clk2x_i    : in    std_logic;
      rst_i      : in    std_logic;
      ena_i      : in    std_logic:='1'; -- CPU clock enable
      tmr_ena_i  : in    std_logic:='1'; -- T/C clock enable
      -- Ports
      portb_i    : in    std_logic_vector(PORTB_SIZE-1 downto 0):=(others => '0');
      portc_i    : in    std_logic_vector(PORTC_SIZE-1 downto 0):=(others => '0');
      portd_i    : in    std_logic_vector(PORTD_SIZE-1 downto 0):=(others => '0');
      portb_o    : out   std_logic_vector(PORTB_SIZE-1 downto 0);
      portc_o    : out   std_logic_vector(PORTC_SIZE-1 downto 0);
      portd_o    : out   std_logic_vector(PORTD_SIZE-1 downto 0);
      -- Program Memory
      pc_o       : out   unsigned(15 downto 0); -- PROM address
      inst_i     : in    std_logic_vector(15 downto 0); -- PROM data
      inst_o     : out   std_logic_vector(15 downto 0); -- PROM data
      pgm_we_o   : out   std_logic; -- PROM WE
      -- External device interrupts (external UART, Timer, etc.)
      dev_irq_i  : in    std_logic_vector(2 downto 0):="000";
      dev_ack_o  : out   std_logic_vector(2 downto 0);
      -- External PIN interrupts
      pin_irq_i  : in    std_logic_vector(1 downto 0):="00";
      -- WISHBONE
      wb_adr_o   : out   std_logic_vector(7 downto 0); -- I/O Address
      wb_dat_o   : out   std_logic_vector(7 downto 0); -- Data Bus output
      wb_dat_i   : in    std_logic_vector(7 downto 0):="00000000"; -- Data Bus input
      wb_stb_o   : out   std_logic;  -- Strobe output
      wb_we_o    : out   std_logic;  -- Write Enable output
      wb_ack_i   : in    std_logic:='1'; -- Acknowledge input
      -- SPI
      spi_ena_o  : out   std_logic;
      sclk_o     : out   std_logic;
      miso_i     : in    std_logic:='0';
      mosi_o     : out   std_logic;
      -- Debug
      dbg_o      : out debug_o_t;  -- Debug status
      dbg_i      : in  debug_i_t:=DEBUG_I_INIT); -- Debug control
end entity ATtX5;

architecture Struct of ATtX5 is
   constant EXT_BUS_W : positive:=6; -- 6 peripherals
   constant IRQ_NUM   : positive:=5; -- Number of IRQ lines
   constant ID_W      : positive:=3; -- Width of the ID
   -- i.e. 512 => 9 bits, but from 0x60 to 0x25F => 10 bits
   constant ADR_W     : positive:=RAM_ADDR_W+1;

   -- I/O registers
   signal io_adr    : unsigned(5 downto 0);
   signal io_re     : std_logic;
   signal io_we     : std_logic;
   -- Data bus
   signal core_din  : std_logic_vector (7 downto 0);
   signal core_dout : std_logic_vector (7 downto 0);
   -- Interrupts
   signal irq_lines : std_logic_vector(IRQ_NUM-1 downto 0):=(others => '0');
   signal irq_acks  : std_logic_vector(irq_lines'range);
   -- RAM
   signal ram_din_r : std_logic_vector(7 downto 0);
   signal ram_clk   : std_logic; -- RAM clock
   signal ram_datao : std_logic_vector(7 downto 0);
   signal ram_adr   : std_logic_vector(RAM_ADDR_W downto 0);
   signal ram_adr_2 : std_logic_vector(RAM_ADDR_W-1 downto 0);
   signal ram_we    : std_logic;
   signal ram_re    : std_logic;
   -- I/O mux
   signal io_out    : ext_mux_t(0 to EXT_BUS_W-1):=(others => (others => '0'));
   signal io_out_en : std_logic_vector(EXT_BUS_W-1 downto 0):=(others => '0');
   -- WISHBONE
   signal wb_go_on  : std_logic:='1';
   signal cpu_ena   : std_logic;
   -- Watchdog
   signal cpu_rst   : std_logic;
begin
   AVRCoreInst : entity avr.AVRCore
      generic map(
         RF_ENA_RST => RF_ENA_RST, ID_W => ID_W, IRQ_LINES => IRQ_NUM,
         ENA_RAMPZ => false, SP_W => ADR_W, RAM_ADR_W => ADR_W, ENA_AVR3 => false,
         ENA_DEBUG => ENA_DEBUG, ENA_SPM => ENA_SPM, RESET_JUMP => RESET_JUMP,
         ENA_AVR4 => false, ENA_AVR25 => ENA_AVR25)
      port map(
         --Clock and reset
         clk_i => clk_i, ena_i => cpu_ena, rst_i => rst_i,
         -- Program Memory
         pc_o => pc_o, inst_i => inst_i, inst_o => inst_o, pgm_we_o => pgm_we_o,
         -- I/O control
         io_adr_o => io_adr, io_re_o => io_re, io_we_o => io_we,
         -- Data memory control
         ram_adr_o => ram_adr, ram_re_o => ram_re, ram_we_o => ram_we,
         -- Data paths
         io_data_i => core_din, data_o => core_dout,
         ram_data_i => ram_datao,
         -- Interrupts
         irq_lines_i => irq_lines, irq_ok_o => open, irq_acks_o => irq_acks,
         irq_ena_o => open,
         -- Sleep Control
         sleep_o => open,
         --Watchdog
         wdr_o => open,
         -- Debug
         dbg_o => dbg_o, dbg_i => dbg_i);
   cpu_ena <= wb_go_on and ena_i;
   cpu_rst <= rst_i;

   do_mux:
   process (io_out_en, io_out)
      variable do : std_logic_vector(7 downto 0);
   begin
      do:=(others => '0');
      for i in io_out_en'range loop
          if io_out_en(i)='1' then
             do:=io_out(i);
          end if;
      end loop;
      core_din <= do;
   end process do_mux;

   -- Used IRQ lines (add 2 for the "vector number"), address=(irq+1)*2
   -- 0 External PIN INT0
   -- 1 External PIN INT1
   -- 2 External Device Interrupt 0
   -- 3 External Device Interrupt 1
   -- 4 External Device Interrupt 2 (16 bits Timer)
   dev_ack_o <= irq_acks(4 downto 2);
   
   -- Used io_out_en
   -- 0 PortB
   -- 1 PortC
   -- 2 WB bridge
   -- 3 IRQ Ctrl
   -- 4 PortD for experimental use
   -- 5 SPI

   -----------
   -- PortB --
   -----------
   portb_impl :
   if ENA_PORTB generate
      portb_comp : IOPort
         generic map(NUMBER => 1, BITS => PORTB_SIZE)
         port map(
            -- AVR Control
            clk_i      => clk_i,
            rst_i      => rst_i,
            ena_i      => ena_i,
            adr_i      => io_adr,
            data_i     => core_dout,
            data_o     => io_out(0),
            re_i       => io_re,
            we_i       => io_we,
            selected_o => io_out_en(0),
            -- External connection
            port_i     => portb_i,
            port_o     => portb_o);
   end generate portb_impl;
   
   portb_not_impl:
   if not ENA_PORTB generate
      portb_o <= (others => 'Z');
   end generate portb_not_impl;
   -- ************************************************

   -----------
   -- PortC -- -- For experimental use
   -----------
   portc_impl :
   if ENA_PORTC generate
      portc_comp : IOPort
         generic map(NUMBER => 2, BITS => PORTC_SIZE)
         port map(
            -- AVR Control
            clk_i      => clk_i,
            rst_i      => rst_i,
            ena_i      => ena_i,
            adr_i      => io_adr,
            data_i     => core_dout,
            data_o     => io_out(1),
            re_i       => io_re,
            we_i       => io_we,
            selected_o => io_out_en(1),
            -- External connection
            port_i     => portc_i,
            port_o     => portc_o);
   end generate portc_impl;
   
   portc_not_impl:
   if not ENA_PORTC generate
      portc_o <= (others => 'Z');
   end generate portc_not_impl;
   -- ************************************************

   -----------
   -- PortD -- -- For experimental use
   -----------
   portd_impl :
   if ENA_PORTD generate
      portd_comp : IOPort
         generic map(NUMBER => 3, BITS => PORTD_SIZE)
         port map(
            -- AVR Control
            clk_i      => clk_i,
            rst_i      => rst_i,
            ena_i      => ena_i,
            adr_i      => io_adr,
            data_i     => core_dout,
            data_o     => io_out(4),
            re_i       => io_re,
            we_i       => io_we,
            selected_o => io_out_en(4),
            -- External connection
            port_i     => portd_i,
            port_o     => portd_o);
   end generate portd_impl;
   
   portd_not_impl:
   if not ENA_PORTD generate
      portd_o <= (others => 'Z');
   end generate portd_not_impl;
   -- ************************************************

   ---------------------
   -- WISHBONE bridge --
   ---------------------
   WB_Impl:
   if ENA_WB generate
      wb_bridge : WBControl
         port map(
            -- AVR Control
            rst_i      => rst_i,
            clk_i      => clk_i,
            ena_o      => wb_go_on,
            -- I/O Bus
            adr_i      => io_adr,
            data_i     => core_dout,
            data_o     => io_out(2),
            re_i       => io_re,
            we_i       => io_we,
            selected_o => io_out_en(2),
            -- WISHBONE side
            wb_adr_o => wb_adr_o, wb_dat_o => wb_dat_o, wb_dat_i => wb_dat_i,
            wb_stb_o => wb_stb_o, wb_we_o  => wb_we_o,  wb_ack_i => wb_ack_i);
   end generate WB_Impl;

   ---------------------------------
   -- External Interrupts Control --
   ---------------------------------
   irq_ctrl : IRQCtrl
      generic map(
         ENABLE => ENA_IRQ_CTRL)
      port map(
         -- AVR Control
         clk_i      => clk_i,
         rst_i      => rst_i,
         ena_i      => ena_i,
         adr_i      => io_adr,
         data_i     => core_dout,
         data_o     => io_out(3),
         re_i       => io_re,
         we_i       => io_we,
         selected_o => io_out_en(3),
         -- External connection
          -- 3 peripherals
         dev_irq_i => dev_irq_i,
          -- 2 PINs
         pin_irq_i => pin_irq_i,
         irq_ack_i => irq_acks(1 downto 0),
          -- 5 lines for the CPU
         ext_irq_o  => irq_lines);

   ---------
   -- SPI --
   ---------
   SPI_Impl:
   if ENA_SPI generate
      SPI_Device : SPI_Dev
         port map(
            -- AVR Control
            clk_i      => clk_i,
            rst_i      => rst_i,
            ena_i      => ena_i,
            adr_i      => io_adr,
            data_i     => core_dout,
            data_o     => io_out(5),
            re_i       => io_re,
            we_i       => io_we,
            selected_o => io_out_en(5),
            -- IRQ control. Currently disabled
            irq_req_o  => open,
            irq_ack_i  => '0',
            -- External connection
            mux_en_o   => spi_ena_o,
            -- SPI
            clk2x_i    => clk2x_i,
            sclk_o     => sclk_o,
            miso_i     => miso_i,
            mosi_o     => mosi_o);
   end generate SPI_Impl;

   SPI_Not_Impl:
   if not(ENA_SPI) generate
      spi_ena_o <= '0';
      sclk_o    <= '0';
      mosi_o    <= '0';
   end generate SPI_Not_Impl;

   ------------
   -- Memory --
   ------------
   -- Memory configuration (DM 128/256/512x8)
   DRAM_Inst : SinglePortRAM
      generic map(
         WORD_SIZE => 8, ADDR_W => RAM_ADDR_W)
      port map(
         clk_i => clk_i, we_i => ram_we,
         addr_i => ram_adr_2,
         d_i => core_dout, d_o => ram_datao);
   -- ram_adr valid values range from 0x60 to 0x0DF we use the 7 LSBs
   -- ram_adr valid values range from 0x60 to 0x15F we use the 8 LSBs
   -- ram_adr valid values range from 0x60 to 0x25F we use the 9 LSBs
   ram_adr_2 <= ram_adr(RAM_ADDR_W-1 downto 0);

end architecture Struct; -- Entity: ATtX5

