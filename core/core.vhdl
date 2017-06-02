------------------------------------------------------------------------------
----                                                                      ----
----  AVR CPU core (CPU+Satus+Stack Pointer)                              ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  This joins all the CPU with some basic I/O registers, like the      ----
----  status register and the stack pointer.                              ----
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
---- Design unit:      AVRCore(Struct) (Entity and architecture)          ----
---- File name:        core.vhdl                                          ----
---- Note:             None                                               ----
---- Limitations:      None known                                         ----
---- Errors:           None known                                         ----
---- Library:          avr                                                ----
---- Dependencies:     IEEE.std_logic_1164                                ----
----                   IEEE.numeric_std                                   ----
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

entity AVRCore is
   generic(
      SP_W       : integer range 8 to 16:=16; -- Stack pointer size
      RAM_ADR_W  : integer range 8 to 16:=16; -- RAM address width
      ENA_RAMPZ  : boolean:=false; -- RAMPZ enable
      ID_W       : positive:=5;    -- Width of the IRQ ID
      ENA_AVR25  : std_logic:='0'; -- Enable AVR25 instructions (MOVW/LPM Rd,Z)
      ENA_AVR3   : boolean:=true;  -- Enable AVR3 instructions
      ENA_AVR4   : boolean:=false; -- Enable AVR4 instructions
      ENA_SPM    : std_logic:='0'; -- Enable SPM instructions
      ENA_DEBUG  : std_logic:='0'; -- Enable debug interface
      RESET_JUMP : natural:=0;     -- Address of the reset vector
      IRQ_LINES  : positive:=23);  -- Number of IRQ lines
   port(
      -- Clock and reset
      clk_i        : in  std_logic;
      ena_i        : in  std_logic;
      rst_i        : in  std_logic;
      -- Program Memory
      pc_o         : out unsigned(15 downto 0);
      inst_i       : in  std_logic_vector(15 downto 0);
      inst_o       : out std_logic_vector(15 downto 0);
      pgm_we_o     : out std_logic;
      -- I/O control
      io_adr_o     : out unsigned(5 downto 0);
      io_re_o      : out std_logic;
      io_we_o      : out std_logic;
      io_data_i    : in  std_logic_vector(7 downto 0);
      -- Data memory control
      ram_adr_o    : out std_logic_vector(RAM_ADR_W-1 downto 0);
      ram_re_o     : out std_logic;
      ram_we_o     : out std_logic;
      ram_data_i   : in  std_logic_vector(7 downto 0);
      -- Data paths
      data_o       : out std_logic_vector(7 downto 0);
      -- Interrupt
      irq_lines_i  : in  std_logic_vector(IRQ_LINES-1 downto 0);
      irq_acks_o   : out std_logic_vector(IRQ_LINES-1 downto 0);
      irq_ok_o     : out std_logic;
      irq_ena_o    : out std_logic; -- SREG(I)
      -- Sleep Instruction
      sleep_o      : out std_logic;
      -- Watchdog Reset Instruction
      wdr_o        : out std_logic;
      -- Debug
      dbg_stop_i      : in  std_logic; -- Stop request
      dbg_pc_o        : out unsigned(15 downto 0);
      dbg_inst_o      : out std_logic_vector(15 downto 0);
      dbg_inst2_o     : out std_logic_vector(15 downto 0);
      dbg_exec_o      : out std_logic;
      dbg_is32_o      : out std_logic;
      dbg_stopped_o   : out std_logic; -- CPU is stopped
      -- Debug used for Test_ALU_1_TB
      dbg_rf_fake_i   : in  std_logic;
      dbg_rr_data_i   : in  std_logic_vector(7 downto 0);
      dbg_rd_data_i   : in  std_logic_vector(7 downto 0);
      dbg_rd_data_o   : out std_logic_vector(7 downto 0);
      dbg_rd_we_o     : out std_logic;
      dbg_cyc_last_o  : out std_logic); -- Last cycle in the instruction
end entity AVRCore;

architecture Struct of AVRCore is
   -- I/O Registers File
   -- Input Data after the IORegFile block
   signal io_datai    : std_logic_vector(7 downto 0);
   -- Output Data (also used internally)
   signal datao       : std_logic_vector(7 downto 0);
   -- I/O control (also used internally)
   signal io_adr      : unsigned(5 downto 0);
   signal io_we       : std_logic;
   signal io_re       : std_logic;
   -- I/O Registers
   signal new_sreg    : std_logic_vector(7 downto 0);
   signal sreg        : std_logic_vector(7 downto 0);
   signal sreg_we     : std_logic_vector(7 downto 0);
   signal sp          : unsigned(SP_W-1 downto 0);
   signal rampz       : std_logic_vector(7 downto 0);
   signal sp_pop      : std_logic;
   signal sp_we       : std_logic;
begin
   CPU: entity avr.CPU
      generic map(
         ENA_DEBUG => ENA_DEBUG, RAM_ADR_W => RAM_ADR_W,
         SP_W => SP_W, ENA_AVR4 => ENA_AVR4, ENA_AVR3 => ENA_AVR3,
         ENA_SPM => ENA_SPM, RESET_JUMP => RESET_JUMP,
         ENA_AVR25 => ENA_AVR25, IRQ_ID_W => ID_W,
         IRQ_LINES => IRQ_LINES)
      port map(
         -- Clock and reset
         clk_i       => clk_i,
         ena_i       => ena_i,
         rst_i       => rst_i,
         -- Program memory
         pc_o        => pc_o,
         inst_i      => inst_i,
         inst_o      => inst_o,
         pgm_we_o    => pgm_we_o,
         -- I/O control
         io_adr_o    => io_adr,
         io_re_o     => io_re,
         io_we_o     => io_we,
         io_data_i   => io_datai,
         -- Data memory control
         ram_adr_o   => ram_adr_o,
         ram_re_o    => ram_re_o,
         ram_we_o    => ram_we_o,
         ram_data_i  => ram_data_i,
         -- Data paths
         data_o      => datao,
         -- Interrupt
         irq_lines_i => irq_lines_i,
         irq_acks_o  => irq_acks_o,
         --Sleep 
         sleep_o      => sleep_o,
         irq_ok_o     => irq_ok_o,
         --Watchdog
         wdr_o        => wdr_o,
         -- I/O register file interface
         sreg_o       => new_sreg,
         sreg_i       => sreg,
         sreg_we_o    => sreg_we,
         sp_i         => sp,
         sp_pop_o     => sp_pop,
         sp_we_o      => sp_we,
         rampz_i      => rampz,
         -- Debug
         dbg_stop_i     => dbg_stop_i,
         dbg_pc_o       => dbg_pc_o,
         dbg_inst_o     => dbg_inst_o,
         dbg_inst2_o    => dbg_inst2_o,
         dbg_exec_o     => dbg_exec_o,
         dbg_is32_o     => dbg_is32_o,
         dbg_stopped_o  => dbg_stopped_o,
         -- Debug used for Test_ALU_1_TB
         dbg_rf_fake_i  => dbg_rf_fake_i,
         dbg_rr_data_i  => dbg_rr_data_i,
         dbg_rd_data_i  => dbg_rd_data_i,
         dbg_rd_data_o  => dbg_rd_data_o,
         dbg_rd_we_o    => dbg_rd_we_o,
         dbg_cyc_last_o => dbg_cyc_last_o);
   
   IORegs_Inst: IORegFile
      generic map(
         ENA_RAMPZ => ENA_RAMPZ, SP_W => SP_W)
      port map(
         -- Clock and reset
         clk_i      => clk_i,
         ena_i      => ena_i,
         rst_i      => rst_i,
         -- I/O Bus
         adr_i      => io_adr,
         we_i       => io_we,
         re_i       => io_re,
         data_i     => datao,
         data_o     => io_datai, -- Local data bus
         e_data_i   => io_data_i,    -- External data bus
         -- Status Register
         sreg_i     => new_sreg,
         sreg_o     => sreg,
         sreg_we_i  => sreg_we,
         -- Stack Pointer
         sp_o       => sp,
         sp_pop_i   => sp_pop,
         sp_we_i    => sp_we,
         -- RAM Page Z
         rampz_o    => rampz);
   
   -- Outputs
   io_adr_o  <= io_adr;
   io_we_o   <= io_we;
   io_re_o   <= io_re;
   data_o    <= datao;
   -- Sleep support
   irq_ena_o <= sreg(7); -- I flag
end architecture Struct; -- Entity: AVRCore

