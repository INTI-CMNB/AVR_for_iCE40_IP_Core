------------------------------------------------------------------------------
----                                                                      ----
----  External Interrupt Control stuff [GICR style for Lattuino]          ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  Implements the external interrupts control registers: GICR, GIFR    ----
----  and some bits from MCUCR. Used by ATmega8, ATtiny22 and similar     ----
----  AVRs. Note that ATmega103 uses EIMSK and other registers for the    ----
----  same purpose.@p                                                     ----
----  Note that when this module is disabled the external irq lines at    ----
----  the input are copied to the output (irqs are active high). But when ----
----  enabled, and irqs by level are selected, the external irq lines are ----
----  negated (irqs are active low).@p                                    ----
----  This file contains some extensions specific for Lattuino:           ----
----  GICR/GIFR:                                                          ----
----  B7/B6/B5 External Devices                                           ----
----  B4       ExtINT1                                                    ----
----  B3       ExtINT0                                                    ----
----  MCUCR: B3/B2 ExtINT1 Mode B1/B0 ExtINT0 Mode@p                      ----
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
---- Design unit:      IRQCtrl(RTL) (Entity and architecture)             ----
---- File name:        irq.vhdl                                           ----
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

entity IRQCtrl is
   generic(
      ENABLE : std_logic:='1');
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
      -- External connection
       -- 3 peripherals
      dev_irq_i  : in   std_logic_vector(2 downto 0);
       -- 2 PINs
      pin_irq_i  : in   std_logic_vector(1 downto 0);
      irq_ack_i  : in   std_logic_vector(1 downto 0);
       -- 5 lines for the CPU
      ext_irq_o  : out  std_logic_vector(4 downto 0));
end entity IRQCtrl;

architecture RTL of IRQCtrl is
   -- Registers
   signal gic_r     : std_logic_vector(4 downto 0):="00000"; -- B7/B6+B5/B4/B3
   signal gif_r     : std_logic_vector(1 downto 0):="00";    -- B7/B6
   signal mcuc_r    : std_logic_vector(3 downto 0):="0000"; -- B3..B0
   signal ext_irq_r : std_logic_vector(1 downto 0):="00"; -- Previous IRQ
   alias  isc0_r    : std_logic_vector(1 downto 0) is mcuc_r(1 downto 0);
   alias  isc1_r    : std_logic_vector(1 downto 0) is mcuc_r(3 downto 2);
   -- Address decoding
   signal gicr_sel  : std_logic;
   signal gifr_sel  : std_logic;
   signal mcucr_sel : std_logic;
begin
   gicr_sel  <= '1' when adr_i=GICR_ADDRESS  and ENABLE='1' else '0';
   gifr_sel  <= '1' when adr_i=GIFR_ADDRESS  and ENABLE='1' else '0';
   mcucr_sel <= '1' when adr_i=MCUCR_ADDRESS and ENABLE='1' else '0';

   selected_o <= (gicr_sel or gifr_sel or mcucr_sel) and (re_i or we_i);

   do_regs:
   process (clk_i)
   begin
      if rising_edge(clk_i) then
         ext_irq_r <= pin_irq_i; -- Previous state
         if rst_i='1' then
            gic_r  <= (others => '0');
            gif_r  <= (others => '0');
            mcuc_r <= (others => '0');
         else
            -- General Interrupt Flag irq ack
            gif_r <= gif_r and not(irq_ack_i);
            -- Write to registers
            if we_i='1' and ena_i='1' then
               -- Global Interrupt Control [0x3B]
               if gicr_sel='1' then
                  gic_r <= data_i(7 downto 3);
               end if;
               -- General Interrupt Flag [0x3A]
               -- Writing 1 clear the flag
               if gifr_sel='1' then
                  gif_r <= gif_r and not(data_i(4 downto 3));
               end if;
               -- MCU Control [0x35]
               if mcucr_sel='1' then
                  isc0_r <= data_i(1 downto 0);
                  isc1_r <= data_i(3 downto 2);
               end if;
            end if; -- we_i='1' and ena_i='1'
            -- General Interrupt Flag logic
            case isc0_r is
                 when "01" =>
                      -- By change
                      if ext_irq_r(0)/=pin_irq_i(0) then
                         gif_r(0) <= '1';
                      end if;
                 when "10" =>
                      -- Falling edge
                      if ext_irq_r(0)='1' and pin_irq_i(0)='0' then
                         gif_r(0) <= '1';
                      end if;
                 when "11" =>
                      -- Rising edge
                      if ext_irq_r(0)='0' and pin_irq_i(0)='1' then
                         gif_r(0) <= '1';
                      end if;
                 when others => -- "00" xst is crazy ...
                      -- By level, this flag is always 0
                      gif_r(0) <= '0';
            end case;
            case isc1_r is
                 when "01" =>
                      -- By change
                      if ext_irq_r(1)/= pin_irq_i(1) then
                         gif_r(1) <= '1';
                      end if;
                 when "10" =>
                      -- Falling edge
                      if ext_irq_r(1)='1' and pin_irq_i(1)='0' then
                         gif_r(1) <= '1';
                      end if;
                 when "11" =>
                      -- Rising edge
                      if ext_irq_r(1)='0' and pin_irq_i(1)='1' then
                         gif_r(1) <= '1';
                      end if;
                 when others => -- "00"
                      -- By level, this flag is always 0
                      gif_r(1) <= '0';
            end case;
         end if; -- else rst_i='1'
      end if; -- rising_edge(clk_i)
   end process do_regs;

   ext_irq_o(0) <= pin_irq_i(0)                   when ENABLE='0'  else
                   not(pin_irq_i(0)) and gic_r(0) when isc0_r="00" else
                   gif_r(0)          and gic_r(0);
   ext_irq_o(1) <= pin_irq_i(1)                   when ENABLE='0'  else
                   not(pin_irq_i(1)) and gic_r(1) when isc1_r="00" else
                   gif_r(1)          and gic_r(1);
   ext_irq_o(4 downto 2) <= dev_irq_i and gic_r(4 downto 2) when ENABLE='1' else
                            dev_irq_i;

   data_o <= (gic_r&"000"           and gicr_sel) or
             (dev_irq_i&gif_r&"000" and gifr_sel) or
             ("0000"&mcuc_r         and mcucr_sel);
end architecture RTL; -- Entity: IRQCtrl

