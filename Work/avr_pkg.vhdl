------------------------------------------------------------------------------
----                                                                      ----
----  AVR CPU core constants, types and prototypes                        ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  Package for the CPU core                                            ----
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
---- Design unit:      Types (Package)                                    ----
----                   Constants (Package)                                ----
----                   Internal (Package)                                 ----
----                   Core (Package)                                     ----
---- File name:        avr_pkg.vhdl                                       ----
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

package Types is
   type pport_data_t is
   record
      data    : unsigned(5 downto 0); -- Data to output address
      ddr     : unsigned(5 downto 0); -- Data Direction address
      inp     : unsigned(5 downto 0); -- Data from input address
   end record;

   constant LD_X : std_logic_vector(1 downto 0):="11";
   constant LD_Y : std_logic_vector(1 downto 0):="10";
   constant LD_Z : std_logic_vector(1 downto 0):="00";
   constant LD_POST_INC : std_logic_vector(1 downto 0):="01";
   constant LD_PRE_DEC  : std_logic_vector(1 downto 0):="10";

   -- Instruction Decoder
   type idc_t is
   record
      add     : std_logic; -- ADD/ADC
      add_wc_r: std_logic; -- With Carry
      xxiw    : std_logic; -- ADIW/SBIW
      is_sub_r: std_logic; -- '1' for SBIW, '0' for ADIW
      ando    : std_logic; -- AND
      andi    : std_logic; -- ANDI
      asr     : std_logic; -- ASR
      bc_bs   : std_logic; -- BCLR/BSET
      bclr    : std_logic; -- Clear
      bld     : std_logic; -- BLD
      brbx    : std_logic; -- BRBC/BRBS
      brbc    : std_logic; -- Is Clear
      bst     : std_logic; -- BST
      sbix    : std_logic; -- SBIC/SBIS
      sbrx    : std_logic; -- SBRC/SBRS
      xbi     : std_logic; -- CBI/SBI
      set_r   : std_logic; -- Set (CBI/SBI/SBIC/SBIS/SBRC/SBRS)
      com     : std_logic; -- COM
      cp      : std_logic; -- CP
      cpc     : std_logic; -- CPC
      cpi     : std_logic; -- CPI
      cpse    : std_logic; -- CPSE
      dec     : std_logic; -- DEC
      lpm     : std_logic; -- ELPM/LPM
      elpm_r  : std_logic; -- Extended LPM
      spm     : std_logic; -- SPM
      eor     : std_logic; -- EOR
      icall   : std_logic; -- ICALL
      ijmp    : std_logic; -- IJMP
      ino     : std_logic; -- IN
      inc     : std_logic; -- INC
      ld      : std_logic; -- LD Rd,REG REG=X/X+/-X/Y+/-Y/Z+/-Z
      ld_reg  : std_logic_vector(1 downto 0); -- 11=X 10=Y 00=Z
      ld_op_r : std_logic_vector(1 downto 0); -- 00 nop 01 X+ 10 -X
      ldd     : std_logic; -- LDD Rd,Y+q/Z+q
      ldd_y   : std_logic; -- Y variant (same for STD)
      q_r     : unsigned(5 downto 0); -- registered q offset
      ldi     : std_logic; -- LDI
      lds     : std_logic; -- LDS
      lsr     : std_logic; -- LSR
      mov     : std_logic; -- MOV
      neg     : std_logic; -- NEG
      oro     : std_logic; -- OR
      ori     : std_logic; -- ORI
      outo    : std_logic; -- OUT
      pop     : std_logic; -- POP
      push    : std_logic; -- PUSH
      rcall   : std_logic; -- RCALL
      ret     : std_logic; -- RET
      reti_r  : std_logic; -- RETI variant
      rjmp    : std_logic; -- RJMP
      roro    : std_logic; -- ROR
      sbc     : std_logic; -- SBC
      shf_op_r: std_logic_vector(1 downto 0); -- ASR,ASR,LSR,ROR
      sleep   : std_logic; -- SLEEP
      st      : std_logic; -- ST REG,Rr REG=X/X+/-X/Y+/-Y/Z+/-Z
      stdo    : std_logic; -- STD Y+q/Z+q,Rr
      sts     : std_logic; -- STS
      subo    : std_logic; -- SUB
      subic   : std_logic; -- SUBI/SBCI
      sub_nc  : std_logic; -- No Carry (SBC/SBCI/CPC)
      swap    : std_logic; -- SWAP
      wdr     : std_logic; -- WDR
      -- Post/Pre-Inc/Dec for LD/ST
      psinc   :  std_logic; -- Post increment
      prdec   :  std_logic; -- Pre decrement
      -- AVR3 instructions
      call    : std_logic; -- CALL
      jmp     : std_logic; -- JMP
      -- AVR4 instructions
      mul     : std_logic; -- MUL   (unsigned*unsigned) [8*8=16]
      muls    : std_logic; -- MULS  (signed*signed)
      mulsu   : std_logic; -- MULSU (signed*unsigned)
      fmul    : std_logic; -- FMUL   (unsigned*unsigned) [1.7*1.7=2.14]
      fmuls   : std_logic; -- FMULS  (signed*signed)
      fmulsu  : std_logic; -- FMULSU (signed*unsigned)
      movw    : std_logic; -- MOVW
   end record;
end package Types;

--************************************************************************************************
-- Internal: Components used to implement the AVR core.
--************************************************************************************************

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
library avr;
use avr.Types.all;

package Internal is
   component RegisterFile is
      port(
         clk_i    : in  std_logic;
         -- Main 8 bits port
         rd_we_i  : in  std_logic;
         rd_adr_i : in  unsigned(4 downto 0);
         rd_i     : in  std_logic_vector( 7 downto 0);
         rd_o     : out std_logic_vector( 7 downto 0);
         -- Man 16 bits port
         rd16_we_i: in  std_logic:='0';
         rd16_i   : in  unsigned(15 downto 0):=x"0000";
         rd16_o   : out unsigned(15 downto 0);
         -- Secondary 8 bits read port
         rr_adr_i : in  unsigned(4 downto 0):="00000";
         rr_o    : out std_logic_vector(7 downto 0));
   end component RegisterFile;

   component IORegFile is
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
   end component IORegFile;
end package Internal;

--************************************************************************************************
-- Constants: Various related constants.
--************************************************************************************************

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
library avr;
use avr.Types.all;

package Constants is
   -- External multiplexer
   type ext_mux_t is array(natural range<>) of std_logic_vector(7 downto 0);
   
   -- I/O Register File (Internal)
   constant RAMPZ_ADDRESS  : unsigned(5 downto 0):=resize(x"3B",6);
   constant SPL_ADDRESS    : unsigned(5 downto 0):=resize(x"3D",6);
   constant SPH_ADDRESS    : unsigned(5 downto 0):=resize(x"3E",6);
   constant SREG_ADDRESS   : unsigned(5 downto 0):=resize(x"3F",6);
   -- Interrupt Control
   -- ATmega8, ATtiny22
   constant GICR_ADDRESS   : unsigned(5 downto 0):=resize(x"3B",6);
   constant GIFR_ADDRESS   : unsigned(5 downto 0):=resize(x"3A",6);
   -- ATmega103
   constant EIMSK_ADDRESS  : unsigned(5 downto 0):=resize(x"39",6);
   constant EIFR_ADDRESS   : unsigned(5 downto 0):=resize(x"38",6);
   constant EICR_ADDRESS   : unsigned(5 downto 0):=resize(x"3A",6);
   -- UART
   constant UDR_ADDRESS    : unsigned(5 downto 0):=resize(x"0C",6);
   constant UBRR_ADDRESS   : unsigned(5 downto 0):=resize(x"09",6);
   constant USR_ADDRESS    : unsigned(5 downto 0):=resize(x"0B",6);
   constant UCR_ADDRESS    : unsigned(5 downto 0):=resize(x"0A",6);
   -- Timer/Counter
   constant TCCR0_ADDRESS  : unsigned(5 downto 0):=resize(x"33",6);
   constant TCCR1A_ADDRESS : unsigned(5 downto 0):=resize(x"2F",6);
   constant TCCR1B_ADDRESS : unsigned(5 downto 0):=resize(x"2E",6);
   constant TCCR2_ADDRESS  : unsigned(5 downto 0):=resize(x"25",6);
   constant ASSR_ADDRESS   : unsigned(5 downto 0):=resize(x"30",6);
   constant TIMSK_ADDRESS  : unsigned(5 downto 0):=resize(x"37",6);
   constant TIFR_ADDRESS   : unsigned(5 downto 0):=resize(x"36",6);
   constant TCNT0_ADDRESS  : unsigned(5 downto 0):=resize(x"32",6);
   constant TCNT2_ADDRESS  : unsigned(5 downto 0):=resize(x"24",6);
   constant OCR0_ADDRESS   : unsigned(5 downto 0):=resize(x"31",6);
   constant OCR2_ADDRESS   : unsigned(5 downto 0):=resize(x"23",6);
   constant TCNT1H_ADDRESS : unsigned(5 downto 0):=resize(x"2D",6);
   constant TCNT1L_ADDRESS : unsigned(5 downto 0):=resize(x"2C",6);
   constant OCR1AH_ADDRESS : unsigned(5 downto 0):=resize(x"2B",6);
   constant OCR1AL_ADDRESS : unsigned(5 downto 0):=resize(x"2A",6);
   constant OCR1BH_ADDRESS : unsigned(5 downto 0):=resize(x"29",6);
   constant OCR1BL_ADDRESS : unsigned(5 downto 0):=resize(x"28",6);
   constant ICR1AH_ADDRESS : unsigned(5 downto 0):=resize(x"27",6);
   constant ICR1AL_ADDRESS : unsigned(5 downto 0):=resize(x"26",6);
   -- Service module
   constant MCUCR_ADDRESS  : unsigned(5 downto 0):=resize(x"35",6);
   constant MCUSR_ADDRESS  : unsigned(5 downto 0):=resize(x"34",6);
   constant XDIV_ADDRESS   : unsigned(5 downto 0):=resize(x"3C",6);
   -- EEPROM 
   constant EEARH_ADDRESS  : unsigned(5 downto 0):=resize(x"1F",6);
   constant EEARL_ADDRESS  : unsigned(5 downto 0):=resize(x"1E",6);
   constant EEDR_ADDRESS   : unsigned(5 downto 0):=resize(x"1D",6);
   constant EECR_ADDRESS   : unsigned(5 downto 0):=resize(x"1C",6);
   -- WISHBONE extension, added by SET.
   -- Uses the EEPROM space.
   constant WB_ADDRESS     : unsigned(5 downto 0):=resize(x"1F",6);
   constant WB_DATA        : unsigned(5 downto 0):=resize(x"1E",6);
   -- WISHBONE extension, added by David Caruso.
   -- Uses the ADC space, internal wishbone for internal peripherals
   constant WB_DATA_INT    : unsigned(5 downto 0):=resize(x"05",6);
   -- SPI
   constant SPCR_ADDRESS   : unsigned(5 downto 0):=resize(x"0D",6);
   constant SPSR_ADDRESS   : unsigned(5 downto 0):=resize(x"0E",6);
   constant SPDR_ADDRESS   : unsigned(5 downto 0):=resize(x"0F",6);
   -- PORTA
   constant PORTA_ADDRESS  : unsigned(5 downto 0):=resize(x"1B",6);
   constant DDRA_ADDRESS   : unsigned(5 downto 0):=resize(x"1A",6);
   constant PINA_ADDRESS   : unsigned(5 downto 0):=resize(x"19",6);
   -- PORTB
   constant PORTB_ADDRESS  : unsigned(5 downto 0):=resize(x"18",6);
   constant DDRB_ADDRESS   : unsigned(5 downto 0):=resize(x"17",6);
   constant PINB_ADDRESS   : unsigned(5 downto 0):=resize(x"16",6);
   -- PORTC
   constant PORTC_ADDRESS  : unsigned(5 downto 0):=resize(x"15",6);
   constant DDRC_ADDRESS   : unsigned(5 downto 0):=resize(x"14",6);
   constant PINC_ADDRESS   : unsigned(5 downto 0):=resize(x"13",6);
   -- PORTD
   constant PORTD_ADDRESS  : unsigned(5 downto 0):=resize(x"12",6);
   constant DDRD_ADDRESS   : unsigned(5 downto 0):=resize(x"11",6);
   constant PIND_ADDRESS   : unsigned(5 downto 0):=resize(x"10",6);
   -- PORTE
   constant PORTE_ADDRESS  : unsigned(5 downto 0):=resize(x"03",6);
   constant DDRE_ADDRESS   : unsigned(5 downto 0):=resize(x"02",6);
   constant PINE_ADDRESS   : unsigned(5 downto 0):=resize(x"01",6);
   -- PORTF (Input only)
   constant PINF_ADDRESS   : unsigned(5 downto 0):=resize(x"00",6);
   
   -- ******************** Parallel port address table ************************
   constant MAX_PPORT : positive:=6;
   
   type pport_table_t is array (0 to MAX_PPORT-1) of pport_data_t;
   
   constant PPORT_TABLE : pport_table_t:=
      ((PORTA_ADDRESS,DDRA_ADDRESS,PINA_ADDRESS),  -- PORTA
       (PORTB_ADDRESS,DDRB_ADDRESS,PINB_ADDRESS),  -- PORTB
       (PORTC_ADDRESS,DDRC_ADDRESS,PINC_ADDRESS),  -- PORTC
       (PORTD_ADDRESS,DDRD_ADDRESS,PIND_ADDRESS),  -- PORTD
       (PORTE_ADDRESS,DDRE_ADDRESS,PINE_ADDRESS),  -- PORTE
       (     "000000",    "000000",PINF_ADDRESS)); -- PORTF
   -- *************************************************************************

   -- Analog to digital converter
   constant ADCL_ADDRESS   : unsigned(5 downto 0):=resize(x"04",6);
   constant ADCH_ADDRESS   : unsigned(5 downto 0):=resize(x"05",6);
   constant ADCSR_ADDRESS  : unsigned(5 downto 0):=resize(x"06",6);
   constant ADMUX_ADDRESS  : unsigned(5 downto 0):=resize(x"07",6);
   -- Analog comparator
   constant ACSR_ADDRESS   : unsigned(5 downto 0):=resize(x"08",6);
   -- Watchdog
   constant WDTCR_ADDRESS  : unsigned(5 downto 0):=resize(x"21",6);

   -- ***************************************************************************************
   
   -- Function declaration
   function "and"(l: std_logic_vector; r: std_logic) return std_logic_vector;
   function "and"(l: unsigned; r: std_logic) return unsigned;
end package Constants;

package body Constants is
   function "and"(l: std_logic_vector; r: std_logic)
      return std_logic_vector is
      variable aux : std_logic_vector(l'range):=(others => '0');
   begin
      if r='1' then
         return l;
      end if;
      return aux;
   end "and";

   function "and"(l: unsigned; r: std_logic)
      return unsigned is
      variable aux : unsigned(l'range):=(others => '0');
   begin
      if r='1' then
         return l;
      end if;
      return aux;
   end "and";
   -- End of functions
end package body Constants;

--************************************************************************************************
-- Core: CPU Core
--************************************************************************************************

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
library avr;
use avr.Types.all;
use avr.Constants.all;

package Core is
   component AVRCore is
      generic(
         SP_W       : integer range 8 to 16:=16; -- Stack pointer size
         RAM_ADR_W  : integer range 8 to 16:=16; -- RAM address width
         ENA_RAMPZ  : std_logic:='0'; -- RAMPZ enable
         ID_W       : positive:=5;    -- Width of the IRQ ID
         ENA_AVR25  : std_logic:='0'; -- Enable AVR25 instructions (MOVW/LPM Rd,Z)
         ENA_AVR3   : std_logic:='1'; -- Enable AVR3 instructions
         ENA_AVR4   : std_logic:='0'; -- Enable AVR4 instructions
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
   end component AVRCore;
end package Core;

