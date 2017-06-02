------------------------------------------------------------------------------
----                                                                      ----
----  AVR CPU                                                             ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  AVR 2.0 and 2.5 CPU implementation.                                 ----
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
---- Design unit:      CPU(RTL) (Entity and architecture)                 ----
---- File name:        cpu.vhdl                                           ----
---- Note:             None                                               ----
---- Limitations:      None known                                         ----
---- Errors:           None known                                         ----
---- Library:          avr                                                ----
---- Dependencies:     IEEE.std_logic_1164                                ----
----                   IEEE.numeric_std                                   ----
----                   avr.Constants                                      ----
----                   avr.Types                                          ----
---- Target FPGA:      iCE40HX4K-TQ144                                    ----
---- Language:         VHDL                                               ----
---- Wishbone:         None                                               ----
---- Synthesis tools:  Lattice iCECube2 2016.02.27810                     ----
---- Simulation tools: GHDL [Sokcho edition] (0.2x)                       ----
---- Text editor:      SETEdit 0.5.x                                      ----
----                                                                      ----
------------------------------------------------------------------------------
-- TODO:
-- Posibles optimizaciones.
-- * Si en un ciclo tiene que leer un registro Rx y ese registro ya estaba direccionado en la
--   instrucción anterior ... es posible saltear la lectura, ya se hizo.
-- * Hay un tema con las instrucciones de lectura de I/O. En principio las pensé sincrónicas, e
--   pero en realidad son asincrónicas. Así hay varias instrucciones que se acotarían en 1 ciclo
--   Lo que hice fue sincronizarlas con io_data_r, en la esperanza que eso les de más tiempo para
--   decodificar el espacio de I/O, pero quizás no valga la pena. Evaluar.
--

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

library avr;
use avr.Constants.all;
use avr.Types.all;

entity CPU is
   generic(
      IRQ_ID_W   : positive:=2;     -- Width of the ID
      ENA_AVR25  : std_logic:='0';  -- Enable AVR25 instructions (MOVW/LPM Rd,Z)
      ENA_AVR3   : boolean:=false;  -- Enable AVR3 instructions
      ENA_AVR4   : boolean:=false;  -- Enable AVR4 instructions
      ENA_SPM    : std_logic:='0';  -- AVR4 Store Program Memory
      SP_W       : integer range 8 to 16:=8; -- SP register width
      RAM_ADR_W  : integer range 8 to 16:=8; -- RAM address width
      RESET_JUMP : natural:=0;      -- Address of the reset vector
      ENA_DEBUG  : std_logic:='0';  -- Enable debug interface
      IRQ_LINES  : positive:=2);    -- Number of IRQ lines
   port(
      -- Clock and reset
      clk_i           : in  std_logic;
      ena_i           : in  std_logic;
      rst_i           : in  std_logic;
      -- Program memory
      pc_o            : out unsigned(15 downto 0);
      inst_i          : in  std_logic_vector(15 downto 0);
      inst_o          : out std_logic_vector(15 downto 0);
      pgm_we_o        : out std_logic;
      -- I/O control
      io_adr_o        : out unsigned(5 downto 0);
      io_re_o         : out std_logic;
      io_we_o         : out std_logic;
      io_data_i       : in  std_logic_vector(7 downto 0):=x"00";
      -- Data memory control
      ram_adr_o       : out std_logic_vector(RAM_ADR_W-1 downto 0);
      ram_re_o        : out std_logic;
      ram_we_o        : out std_logic;
      ram_data_i      : in  std_logic_vector(7 downto 0):=x"00";
      -- Shared between I/O and RAM
      data_o          : out std_logic_vector(7 downto 0);
      -- I/O register file interface
      sreg_o          : out std_logic_vector(7 downto 0);
      sreg_i          : in  std_logic_vector(7 downto 0); -- Current SREG
      sreg_we_o       : out std_logic_vector(7 downto 0); -- Flags write enable
      sp_i            : in  unsigned(SP_W-1 downto 0); -- Stack Pointer
      sp_pop_o        : out std_logic; -- Direction of changing of stack pointer 0->up(+) 1->down(-)
      sp_we_o         : out std_logic; -- Write enable(count enable) for SP
      rampz_i         : in  std_logic_vector(7 downto 0);
      -- Interrupt
      irq_lines_i     : in  std_logic_vector(IRQ_LINES-1 downto 0);
      irq_acks_o      : out std_logic_vector(IRQ_LINES-1 downto 0);
      -- Sleep
      sleep_o         : out std_logic;
      irq_ok_o        : out std_logic;
      --Watchdog
      wdr_o           : out std_logic;
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
end entity CPU;

architecture RTL of CPU is
   ---------------------
   -- Program counter --
   ---------------------
   -- Registers
   signal pc_l_r          : unsigned(7 downto 0);--:=(to_unsigned(RESET_JUMP,16))(7 downto 0);
   signal pc_h_r          : unsigned(7 downto 0);--:=(to_unsigned(RESET_JUMP,16))(15 downto 8);
   -- The whole PC
   signal pc              : unsigned(15 downto 0);
   -- Future value
   signal pc_next         : unsigned(15 downto 0);
   signal pc_l_next       : unsigned(7 downto 0);
   signal pc_h_next       : unsigned(7 downto 0);
   -- Sources
   signal do_pc_inc       : std_logic;
   signal do_pc_imm1      : std_logic;
   signal do_pc_imm2      : std_logic;
   signal do_pc_z         : std_logic;
   signal do_pc_irq       : std_logic;
   signal do_pc_pl        : std_logic;
   signal do_pc_ph        : std_logic;
   signal do_pc_def       : std_logic;
   -- Instruction Register
   signal inst_r          : std_logic_vector(15 downto 0);
   signal inst_cur        : std_logic_vector(15 downto 0); -- Current instruction under execution
   signal inst_stop       : std_logic_vector(15 downto 0); -- inst_i at the moment we stopped
   signal inst_stop_r     : std_logic_vector(15 downto 0); -- Register to remmember inst_i when we stop
   signal inst            : std_logic_vector(15 downto 0); -- Used to disable the decoder when we have an interrupt
   signal ir_we           : std_logic;
   -- Instruction Decoder
   signal idc             : idc_t;
   -- Multicycle FSM
   signal cyc_1_r         : std_logic;
   signal cyc_1_next      : std_logic;
   signal cyc_fetch_r     : std_logic; -- PC changed, wait for a fetch
   signal cyc_fetch_next  : std_logic; -- PC changed, wait for a fetch
   alias  cyc_2_rjmp_r    : std_logic is cyc_fetch_r; -- RJMP k (Fetch next instruction)
   signal cyc_2_rjmp_next : std_logic; -- RJMP
   signal cyc_2_rcall_r   : std_logic; -- RCALL k (PUSH PC High)
   signal cyc_2_rcall_next: std_logic; -- RCALL
   alias  cyc_3_xcall_r   : std_logic is cyc_fetch_r; -- RCALL/ICALL/IRQ (Fetch next instruction)
   signal cyc_2_icall_r   : std_logic; -- ICALL (PUSH PC High)
   signal cyc_2_icall_next: std_logic; -- ICALL
   signal cyc_2_ijmp_r    : std_logic; -- IJMP (Write PC)
   signal cyc_2_ijmp_next : std_logic; -- IJMP
   alias  cyc_3_ijmp_r    : std_logic is cyc_fetch_r; -- IJMP (Fetch next instruction)
   signal cyc_1_brbx      : std_logic; -- BRBC/BRBS k
   alias  cyc_2_brbx_r    : std_logic is cyc_fetch_r; -- BRBC/BRBS k (2 Fetch next instruction)
   signal cyc_2_brbx_next : std_logic; -- BRBC/BRBS k
   signal cyc_2_ret_r     : std_logic; -- RET/RETI (2 Read PCL)
   signal cyc_2_ret_next  : std_logic; -- RET/RETI
   signal cyc_3_ret_r     : std_logic; -- RET/RETI (3 Read PCH)
   signal cyc_3_ret_next  : std_logic; -- RET/RETI
   signal cyc_4_ret_r     : std_logic; -- RET/RETI (4 Fetch)
   signal cyc_4_ret_next  : std_logic; -- RET/RETI
   signal cyc_2_out_r     : std_logic; -- OUT A,Rr (Write to I/O space from mem)
   signal cyc_2_out_next  : std_logic; -- OUT A,Rr
   signal cyc_2_in_r      : std_logic; -- IN Rd,A (Write to Rd from I/O space from mem) TODO: From ALU?
   signal cyc_2_in_next   : std_logic; -- IN Rd,A
   signal cyc_2_wioa_r    : std_logic; -- SBI/CBI A,b (Write to I/O space from ALU)
   signal cyc_2_wioa_next : std_logic; -- SBI/CBI A,b
   signal cyc_2_sbix_r    : std_logic; -- SBIS/SBIC A,b (2 Check condition)
   signal cyc_2_sbix_next : std_logic; -- SBIS/SBIC A,b
   signal cyc_2_cpse_r    : std_logic; -- CPSE Rd,Rr (2 Check condition)
   signal cyc_2_cpse_next : std_logic; -- CPSE Rd,Rr
   signal cyc_2_sbrx_r    : std_logic; -- SBRC/SBRS Rr,b (2 Check condition)
   signal cyc_2_sbrx_next : std_logic; -- SBRC/SBRS Rr,b
   signal cyc_3_skip_r    : std_logic; -- SBIS/SBIC A,b/CPSE (3 Skip 1 word)
   signal cyc_3_skip_next : std_logic; -- SBIS/SBIC A,b/CPSE
   alias  cyc_4_skip_r    : std_logic is cyc_fetch_r; -- SBIS/SBIC A,b/CPSE (4 Skip 2 words, fetch)
   signal cyc_4_skip_next : std_logic; -- SBIS/SBIC A,b/CPSE
   signal cyc_2_sts_r     : std_logic; -- STS k,Rr (Write to full address space)
   signal cyc_2_sts_next  : std_logic; -- STS k,Rr
   signal cyc_2_st_r      : std_logic; -- ST X,Rr (Write to full address space)
   signal cyc_2_st_next   : std_logic; -- ST X,Rr
   signal cyc_3_st_r      : std_logic; -- ST -X/X+,Rr (Update pointer) Only when inc/dec
   signal cyc_3_st_next   : std_logic; -- ST -X/X+,Rr
   signal cyc_2_std_r     : std_logic; -- STD Y+q/Z+q,Rr (Write to full address space+q)
   signal cyc_2_std_next  : std_logic; -- STD Y+q/Z+q,Rr
   signal cyc_2_push_r    : std_logic; -- PUSH Rr (Write to full address space)
   signal cyc_2_push_next : std_logic; -- PUSH Rr
   signal cyc_2_ld_r      : std_logic; -- LD Rd,X/Y/Z (Read from full address space)
   signal cyc_2_ld_next   : std_logic; -- LD Rd,X/Y/Z
   signal cyc_3_ld_r      : std_logic; -- LD Rd,X/Y/Z (Write to Rd from memory)
   signal cyc_3_ld_next   : std_logic; -- LD Rd,X/Y/Z
   signal cyc_2_ldd_r     : std_logic; -- LDD Rd,Y+q/Z+q (Read from full address space+q)
   signal cyc_2_ldd_next  : std_logic; -- LDD Rd,Y+q/Z+q
   signal cyc_2_lds_r     : std_logic; -- LDS Rd,k (Read from full address space)
   signal cyc_2_lds_next  : std_logic; -- LDS Rd,k
   signal cyc_2_xxiw_r    : std_logic; -- ADIW/SBIW (2 Add+Write Rd)
   signal cyc_2_xxiw_next : std_logic; -- ADIW/SBIW (2 Add+Write Rd)
   signal cyc_3_xxiw_r    : std_logic; -- ADIW/SBIW (3 Read Rd+1)
   signal cyc_3_xxiw_next : std_logic; -- ADIW/SBIW (3 Read Rd+1)
   signal cyc_4_xxiw_r    : std_logic; -- ADIW/SBIW (4 Write Rd+1)
   signal cyc_4_xxiw_next : std_logic; -- ADIW/SBIW (4 Write Rd+1)
   signal cyc_2_add_r     : std_logic; -- ADD/ADC
   signal cyc_2_sub_r     : std_logic; -- SUB/SBC/CP/CPC/SUBI/SBCI/CPI/NEG/DEC/CPSE
   signal cyc_2_sbc_r     : std_logic; -- SBC/CPC/SBCI
   signal cyc_2_and_r     : std_logic; -- AND
   signal cyc_2_or_r      : std_logic; -- OR
   signal cyc_2_eor_r     : std_logic; -- EOR
   signal cyc_2_com_r     : std_logic; -- COM
   signal cyc_2_neg_r     : std_logic; -- NEG
   signal cyc_2_inc_dec_r : std_logic; -- INC/DEC
   signal cyc_2_shf_r     : std_logic; -- ASR/LSR/ROR
   signal cyc_2_swap_r    : std_logic; -- SWAP
   signal cyc_2_mov_r     : std_logic; -- MOV
   signal cyc_2_movw_next : std_logic; -- MOVW
   signal cyc_2_movw_r    : std_logic; -- MOVW
   signal cyc_2_bld_clr_r : std_logic; -- BLD T=0
   signal cyc_2_bld_set_r : std_logic; -- BLD T=1
   signal cyc_2_bld_r     : std_logic; -- BLD T=x
   signal cyc_2_bld_next  : std_logic; -- BLD T=x
   signal cyc_2_walu_r    : std_logic; -- ALU Write (ADD/ADC/SUB/SBC/AND/OR/EOR)+(SUBI/SBCI/ANDI/ORI)+(LSR/ASR/ROR)+(COM/NEG/SWAP/MOV)+BLD
   signal cyc_2_walu_next : std_logic; -- ALU Write (ADD/ADC/SUB/SBC/AND/OR/EOR)+(SUBI/SBCI/ANDI/ORI)+(LSR/ASR/ROR)+(COM/NEG/SWAP/MOV)+BLD
   signal cyc_2_alu_r     : std_logic; -- ALU (CP/CPC/CPSE)+ all the ALU write stuff
   signal cyc_2_alu_next  : std_logic; -- ALU (CP/CPC/CPSE)+ all the ALU write stuff
   signal cyc_2_ialu_r    : std_logic; -- ALU Immediate (SUBI/SBCI/ANDI/ORI)
   signal cyc_2_ialu_next : std_logic; -- ALU Immediate (SUBI/SBCI/ANDI/ORI)
   signal cyc_2_bst_r     : std_logic; -- BST (2 write SREG)
   signal cyc_2_bst_next  : std_logic; -- BST (2 write SREG)
   signal cyc_2_lpm_r     : std_logic; -- LPM (2 read PROM)
   signal cyc_2_lpm_next  : std_logic; -- LPM (2 read PROM)
   signal cyc_3_lpm_r     : std_logic; -- LPM (3 write R0)
   signal cyc_3_lpm_next  : std_logic; -- LPM (3 write R0)
   signal cyc_2_spm_r     : std_logic; -- SPM (2 read Z)
   signal cyc_2_spm_next  : std_logic; -- SPM (2 read Z)
   signal cyc_3_spm_r     : std_logic; -- SPM (3 write PROM)
   signal cyc_3_spm_next  : std_logic; -- SPM (3 write PROM)
   signal cyc_1_bc_bs     : std_logic; -- BCLR & BSET (1 Read/Modify/Write)
   signal cyc_1_bclr      : std_logic; -- BCLR (1 Read/Modify/Write)
   signal cyc_1_bset      : std_logic; -- BCLR (1 Read/Modify/Write)
   signal cyc_2_pop_next  : std_logic; -- POP Rd (1 Read [SP+1])
   signal cyc_1_ldi       : std_logic; -- LDI Rd,k
   -- Rd address
   signal do_adr_x        : std_logic; -- Address X reg
   signal do_adr_y        : std_logic; -- Address Y reg
   signal do_adr_z        : std_logic; -- Address Z reg
   signal do_adr_rhi      : std_logic; -- Address r16 to r31
   signal do_adr_mem      : std_logic; -- Address from memory map
   signal do_adr_iw0      : std_logic; -- Address low xxIW reg
   signal do_adr_iw1      : std_logic; -- Address high xxIW reg
   signal do_adr_r0       : std_logic; -- Address R0
   signal do_adr_rr0      : std_logic; -- Address Rr&'0'
   signal do_adr_rd0      : std_logic; -- Address Rd&'0'
   signal do_adr_rd       : std_logic; -- Address Rd
   -- Rr address
   signal do_adr2_mem     : std_logic; -- Address from memory map
   signal do_adr2_rd      : std_logic; -- Address Rd
   signal do_adr2_rr      : std_logic; -- Address Rr
   -- Rd data write source
   signal rd_dat_alu      : std_logic; -- From ALU
   signal rd_dat_imm      : std_logic; -- Immediate
   signal rd_dat_pr0      : std_logic; -- From ROM, even address
   signal rd_dat_pr1      : std_logic; -- From ROM, odd address
   -- Memory address source
   signal fadr_imm        : std_logic; -- Immediate
   signal fadr_p_q        : std_logic; -- X/Y/Z Pointer + q(imm)
   signal fadr_p          : std_logic; -- X/Y/Z Pointer (optionally -1)
   -- I/O address sources
   signal io_adr_imm1     : std_logic; -- Immediate 1 (IN/OUT)
   signal io_adr_imm2     : std_logic; -- Immediate Bit manipulation cyc 1
   signal io_adr_b2       : std_logic; -- Immediate Bit manipulation cyc 2
   signal io_adr_mem      : std_logic; -- From RAM
   -- Data outputs sources
   signal data_from_alu   : std_logic; -- From ALU
   signal data_from_pcl   : std_logic; -- From PC Low
   signal data_from_pch   : std_logic; -- From PC High
   signal data_from_rr    : std_logic; -- From Rr
   -- Some actions
   signal do_push         : std_logic; -- Use SP as RAM adr, store a value, SP--
   signal do_pop          : std_logic; -- Use SP+1 as RAM adr, load a value, SP++
   signal do_store        : std_logic; -- Store a value in memory (ST/STD/STS)
   signal do_load         : std_logic; -- Load a value from memory (LD/LDD/LDS)
   signal do_rd_pointer1  : std_logic; -- Read X/Y/Z LD/ST
   signal do_rd_pointer2  : std_logic; -- Read X/Y/Z LDD/STD
   signal do_pre_dec      : boolean;   -- Pre-decrement pointer
   signal do_post_inc     : boolean;   -- Post-increment pointer
   signal do_upd_pointer  : std_logic; -- Pre-decrement/post-increment pointer
   -- Some equivalences
   -- Write Rd from memory
   alias  cyc_3_lds_r     : std_logic is cyc_3_ld_r;
   alias  cyc_3_lds_next  : std_logic is cyc_3_ld_next;
   alias  cyc_3_ldd_r     : std_logic is cyc_3_ld_r;
   alias  cyc_3_ldd_next  : std_logic is cyc_3_ld_next;
   alias  cyc_2_pop_r     : std_logic is cyc_3_ld_r;
   -- Do a fetch (we changed PC in previous cycle)
   alias  cyc_4_lpm_r     : std_logic is cyc_fetch_r;
   -- Registers file
   -- Main 8 bits port R/W (Rd)
   signal rd_adr          : unsigned(4 downto 0);
   signal rd_we           : std_logic;
   signal rd_write        : std_logic_vector(7 downto 0);
   signal rd_read         : std_logic_vector(7 downto 0);
   signal rd_read_aux     : std_logic_vector(7 downto 0);
   -- Main 16 bits port R/W (Rd)
   signal rd16_we         : std_logic;
   signal rd16_write      : unsigned(15 downto 0);
   signal rd16_read       : unsigned(15 downto 0);
   -- Secondary 8 bits port R (Rr)
   signal rr_adr          : unsigned(4 downto 0);
   signal rr_read         : std_logic_vector(7 downto 0);
   signal rr_read_aux     : std_logic_vector(7 downto 0);
   -- Store address range
   signal full_adr        : unsigned(15 downto 0);
   signal full_adr_is_reg : std_logic;
   signal full_adr_is_ram : std_logic;
   signal full_adr_is_io  : std_logic;
   signal full_adr_is_reg_r : std_logic;
   signal full_adr_is_ram_r : std_logic;
   signal full_adr_is_io_r  : std_logic;
   -- Pointer offset (-X; X+q; X)
   signal fadr_p_off1       : unsigned(15 downto 0);
   signal fadr_p_off        : unsigned(15 downto 0);
   -- Stack pointer offset
   signal fadr_sp_off       : unsigned(15 downto 0);
   signal sp16              : unsigned(15 downto 0);
   --alias  xyz_ram_pointer   : unsigned(RAM_ADR_W-1 downto 0) is rd16_read(RAM_ADR_W-1 downto 0);
   -- SREG  I T H S V N Z C
   alias  sreg_i_r    : std_logic is sreg_i(7); -- Interrupt
   signal sreg_if     : std_logic;
   signal sreg_i_next : std_logic; -- I flag in the next clock cycle
   alias  sreg_t_r    : std_logic is sreg_i(6); -- Test
   signal sreg_t      : std_logic;
   alias  sreg_h_r    : std_logic is sreg_i(5); -- Half Carry
   alias  sreg_s_r    : std_logic is sreg_i(4); -- Sign
   signal sreg_s      : std_logic;
   alias  sreg_v_r    : std_logic is sreg_i(3); -- oVerflow
   signal sreg_v      : std_logic;
   alias  sreg_n_r    : std_logic is sreg_i(2); -- Negative
   signal sreg_n      : std_logic;
   alias  sreg_z_r    : std_logic is sreg_i(1); -- Zero
   signal sreg_z      : std_logic;
   alias  sreg_c_r    : std_logic is sreg_i(0); -- Carry
   signal sreg_c      : std_logic;
   signal sreg_out    : std_logic_vector(7 downto 0); -- sreg_o read-out
   signal sreg_we_out : std_logic_vector(7 downto 0); -- sreg_we_o read-out
   signal sreg_we_out1: std_logic_vector(7 downto 0); -- sreg_we_o read-out
   signal do_add_sub_flag : std_logic;
   signal do_shift_flag   : std_logic;
   signal do_arith_flag   : std_logic;
   signal do_alu_flag     : std_logic;
   -------------------------------------
   -- ALU
   signal cin     : unsigned(0 downto 0); -- Adder/Sub Carry In
   -- Adder
   signal add_lo  : unsigned(4 downto 0); -- Adder Low Nibble
   signal add_hi  : unsigned(4 downto 0); -- Adder High Nibble
   -- Substractor
   signal sub_lo  : unsigned(4 downto 0); -- Adder Low Nibble 
   signal sub_hi  : unsigned(4 downto 0); -- Adder High Nibble
   signal bit_mask : unsigned(7 downto 0);
   -- ALU Operands
   signal alu_a_1  : unsigned(7 downto 0);
   signal alu_a    : unsigned(7 downto 0);
   signal alu_b    : unsigned(7 downto 0);
   -- ALU Input flags
   signal alu_ci_1 : std_logic;
   signal alu_ci   : std_logic;
   -- ALU Operation (only one active at the time)
   signal op_add   : std_logic;
   signal op_sub   : std_logic;
   signal op_and   : std_logic;
   signal op_or    : std_logic;
   signal op_xor   : std_logic;
   signal op_nop   : std_logic;
   signal op_shf   : std_logic;
   signal op_swp   : std_logic;
   -- ALU Result
   signal alu_s    : unsigned(7 downto 0);
   -- ALU Flags
   signal alu_c    : std_logic;
   signal alu_h    : std_logic;
   signal alu_z    : std_logic;
   signal alu_v    : std_logic;
   -- ALU misc
   signal is_ror   : std_logic;
   signal is_asr   : std_logic;
   -- ALU A sources
   signal a_from_mask : std_logic; -- A is a bitmask
   signal a_from_rd   : std_logic; -- A is Rd
   -- ALU B sources
   signal b_from_io   : std_logic; -- From I/O
   signal b_from_imm2 : std_logic; -- Immediate (small)
   signal b_from_sreg : std_logic; -- From SREG
   signal b_from_imm1 : std_logic; -- Immediate (full)
   signal b_from_rr   : std_logic; -- From Rr
   signal b_from_0    : std_logic; -- Just 0x00
   signal b_from_mem  : std_logic; -- From SRAM
   -- I/O signals
   signal io_re    : std_logic;
   signal io_data_r: std_logic_vector(7 downto 0);
   signal io_map_a : unsigned(6 downto 0); -- Memory address translated to I/O space
   -- X,Y,Z memorized value for ST with pre-dec/post-inc
   signal cur_ptr  : unsigned(15 downto 0);
   signal rd16_r   : unsigned(15 downto 0);
   -- Interrupts
   signal irq_req         : std_logic; -- At least one source requesting
   signal irq_vector_adr  : unsigned(IRQ_ID_W-1 downto 0);
   signal was_irq_next    : std_logic:='0'; -- Last operation was interrupted
   signal cyc_1_irq_next  : std_logic; -- Next instruction will be interrupted
   signal cyc_1_irq_r     : std_logic; -- IRQ (1 Push PC Low)
   alias  cyc_2_irq_next  : std_logic is cyc_1_irq_r;
   signal cyc_2_irq_r     : std_logic; -- IRQ (2 Push PC High, ACK, Clear SREG(I), Load PC)
   alias  cyc_3_irq_r     : std_logic is cyc_fetch_r; -- IRQ (3 Fetch)
   -- ELPM's address bit 15 (reg_z_b16&Z)
   signal reg_z_b16 : std_logic;
   signal lpm_r0    : std_logic;
   signal lpm_inc   : std_logic;
   -- Debug Control
   signal enabled   : std_logic;
   signal stopped   : std_logic;
   signal was_ena_r : std_logic;
begin
   ----------------------------------------------------------------------------
   -- Program Counter (PC)
   ----------------------------------------------------------------------------
   -- Next PC sources
   -- The most common case (PC increment)
   do_pc_inc <= (cyc_1_next or                           -- We are finishing this instruction OR
                 cyc_2_sts_next  or cyc_2_lds_next or    -- 32 bits instruction
                 cyc_3_skip_next or cyc_4_skip_next) and -- We are skipping the next AND
               not(cyc_1_irq_next);                      -- Won't interrupt
   -- Immediate 12 bits relative (+/-2k) (RJMP/RCALL)
   do_pc_imm1 <= cyc_2_rjmp_next or cyc_2_rcall_r;
   -- Immediate 7 bits relative (+-63) (BRBC/BRBS)
   do_pc_imm2 <= cyc_2_brbx_next;
   -- Z pointer (IJMP/ICALL)
   do_pc_z    <= cyc_2_ijmp_r or cyc_2_icall_r;
   -- IRQ vector (IRQ)
   do_pc_irq  <= cyc_2_irq_r;
   -- POP low PC (RET/RETI)
   do_pc_pl   <= cyc_3_ret_r;
   -- POP high PC (RET/RETI)
   do_pc_ph   <= cyc_2_ret_r;
   -- Default case (no change)
   do_pc_def  <= not(do_pc_inc or do_pc_imm1 or do_pc_imm2 or do_pc_z or do_pc_irq or
                     do_pc_pl or do_pc_ph);
   -- Next PC mux
   pc_next <= (pc+1                                                  and do_pc_inc)  or
              (rd16_read                                             and do_pc_z)    or
              (resize(irq_vector_adr,16)                             and do_pc_irq)  or
              (pc_h_r&unsigned(ram_data_i)                           and do_pc_pl)   or
              (unsigned(ram_data_i)&pc_l_r                           and do_pc_ph)   or
              -- PC+K+1 (we already incremented 1)
              (pc+unsigned(resize(signed(inst_cur(11 downto 0)),16)) and do_pc_imm1) or
              (pc+unsigned(resize(signed(inst_cur(9 downto 3)),16))  and do_pc_imm2) or
              -- Default is to retain
              (pc_h_r&pc_l_r                                         and do_pc_def);
   -- PC registers
   pc_h_next <= pc_next(15 downto 8);
   pc_l_next <= pc_next( 7 downto 0);
   do_pc:
   process (clk_i)
   begin
      if rising_edge(clk_i) then
         if rst_i='1' then
            pc_l_r <= to_unsigned(RESET_JUMP,16)(7 downto 0);
            pc_h_r <= to_unsigned(RESET_JUMP,16)(15 downto 8);
         elsif enabled='1' then
            pc_l_r <= pc_l_next;
            pc_h_r <= pc_h_next;
         end if;
      end if;
   end process do_pc;
   -- 16 bits PC
   pc   <= pc_h_r&pc_l_r;
   -- Mux for LPM, we don't change PC, just use Z
   pc_o <= pc when (cyc_2_lpm_r or cyc_3_spm_r)='0' else reg_z_b16&rd16_read(15 downto 1); -- LPM
   -- rampz_i(0) for ELPM, 0 for LPM
   reg_z_b16 <= rampz_i(0) and idc.elpm_r when ENA_AVR3 else '0';
   -- SPM's inst_o
   inst_o_implemented:
   if ENA_SPM='1' generate
      do_inst_o:
      process (clk_i)
      begin
         if rising_edge(clk_i) then
            if rst_i='1' then
               inst_o <= (others => '0');
            elsif cyc_2_spm_r='1' then
               inst_o <= std_logic_vector(rd16_read);
            end if;
         end if;
      end process do_inst_o;
      pgm_we_o <= cyc_3_spm_r;
   end generate inst_o_implemented;

   inst_o_not_implemented:
   if ENA_SPM='0' generate
      inst_o <= (others => '0');
      pgm_we_o <= '0';
   end generate inst_o_not_implemented;

   ----------------------------------------------------------------------------
   -- Instruction Register
   ----------------------------------------------------------------------------
   ir_we <= enabled and cyc_1_r;
   do_ir:
   process (clk_i)
   begin
      if rising_edge(clk_i) then
         if rst_i='1' then
            inst_r <= (others => '0'); -- NOP
         elsif ir_we='1' then
            inst_r <= inst_stop;
         end if;
      end if;
   end process do_ir;
   inst_cur <= inst_stop when cyc_1_r='1' else inst_r;
   dbg_inst_o <= inst_cur;

   ----------------------------------------------------------------------------
   -- Instruction Decoder
   ----------------------------------------------------------------------------
   -- If we will serve an interrupt we must disable the decoder
   inst <= (others => '0') when cyc_1_irq_r='1' else inst_stop;
   InstDec : entity avr.Decoder
      generic map(
         ENA_AVR25 => ENA_AVR25, ENA_AVR3 => ENA_AVR3, ENA_AVR4 => ENA_AVR4)
      port map(
         inst_i => inst, inst_r_i => inst_r, idc_o => idc);

   ----------------------------------------------------------------------------
   -- Multicycle FSM
   ----------------------------------------------------------------------------
   do_fsm:
   process (clk_i)
   begin
      if rising_edge(clk_i) then
         if rst_i='1' then
            -- We start execution doing a fetch
            cyc_fetch_r     <= '1';
            -- First cycle for any instruction
            cyc_1_r         <= '0';
            cyc_2_ijmp_r    <= '0';
            cyc_2_rcall_r   <= '0';
            cyc_2_icall_r   <= '0';
            cyc_2_ret_r     <= '0';
            cyc_3_ret_r     <= '0';
            cyc_4_ret_r     <= '0';
            cyc_2_out_r     <= '0';
            cyc_2_in_r      <= '0';
            cyc_2_wioa_r    <= '0';
            cyc_2_sts_r     <= '0';
            cyc_2_st_r      <= '0';
            cyc_3_st_r      <= '0';
            cyc_2_std_r     <= '0';
            cyc_2_push_r    <= '0';
            cyc_2_ld_r      <= '0';
            cyc_3_ld_r      <= '0';
            cyc_2_lds_r     <= '0';
            cyc_2_ldd_r     <= '0';
            cyc_2_sbix_r    <= '0';
            cyc_2_cpse_r    <= '0';
            cyc_2_sbrx_r    <= '0';
            cyc_3_skip_r    <= '0';
            cyc_2_xxiw_r    <= '0';
            cyc_3_xxiw_r    <= '0';
            cyc_4_xxiw_r    <= '0';
            cyc_2_add_r     <= '0';
            cyc_2_sub_r     <= '0';
            cyc_2_sbc_r     <= '0';
            cyc_2_and_r     <= '0';
            cyc_2_or_r      <= '0';
            cyc_2_eor_r     <= '0';
            cyc_2_com_r     <= '0';
            cyc_2_neg_r     <= '0';
            cyc_2_inc_dec_r <= '0';
            cyc_2_shf_r     <= '0';
            cyc_2_swap_r    <= '0';
            cyc_2_mov_r     <= '0';
            cyc_2_movw_r    <= '0';
            cyc_2_walu_r    <= '0';
            cyc_2_alu_r     <= '0';
            cyc_2_ialu_r    <= '0';
            cyc_2_bst_r     <= '0';
            cyc_2_lpm_r     <= '0';
            cyc_3_lpm_r     <= '0';
            cyc_2_spm_r     <= '0';
            cyc_3_spm_r     <= '0';
            cyc_1_irq_r     <= '0';
            cyc_2_irq_r     <= '0';
         elsif enabled='1' then
            cyc_1_r         <= cyc_1_next;
            cyc_fetch_r     <= cyc_fetch_next;
            cyc_2_ijmp_r    <= cyc_2_ijmp_next;
            cyc_2_rcall_r   <= cyc_2_rcall_next;
            cyc_2_icall_r   <= cyc_2_icall_next;
            cyc_2_ret_r     <= cyc_2_ret_next;
            cyc_3_ret_r     <= cyc_3_ret_next;
            cyc_4_ret_r     <= cyc_4_ret_next;
            cyc_2_out_r     <= cyc_2_out_next;
            cyc_2_in_r      <= cyc_2_in_next;
            cyc_2_wioa_r    <= cyc_2_wioa_next;
            cyc_2_sts_r     <= cyc_2_sts_next;
            cyc_2_st_r      <= cyc_2_st_next;
            cyc_3_st_r      <= cyc_3_st_next;
            cyc_2_std_r     <= cyc_2_std_next;
            cyc_2_push_r    <= cyc_2_push_next;
            cyc_2_ld_r      <= cyc_2_ld_next;
            cyc_3_ld_r      <= cyc_3_ld_next;
            cyc_2_lds_r     <= cyc_2_lds_next;
            cyc_2_ldd_r     <= cyc_2_ldd_next;
            cyc_2_sbix_r    <= cyc_2_sbix_next;
            cyc_2_cpse_r    <= cyc_2_cpse_next;
            cyc_2_sbrx_r    <= cyc_2_sbrx_next;
            cyc_3_skip_r    <= cyc_3_skip_next;
            cyc_2_xxiw_r    <= cyc_2_xxiw_next;
            cyc_3_xxiw_r    <= cyc_3_xxiw_next;
            cyc_4_xxiw_r    <= cyc_4_xxiw_next;
            cyc_2_add_r     <= (idc.add or idc.inc) and cyc_1_r;
            cyc_2_sub_r     <= (idc.subo or idc.cp or idc.subic or idc.cpi or idc.neg or idc.dec or idc.cpse) and cyc_1_r;
            cyc_2_sbc_r     <= (idc.subo or idc.cp or idc.subic) and not(idc.sub_nc) and cyc_1_r; -- SBC/CPC/SBCI
            cyc_2_and_r     <= (idc.ando or idc.andi) and cyc_1_r;
            cyc_2_or_r      <= (idc.oro or idc.ori) and cyc_1_r;
            cyc_2_eor_r     <= idc.eor  and cyc_1_r;
            cyc_2_com_r     <= idc.com and cyc_1_r;
            cyc_2_neg_r     <= idc.neg and cyc_1_r; -- TODO: needed?
            cyc_2_inc_dec_r <= (idc.inc or idc.dec) and cyc_1_r;
            cyc_2_shf_r     <= (idc.asr or idc.lsr or idc.roro) and cyc_1_r;
            cyc_2_swap_r    <= idc.swap and cyc_1_r;
            cyc_2_mov_r     <= idc.mov and cyc_1_r;
            cyc_2_movw_r    <= cyc_2_movw_next;
            cyc_2_bld_clr_r <= cyc_2_bld_next and not(sreg_t_r);
            cyc_2_bld_set_r <= cyc_2_bld_next and sreg_t_r;
            cyc_2_bld_r     <= cyc_2_bld_next;
            cyc_2_walu_r    <= cyc_2_walu_next;
            cyc_2_alu_r     <= cyc_2_alu_next;
            cyc_2_ialu_r    <= cyc_2_ialu_next;
            cyc_2_bst_r     <= cyc_2_bst_next;
            cyc_2_lpm_r     <= cyc_2_lpm_next;
            cyc_3_lpm_r     <= cyc_3_lpm_next;
            cyc_2_spm_r     <= cyc_2_spm_next;
            cyc_3_spm_r     <= cyc_3_spm_next;
            cyc_1_irq_r     <= cyc_1_irq_next;
            cyc_2_irq_r     <= cyc_2_irq_next;
         end if;
      end if;
   end process do_fsm;
   -- Indicates if we are at the last cycle of an instruction
   cyc_1_next     <= not(cyc_2_out_next   or cyc_2_sts_next or
                         cyc_2_ld_next    or cyc_3_ld_next   or cyc_2_lds_next or
                         cyc_2_st_next    or cyc_2_push_next  or cyc_2_wioa_next or
                         cyc_2_sbix_next  or cyc_3_skip_next  or cyc_2_movw_next or
                         cyc_2_rcall_next or cyc_2_xxiw_next or
                         cyc_3_xxiw_next  or cyc_4_xxiw_next  or cyc_2_ret_next or
                         cyc_3_ret_next   or cyc_4_ret_next   or
                         cyc_2_alu_next   or cyc_2_ijmp_next  or
                         cyc_2_icall_next or cyc_2_sbrx_next  or cyc_fetch_next or
                         cyc_2_bst_next   or cyc_2_in_next   or cyc_3_st_next or
                         cyc_2_std_next   or cyc_2_lpm_next   or cyc_3_lpm_next or
                         cyc_2_spm_next   or cyc_3_spm_next or
                         cyc_2_irq_next   or cyc_2_ldd_next);
   dbg_cyc_last_o   <= cyc_1_next;
   -- Indicates the instruction finished, but PC changed and we need to wait for a fetch
   cyc_fetch_next   <= cyc_2_brbx_next or -- BRBC/BRBS w/branch
                       cyc_2_ijmp_r or -- IJMP
                       cyc_2_rjmp_next or -- RJMP
                       cyc_4_skip_next or -- SBIC/SBIS
                       cyc_2_rcall_r or cyc_2_icall_r or cyc_2_irq_r or -- RCALL/ICALL/IRQ
                       cyc_3_lpm_r or  -- LPM
                       cyc_3_spm_r; -- SPM
   cyc_2_rjmp_next  <= cyc_1_r and idc.rjmp; -- RJMP
   cyc_2_ijmp_next  <= cyc_1_r and idc.ijmp; -- IJMP
   cyc_2_brbx_next  <= cyc_1_r and idc.brbx and (alu_z xnor idc.brbc); -- BRBC/BRBS
   cyc_2_rcall_next <= cyc_1_r and idc.rcall; -- RCALL
   cyc_2_icall_next <= cyc_1_r and idc.icall; -- ICALL
   cyc_2_ret_next   <= cyc_1_r and idc.ret; -- RET/RETI
   cyc_3_ret_next   <= cyc_2_ret_r; -- RET/RETI
   cyc_4_ret_next   <= cyc_3_ret_r; -- RET/RETI
   cyc_2_out_next   <= cyc_1_r and idc.outo; -- OUT
   cyc_2_in_next    <= cyc_1_r and idc.ino; -- IN
   cyc_2_wioa_next  <= cyc_1_r and idc.xbi;  -- SBI/CBI
   cyc_2_sts_next   <= cyc_1_r and idc.sts;  -- STS
   cyc_2_st_next    <= cyc_1_r and idc.st; -- ST X/X+/-X/Y+/-Y/Z+/-Z,Rr
   cyc_3_st_next    <= cyc_2_st_r and do_upd_pointer; -- 3rd cycle to update X/Y/Z
   cyc_2_std_next   <= cyc_1_r and idc.stdo; -- STD
   cyc_2_push_next  <= cyc_1_r and idc.push; -- PUSH
   cyc_2_ld_next    <= cyc_1_r and idc.ld; -- LD Rd,X/X+/-X/Y+/-Y/Z+/-Z
   cyc_2_pop_next   <= cyc_1_r and idc.pop;
   cyc_3_ld_next    <= do_load or cyc_2_pop_next; -- LD/LDS/LDD/POP (Write Rd from memory)
   cyc_2_lds_next   <= cyc_1_r and idc.lds;  -- LDS Rd,K
   cyc_2_ldd_next   <= cyc_1_r and idc.ldd; -- LDD Rd,Y+q/Z+q
   cyc_2_sbix_next  <= cyc_1_r and idc.sbix; -- SBIC/SBIS
   cyc_2_sbrx_next  <= cyc_1_r and idc.sbrx; -- SBRC/SBRS
   cyc_2_cpse_next  <= cyc_1_r and idc.cpse; -- CPSE
   cyc_3_skip_next  <= ((cyc_2_sbix_r or cyc_2_sbrx_r) and (alu_z xor idc.set_r)) or -- SBIC/SBIS/SBRC/SBRS
                       (cyc_2_cpse_r and alu_z); -- CPSE
   cyc_4_skip_next  <= cyc_3_skip_r and (idc.sts or idc.lds); -- SBIC/SBIS
   cyc_2_xxiw_next  <= cyc_1_r and idc.xxiw; -- ADIW/SBIW
   cyc_3_xxiw_next  <= cyc_2_xxiw_r; -- ADIW/SBIW
   cyc_4_xxiw_next  <= cyc_3_xxiw_r; -- ADIW/SBIW
   -- ALU operations that write alu_s to Rd
   cyc_2_walu_next  <= (idc.add or idc.subo or idc.ando or idc.oro or idc.eor or -- ADD/ADC/SUB/SBC/AND/OR/EOR
                        idc.subic or idc.andi or idc.ori or idc.com or idc.neg or-- SUBI/SBCI/ANDI/ORI/COM/NEG
                        idc.inc or idc.dec or idc.asr or idc.lsr or idc.roro or  -- INC/DEC/ASR/LSR/ROR
                        idc.swap or idc.mov or idc.bld) -- SWAP/MOV/BLD
                        and cyc_1_r;
   -- ALU operations with immediate operand
   cyc_2_ialu_next  <= (idc.subic or idc.andi or idc.ori or idc.cpi) and cyc_1_r;-- SUBI/SBCI/ANDI/ORI/CPI
   -- ALU operations (all)
   cyc_2_alu_next   <= ((idc.cp or idc.cpi or idc.cpse) and cyc_1_r) -- CP/CPC/CPI/CPSE
                       or cyc_2_walu_next; -- Also all the ALU Write stuff
   cyc_2_bst_next   <= cyc_1_r and idc.bst; -- BST
   cyc_2_lpm_next   <= cyc_1_r and idc.lpm; -- LPM
   cyc_3_lpm_next   <= cyc_2_lpm_r;
   cyc_2_spm_next   <= cyc_1_r and idc.spm when ENA_SPM='1' else '0'; -- SPM
   cyc_3_spm_next   <= cyc_2_spm_r when ENA_SPM='1' else '0';
   cyc_1_bc_bs      <= idc.bc_bs and cyc_1_r; -- BSET & BCLR
   cyc_1_bclr       <= cyc_1_bc_bs and idc.bclr; -- BCLR
   cyc_1_bset       <= cyc_1_bc_bs and not(idc.bclr); -- BSET
   cyc_1_brbx       <= idc.brbx and cyc_1_r; -- BRBC/BRBS
   cyc_1_ldi        <= idc.ldi and cyc_1_r; -- LDI
   cyc_2_bld_next   <= idc.bld and cyc_1_r; -- BLD
   cyc_2_movw_next  <= idc.movw and cyc_1_r; -- MOVW (AVR25 or AVR4)
   -- Instruction groups
   do_push          <= cyc_2_push_r or                      -- PUSH Rr
                       cyc_2_rcall_next or cyc_2_rcall_r or -- RCALL K (1&2)
                       cyc_2_icall_next or cyc_2_icall_r or -- ICALL K (1&2)
                       cyc_1_irq_r or cyc_2_irq_r;          -- IRQ (1&2)
   do_pop           <= cyc_2_pop_next or              -- POP Rd
                       cyc_2_ret_next or cyc_2_ret_r; -- RET/RETI (1&2)
   do_store         <= cyc_2_st_r or cyc_2_std_r or cyc_2_sts_r or -- ST X/Y/Z,Rr; STD Y/Z+q,Rr; STS K,Rr (2 Memory write)
                       cyc_2_push_r; -- PUSH Rr
   do_load          <= cyc_2_ld_r or cyc_2_ldd_r or cyc_2_lds_r; -- LD Rd,X/Y/Z; LDD Rd,Y/Z+q; LDS Rd,K (2 Memory read)
   do_pre_dec       <= idc.ld_op_r=LD_PRE_DEC; -- Pre-decrement pointer
   do_post_inc      <= idc.ld_op_r=LD_POST_INC; -- Post-increment pointer
   do_upd_pointer   <= '1' when idc.ld_op_r=LD_PRE_DEC or idc.ld_op_r=LD_POST_INC else '0'; -- Pre-decrement/post-increment pointer

   ----------------------------------------------------------------------------
   ----------------------------------------------------------------------------
   -- MEMORY SPACE
   -- 0x0000 to 0x001F Registers
   -- 0x0020 to 0x005F I/O (maps to I/O 0x00 to 0x3F)
   -- 0x0060 to 0xFFFF SRAM
   -- X reg is r26+r27 => 0x1A/0x1B
   -- Y reg is r28+r29 => 0x1C/0x1D
   -- Z reg is r30+r31 => 0x1E/0x1F
   ----------------------------------------------------------------------------
   ----------------------------------------------------------------------------
   -- Memory address (RF+I/O+SRAM)
   -- Store address source
   -- Immediate
   fadr_imm <= cyc_2_sts_r or cyc_2_lds_r; -- STS K,Rr; LDS Rd,K
   -- X/Y/Z Pointer + q(imm)
   fadr_p_q <= cyc_2_ldd_r or cyc_2_std_r; -- LDD/STD Rd,Y+q/Z+q
   -- X/Y/Z Pointer (LD/ST)
   fadr_p   <= cyc_2_ld_r or cyc_2_st_r; -- LD Rd,X/Y/Z - ST X/Y/Z,Rr

   -- Pointer offset (-1,0 for LD/ST or Q for LDD/STDD)
   fadr_p_off1 <= (others => '1') when do_pre_dec else -- LD/ST w/pre-dec
                  (others => '0');                         -- LD/ST no pre-dec
   fadr_p_off  <= resize(idc.q_r,16) when fadr_p_q='1' else fadr_p_off1;

   -- Stack pointer and its offset
   sp16        <= resize(unsigned(sp_i),16);
   fadr_sp_off <= x"0001" when do_pop='1' else x"0000";

   full_adr <= (unsigned(inst_stop)         and fadr_imm) or
               (sp16+fadr_sp_off            and (do_push or do_pop))  or
               (rd16_read+fadr_p_off        and (fadr_p or fadr_p_q));
   full_adr_is_reg <= '1' when full_adr<32 else '0';
   full_adr_is_io  <= '1' when full_adr>=32 and full_adr<96 else '0';
   full_adr_is_ram <= '1' when full_adr>=96 else '0';
   full_adr_src:
   process (clk_i)
   begin
      if rising_edge(clk_i) then
         if rst_i='1' then
            full_adr_is_reg_r <= '0';
            full_adr_is_ram_r <= '0';
            full_adr_is_io_r  <= '0';
         else
            full_adr_is_reg_r <= full_adr_is_reg;
            full_adr_is_ram_r <= full_adr_is_ram;
            full_adr_is_io_r  <= full_adr_is_io;
         end if;
      end if;
   end process full_adr_src;

   ----------------------------------------------------------------------------
   ----------------------------------------------------------------------------
   -- Register File
   ----------------------------------------------------------------------------
   ----------------------------------------------------------------------------
   RegFile : entity avr.RegisterFile
      port map(
         clk_i => clk_i,
         rd_adr_i => rd_adr, rd_i => rd_write, rd_o => rd_read_aux,
         rd_we_i => rd_we, rd16_we_i => rd16_we, rd16_i => rd16_write,
         rd16_o => rd16_read, rr_adr_i => rr_adr, rr_o => rr_read_aux);
   -- Data from RF, can be faked from the debug unit
   rd_read <= rd_read_aux when dbg_rf_fake_i='0' else dbg_rd_data_i;
   rr_read <= rr_read_aux when dbg_rf_fake_i='0' else dbg_rr_data_i;
   ---------------------
   -- Rd data to write -
   ---------------------
   -- From ALU
   rd_dat_alu <= cyc_2_xxiw_r or cyc_4_xxiw_r or   -- ADIW/SBIW Rd,k
                 cyc_2_walu_r or                   -- ADD/ADC/SUB/SBC/AND/OR/EOR/LSR/ASR/ROR/COM/NEG/SWAP/MOV
                 (do_store and full_adr_is_reg) or -- ST* & destination is a Reg address
                 cyc_3_ld_r or                     -- LD Rd,X/Y/Z; LDS Rd,K
                 cyc_2_in_r;                       -- IN Rd,A (2 Write Rd)
   -- Immediate
   rd_dat_imm <= cyc_1_ldi; -- LDI Rd,K
   -- From ROM, even address
   rd_dat_pr0 <= cyc_3_lpm_r and not(rd16_r(0)); -- LPM Z is even
   -- From ROM, odd address
   rd_dat_pr1 <= cyc_3_lpm_r and rd16_r(0); -- LPM Z is odd
   rd_write <= (std_logic_vector(alu_s)                and rd_dat_alu) or
               (inst_i(11 downto 8)&inst_i(3 downto 0) and rd_dat_imm) or
               (inst_i(15 downto 8)                    and rd_dat_pr1) or
               (inst_i( 7 downto 0)                    and rd_dat_pr0);
   -----------
   -- Rd WE --
   -----------
   rd_we <= rd_dat_alu or rd_dat_imm or rd_dat_pr1 or rd_dat_pr0;
   ----------------
   -- Rd address --
   ----------------
   do_rd_pointer1 <= cyc_2_ld_next or -- LD Rd,xx/xx+/-xx (1 Read pointer)
                     cyc_2_st_next or -- ST xx,Rr (1 Read pointer)
                     cyc_2_ld_r or    -- LD Rd,xx/xx+/-xx (2 Update pointer)
                     cyc_3_st_r;      -- ST xx,Rr (3 Update pointer)
   do_rd_pointer2 <= cyc_2_ldd_next or -- LDD Rd,xx+q (1 Read pointer)
                     cyc_2_std_next;   -- STD xx+q,Rr (1 Read pointer)
   do_adr_x <= '1' when inst_cur(3 downto 2)=LD_X and do_rd_pointer1='1' else '0'; -- ST/LD
   do_adr_y <= '1' when (inst_cur(3 downto 2)=LD_Y and do_rd_pointer1='1') or -- ST/LD
                        (idc.ldd_y and do_rd_pointer2)='1'          -- STD/LDD
                        else '0';
   do_adr_z <= '1' when (inst_cur(3 downto 2)=LD_Z and do_rd_pointer1='1') or -- ST/LD
                        (not(idc.ldd_y) and do_rd_pointer2)='1' or  -- STD/LDD
                         cyc_2_lpm_next='1' or                      -- LPM (1 Read pointer)
                         cyc_2_lpm_r='1' or                         -- LPM (2 Read PROM) we need Zlsb
                         cyc_2_spm_r='1' or                         -- SPM (2 Read pointer)
                         --cyc_3_spm_r='1' or                         -- SPM (3 Write PROM) we need Zlsb
                         cyc_2_icall_next='1' or                    -- ICALL (1 Read pointer)
                         cyc_2_ijmp_next='1'                        -- IJMP (1 Read pointer)
                        else '0';
   -- Address r16 to r31
   do_adr_rhi <= cyc_1_ldi or -- LDI Rd,K
                 cyc_2_ialu_next or cyc_2_ialu_r; -- SUBI/SBCI/ANDI/ORI/CPI
   -- Address from memory map
   do_adr_mem <= do_store;
   -- Address low xxIW reg
   do_adr_iw0 <= cyc_2_xxiw_next or cyc_2_xxiw_r; -- ADIW/SBIW Rd,k (1/2 R/W Rd)
   -- Address high xxIW reg
   do_adr_iw1 <= cyc_3_xxiw_r or cyc_4_xxiw_r;    -- ADIW/SBIW Rd,k (3/4 R/W Rd+1)
   -- Address R0
   lpm_r0     <= '1' when not(ENA_AVR25='1' or ENA_AVR4) or -- No Rd or
                          inst_cur(2)='0' else '0';     -- Simple version
   do_adr_r0  <= (cyc_3_lpm_r and lpm_r0) or -- LPM (3 Write R0)
                 cyc_2_spm_next; -- SPM (1 Read R1:R0)
   -- Address Rr&'0' (MOVW)
   do_adr_rr0 <= cyc_2_movw_next;
   -- Address Rd&'0' (MOVW)
   do_adr_rd0 <= cyc_2_movw_r;
   -- Address Rd
   do_adr_rd  <= not(do_adr_rhi or do_adr_mem or do_adr_iw0 or do_adr_iw1 or
                     do_adr_r0  or do_adr_x   or do_adr_y   or do_adr_z or
                     do_adr_rr0 or do_adr_rd0); -- Default
                 -- ADD/ADC/SUB/SBC/AND/OR/EOR/COM
                 -- CP/CPC/CPSE
                 -- BLD
                 -- cyc_3_ld_r='1' or cyc_2_in_r='1' LD Rd,X/Y/Z (write Rd); IN
                 -- LPM (3 Write Rd)
   rd_adr <= (unsigned('1'&inst_cur(7 downto 4))      and do_adr_rhi) or
             (full_adr(4 downto 0)                    and do_adr_mem) or
             (to_unsigned(26,5)                       and do_adr_x)   or
             (to_unsigned(28,5)                       and do_adr_y)   or
             (to_unsigned(30,5)                       and do_adr_z)   or
             (unsigned("11"&inst_cur(5 downto 4)&'0') and do_adr_iw0) or
             (unsigned("11"&inst_cur(5 downto 4)&'1') and do_adr_iw1) or
             (unsigned(inst_cur(3 downto 0)&'0')      and do_adr_rr0) or
             (unsigned(inst_cur(7 downto 4)&'0')      and do_adr_rd0) or
             (unsigned(inst_cur(8 downto 4))          and do_adr_rd);
   -------------------
   -- Rd 16 bits WE --
   -------------------
   lpm_inc <= '1' when inst_cur(0)='1' and (ENA_AVR25='1' or ENA_AVR4) else '0';
   rd16_we <= (cyc_2_ld_r and do_upd_pointer) or -- LD -R/R+
               cyc_3_st_r or                     -- ST -R/R+
               cyc_2_movw_r or                   -- MOVW
              (cyc_2_lpm_r and lpm_inc);         -- LPM Rd,Z+
   -----------------------------
   -- Rd 16 bits data to write -
   -----------------------------
   do_rd16_reg:
   process (clk_i)
   begin
      if rising_edge(clk_i) then
         if rst_i='1' then
            rd16_r <= (others => '0');
         else
            rd16_r <= rd16_read;
         end if;
      end if;
   end process do_rd16_reg;
   cur_ptr    <= rd16_r when cyc_3_st_r='1' else rd16_read;
   rd16_write <= cur_ptr   when cyc_2_movw_r='1' else
                 cur_ptr-1 when do_pre_dec else
                 cur_ptr+1;
   ----------------
   -- Rr address --
   ----------------
   do_adr2_mem <= do_load;
   -- do_adr2_rd  <= ((idc.neg or idc.sbrx or idc.bst or idc.bld or idc.st or
   --                     idc.push or idc.stdo or idc.sts or idc.outo) and cyc_1_r) or
   --                    cyc_2_sts_r or cyc_2_out_r; -- ST X/Y/Z,Rr; PUSH Rr; NEG; STS TODO: esto es para 0-Rd ¿y /Rd+1? ¿and cyc_1_r?
   do_adr2_rd <= not(do_adr2_rr or do_adr2_mem);
   --do_adr2_rr  <= not(do_adr2_rd or do_adr2_mem);
   do_adr2_rr <= cyc_2_alu_next and -- (CP/CPC/CPSE)+(ADD/ADC/SUB/SBC/AND/OR/EOR)+(SUBI/SBCI/ANDI/ORI)+(LSR/ASR/ROR)+(COM/NEG/SWAP/MOV)+BLD
                 not(idc.neg) and    -- !NEG
                 not(idc.bld);       -- !BLD
   rr_adr  <= (full_adr(4 downto 0)                       and do_adr2_mem) or
              (unsigned(inst_cur(8 downto 4))             and do_adr2_rd)  or
              (unsigned(inst_cur(9)&inst_cur(3 downto 0)) and do_adr2_rr);
              -- Note: using inst_cur for do_adr2_rr saves a couple of LUTs,
              -- but the final result is worst (more used in routing).

   ----------------------------------------------------------------------------
   -- I/O Control
   ----------------------------------------------------------------------------
   -----------------
   -- I/O Address --
   -----------------
   io_map_a <= full_adr(6 downto 0)-32;
   -- Immediate 1 (IN/OUT)
   io_adr_imm1 <= cyc_2_out_r or cyc_2_in_next; -- OUT A,Rr (cyc 2) - IN Rd,A (cyc 1)
   -- Immediate 2 (Bit manipulation)
   io_adr_imm2 <= cyc_2_wioa_next or -- SBI/CBI (1 Read)
                  cyc_2_sbix_next or -- SBIS/SBIC (1 Read)
                  cyc_2_wioa_r;      -- SBI/CBI (2 Write)
   -- From memory address
   io_adr_mem <= do_store or do_load; -- ST*; LD*
   io_adr_o <= (unsigned(inst_cur(10 downto 9))&unsigned(inst_cur(3 downto 0))   and io_adr_imm1) or
               (unsigned('0'&inst_cur(7 downto 3))                               and io_adr_imm2) or
               (io_map_a(5 downto 0)                                             and io_adr_mem);
   ------------
   -- I/O WE --
   ------------
   io_we_o  <= cyc_2_out_r or  -- OUT A,Rr (2 Write)
               cyc_2_wioa_r or -- SBI/CBI (2 Write)
               (do_store and full_adr_is_io); -- ST* (2 Write)
   ------------
   -- I/O RE --
   ------------
   io_re    <= cyc_2_wioa_next or -- SBI/CBI (1 Read)
               cyc_2_sbix_next or -- SBIS/SBIC (1 Read)
               cyc_2_in_next or  -- IN (1 Read)
               (do_load and full_adr_is_io); -- LD* (2 Read)
   io_re_o  <= io_re;
   ---------------------
   -- I/O & SRAM data --
   ---------------------
   -- From PC Low
   data_from_pcl <= cyc_2_rcall_next or cyc_2_icall_next or cyc_1_irq_r; -- xCALL/IRQ (1 push PC low)
   -- From PC High
   data_from_pch <= cyc_2_rcall_r or cyc_2_icall_r or cyc_2_irq_r; -- xCALL/IRQ (2 push PC high)
   -- From ALU (Almost all is routed thru the ALU)
   data_from_alu <= not(data_from_pcl or data_from_pch);
   --data_from_alu  <= do_store or cyc_2_out_r or cyc_2_wioa_r; -- ST*; PUSH Rr (2 Write); OUT (2 Write); SBI/CBI (2 Write)
   data_o <= (std_logic_vector(pc_l_r) and data_from_pcl) or
             (std_logic_vector(pc_h_r) and data_from_pch) or
             (std_logic_vector(alu_s)  and data_from_alu);

   io_register:
   process (clk_i)
   begin
      if rising_edge(clk_i) then
         if rst_i='1' then
            io_data_r <= (others => '0');
         elsif io_re='1' then
            io_data_r <= io_data_i;
         end if;
      end if;
   end process io_register;
   ----------------------------------------------------------------------------
   -- Data memory control
   ----------------------------------------------------------------------------
   ram_adr_o <= std_logic_vector(full_adr(RAM_ADR_W-1 downto 0));
   ram_we_o  <= (do_store or do_push) and full_adr_is_ram; -- ST*;PUSH;xCALL;IRQ
   ram_re_o  <= (do_load  or do_pop)  and full_adr_is_ram; -- LD*;POP;RET*

   ----------------------------------------------------------------------------
   -- Registers implemented in the I/O space
   ----------------------------------------------------------------------------
   ----------------------------------------------------------------------------
   -- Status Register
   ----------------------------------------------------------------------------
   sreg_out <= std_logic_vector(alu_s) when cyc_1_bc_bs='1' else
               sreg_if&sreg_t&alu_h&sreg_s&sreg_v&sreg_n&sreg_z&sreg_c;
   sreg_o <= sreg_out;
   -- Flags computed here:
   sreg_s  <= sreg_n xor sreg_v;
   sreg_n  <= alu_s(7);
   sreg_v  <= ((sreg_n xor alu_c) and cyc_2_shf_r) or
              (alu_v              and not(cyc_2_and_r or cyc_2_or_r or cyc_2_eor_r or cyc_2_com_r or cyc_2_shf_r));
   sreg_if <= not(cyc_2_irq_r); -- IRQ => 0; RETI => 1
   sreg_c  <= '1' when cyc_2_com_r='1' else alu_c;
   sreg_t  <= not(alu_z); -- BST
   -- ADIW and SUBIW computes the Z of the 16 bits result, so we must check its previous value
   -- SBC/SBCI/CPC also computes Z using its previous result
   sreg_z  <= alu_z when cyc_4_xxiw_r='0' and cyc_2_sbc_r='0' else alu_z and sreg_z_r;
   -- Which flags are altered
   do_add_sub_flag <= (cyc_2_add_r or cyc_2_sub_r) and not(cyc_2_cpse_r) and not(cyc_2_inc_dec_r); -- ADD/ADC/SUB/SBC/CP/CPC/SUBI/SBCI/NEG (and not inc/dec)
   do_shift_flag   <= (cyc_2_xxiw_r or cyc_4_xxiw_r or cyc_2_com_r or cyc_2_shf_r); -- ADIW/SBIW/COM/ASR/LSR/ROR
   do_arith_flag   <= do_shift_flag or do_add_sub_flag;
   do_alu_flag     <= do_arith_flag or cyc_2_and_r or cyc_2_or_r or cyc_2_eor_r or cyc_2_inc_dec_r; -- AND/OR/EOR/ANDI/ORI/INC/DEC
   -- I
   sreg_we_out1(7) <= ((cyc_3_ret_r and idc.reti_r) or cyc_2_irq_r); -- RETI/IRQ
   -- T
   sreg_we_out1(6) <= cyc_2_bst_r;
   -- H
   sreg_we_out1(5) <= do_add_sub_flag;
   -- S, V, N, Z
   sreg_we_out1(4) <= do_alu_flag;
   sreg_we_out1(3) <= do_alu_flag;
   sreg_we_out1(2) <= do_alu_flag;
   sreg_we_out1(1) <= do_alu_flag;
   -- C
   sreg_we_out1(0) <= do_arith_flag;
   sreg_we_out <= x"FF" when cyc_1_bc_bs='1' else -- BSET/BCLR
                             sreg_we_out1;
   sreg_we_o <= sreg_we_out;
   -- Future value for SREG(I)
   sreg_i_next <= sreg_i_r when sreg_we_out(7)='0' else sreg_out(7);
   ----------------------------------------------------------------------------
   -- Stack Pointer
   ----------------------------------------------------------------------------
   -- Direction of changing of stack pointer 0->up(+) 1->down(-)
   sp_pop_o <= not(do_push);
   sp_we_o <= do_push or do_pop;


   ----------------------------------------------------------------------------
   ----------------------------------------------------------------------------
   -- ALU
   ----------------------------------------------------------------------------
   ----------------------------------------------------------------------------
   cin(0) <= alu_ci;
   -----------------------
   -- Adder: A+B        --
   -----------------------
   -- Low nibble
   add_lo <= resize(alu_a(3 downto 0),5)+alu_b(3 downto 0)+cin;
   -- High nibble
   add_hi <= resize(alu_a(7 downto 4),5)+alu_b(7 downto 4)+add_lo(4 downto 4);
   -----------------------
   -- Substractor: A-B  --
   -----------------------
   -- Low nibble
   sub_lo <= resize(alu_a(3 downto 0),5)-alu_b(3 downto 0)-cin;
   -- High nibble
   sub_hi <= resize(alu_a(7 downto 4),5)-alu_b(7 downto 4)-sub_lo(4 downto 4);

   -- Result
   alu_s <= (add_hi(3 downto 0)&add_lo(3 downto 0) and op_add) or
            (sub_hi(3 downto 0)&sub_lo(3 downto 0) and op_sub) or
            (alu_ci&alu_a(7 downto 1)              and op_shf) or
            ((alu_a and alu_b)                     and op_and) or
            ((alu_a or  alu_b)                     and op_or)  or
            ((alu_a xor alu_b)                     and op_xor) or
            (alu_a(3 downto 0)&alu_a(7 downto 4)   and op_swp);
   -- Zero flag
   alu_z <= '1' when alu_s=x"00" else '0';
   -- Carry flag
   alu_c <= (add_hi(4) and op_add) or -- Cy from the high nibble
            (sub_hi(4) and op_sub) or -- Borrow from the high nibble
            (alu_a(0)  and op_shf);
   -- Half carry flag
   alu_h <= (add_lo(4) and op_add) or -- Cy from the low nibble
            (sub_lo(4) and op_sub);   -- Borrow from the low nibble
   -- Overflow flag
   alu_v <= (((alu_a(7) and alu_b(7) and not(add_hi(3))) or   -- (-)+(-)=(+)
             (not(alu_a(7)) and not(alu_b(7)) and add_hi(3))) -- (+)+(+)=(-)
             and op_add) or
            (((alu_a(7) and not(alu_b(7)) and not(sub_hi(3))) or -- (-)-(+)=(+)
             (not(alu_a(7)) and alu_b(7) and sub_hi(3)))         -- (+)-(-)=(-)
             and op_sub) or
            ((alu_s(7) xor alu_s(0)) and op_shf);

   -----------
   -- ALU A --
   -----------
   -- Bitmask, Rd or 0. Optional: complemented
   -- Is a bitmask
   a_from_mask <= cyc_2_wioa_r or -- CBI/SBI A,b
                  cyc_2_sbix_r or -- SBIS/SBIC (2 Check condition)
                  cyc_2_sbrx_r or -- SBRS/SBRC (2 Check condition)
                  cyc_2_bst_r  or -- BST Rd,b (2 Write SREG)
                  cyc_2_bld_r  or -- BLD
                  cyc_1_brbx   or -- BCLR/BSET b
                  cyc_1_bc_bs;    -- BRBC/BRBS s,k
   -- Is Rd
   a_from_rd <= (cyc_2_xxiw_r or cyc_4_xxiw_r or -- ADIW/SBIW (2&4)
                 cyc_2_alu_r) -- CP/CPC/CPI + ADD/ADC/SUB/SBC/AND/OR/EOR/SUBI/SBCI/ANDI/ORI/COM/NEG/LSR/ASR/ROR/SWAP/MOV
                 and not(cyc_2_neg_r)  -- !NEG
                 and not(cyc_2_mov_r)  -- !MOV
                 and not(cyc_2_bld_r); -- !BLD
   alu_a_1 <= (bit_mask          and a_from_mask) or
              (unsigned(rd_read) and a_from_rd);
              -- 0 for: NEG (cyc_2_neg_r) MOV (cyc_2_mov_r) do_store or cyc_2_out_r or cyc_2_in_r (cyc_3_ld_r and full_adr_is_reg_r)
   alu_a <= not(alu_a_1) when ((cyc_2_wioa_r and not(idc.set_r)) or -- CBI A,b
                               cyc_1_bclr      or                   -- BCLR b (1 cycle)
                               cyc_2_bld_clr_r or                   -- BLD w/T=0
                               cyc_2_com_r                          -- COM Rd
                               )='1' else
            alu_a_1;

   -----------
   -- ALU B --
   -----------
   -- I/O, Immediate(small/full), SREG, Rr or 0.
   b_from_io   <= cyc_2_wioa_r or cyc_2_sbix_r or cyc_2_in_r or -- SBI/CBI A,b; SBIS/SBIC (2 Check condition)
                  (cyc_3_ld_r and full_adr_is_io_r);
   b_from_imm1 <= cyc_2_ialu_r; -- SUBI/SBCI/ANDI/ORI/CPI
   b_from_imm2 <= cyc_2_xxiw_r; -- ADIW/SBIW (2)
   b_from_sreg <= cyc_1_bc_bs or cyc_1_brbx; -- BCLR/BSET b; BRBC/BRBS s,k
   b_from_0    <= cyc_2_inc_dec_r or cyc_4_xxiw_r or cyc_2_com_r; -- INC/DEC (cyc_2_inc_dec_r) ADIW(4) (cyc_4_xxiw_r) COM
   b_from_mem  <= cyc_3_ld_r and full_adr_is_ram_r; -- LD Rd,X/Y/Z; LDS Rd,K (write Rd from SRAM)
   b_from_rr   <= not(b_from_io or b_from_imm1 or b_from_imm2 or b_from_sreg or b_from_0 or b_from_mem);
                  -- when ((cyc_2_alu_r or  -- CP/CPC + ADD/ADC/SUB/SBC/AND/OR/EOR/MOV/NEG
                               --      cyc_2_sbrx_r or -- SBRS/SBRC
                               --      (cyc_3_ld_r and full_adr_is_reg_r)
                               --      cyc_2_bst_r or  -- BST
                               --      cyc_2_out_r or do_store (Rr thru ALU: PUSH, OUT, ST*
                               --      ) and not(cyc_2_inc_dec_r))='1' else
   alu_b <= (unsigned(io_data_r)                                            and b_from_io) or
            (unsigned(inst_r(11 downto 8))&unsigned(inst_r(3 downto 0))     and b_from_imm1) or
            ("00"&unsigned(inst_r(7 downto 6))&unsigned(inst_r(3 downto 0)) and b_from_imm2) or
            (unsigned(sreg_i)                                               and b_from_sreg) or
            (x"00"                                                          and b_from_0) or
            (unsigned(ram_data_i)                                           and b_from_mem) or
            (unsigned(rr_read)                                              and b_from_rr);
   -- Shifts differ in the Cin, clasify them
   is_ror <= '1' when idc.shf_op_r="11" else '0';
   is_asr <= not(idc.shf_op_r(1));
   -- is_lsr => Cin=0
   alu_ci_1 <= (sreg_c_r and (cyc_4_xxiw_r or -- ADIW/SBIW (4, 2nd add)
                             (cyc_2_add_r and idc.add_wc_r) or -- ADC
                             cyc_2_sbc_r or -- SBC/CPC/SBCI
                             (cyc_2_shf_r and is_ror))) -- ROR
                or
                (rd_read(7) and (cyc_2_shf_r and is_asr)); -- ASR
   alu_ci <= alu_ci_1 or cyc_2_inc_dec_r; -- INC/DEC always uses Cin='1'

   op_add <= ((cyc_2_xxiw_r or cyc_4_xxiw_r) and not(idc.is_sub_r)) or cyc_2_add_r; -- ADIW/ADC/ADD/INC
   op_sub <= cyc_2_sub_r or  -- SUB/SBC/CP/CPC/SUBI/SBCI/CPI/NEG/DEC/CPSE
             ((cyc_2_xxiw_r or cyc_4_xxiw_r) and idc.is_sub_r); -- SBIW
   op_and <= (cyc_2_wioa_r and not(idc.set_r)) or -- CBI
             cyc_2_sbix_r or                      -- SBIS/SBIC
             cyc_2_sbrx_r or                      -- SBRS/SBRC
             cyc_1_bclr or                        -- BCLR
             cyc_1_brbx or                        -- BRBC/BRBS
             cyc_2_bst_r or                       -- BST
             cyc_2_bld_clr_r or                   -- BLD w/T=0
             cyc_2_and_r;                         -- AND
   op_or  <= (cyc_2_wioa_r and idc.set_r) or     -- SBI
             cyc_1_bset or                       -- BSET
             cyc_2_or_r or                       -- OR
             cyc_2_bld_set_r or                  -- BLD w/T=1
             cyc_2_mov_r or                      -- MOV
             cyc_3_ld_r or                       -- LD*
             do_store or                         -- ST*
             cyc_2_com_r or                      -- COM
             cyc_2_out_r or cyc_2_in_r;          -- IN/OUT
   op_xor <= cyc_2_eor_r; -- EOR
   op_shf <= cyc_2_shf_r; -- ASR/LSR/ROR
   op_swp <= cyc_2_swap_r; -- SWAP

   do_bit_mask:
   process (inst_cur, cyc_1_bc_bs)
      variable b : std_logic_vector(2 downto 0);
   begin
      bit_mask <= (others => '0');
      if cyc_1_bc_bs='1' then
         b:=inst_cur(6 downto 4);
      else
         b:=inst_cur(2 downto 0);
      end if;
      bit_mask(to_integer(unsigned(b))) <= '1';
   end process do_bit_mask;

   ---------------------------------------
   -- Interrupt logic and state machine --
   ---------------------------------------
   irq_req <= '0' when unsigned(irq_lines_i)=0 else '1';

   -- Compute the IRQ jump
   comp_vector:
   process (irq_lines_i)
   begin
      irq_vector_adr <= (others => '0');
      for i in irq_lines_i'high downto 0 loop
          if irq_lines_i(i)='1' then
             irq_vector_adr <= to_unsigned(i+1,IRQ_ID_W);
          end if;
      end loop;
   end process comp_vector;

   -- Interrupts can be attended when all these conditions are met:
   -- 1) We are at the beggining of an instruction (not in the middle)
   -- 2) The interrupt flag is 1
   -- 3) We executed at least one instruction after the last interrupt
   cyc_1_irq_next <= irq_req and sreg_i_next and not(was_irq_next) and cyc_1_next;

   -- ACK the request during the 2nd cycle
   ack_decoder:
   for i in irq_acks_o'range generate
       irq_acks_o(i) <= cyc_2_irq_r when irq_vector_adr=i+1 else '0';
   end generate ack_decoder;

   -- When returning from an IRQ we must execute at least one instruction
   -- before serving another interrupt. This flag indicates we returned
   -- from an interrupt and blocks the interrupts.
   was_irq_next <= not(rst_i) and cyc_4_ret_r and idc.reti_r;
   irq_ok_o     <= irq_req;
   -- ########################################################################################

   -- Sleep
   sleep_o <= idc.sleep;
   -- Watchdog
   wdr_o <= idc.wdr;

   ---------------------
   -- Debug Interface --
   ---------------------
   -- For 32 bits instructions we wait until we have the second part fetched
   do_debug_stuff:
   if ENA_DEBUG='1' generate
      dbg_inst_o    <= inst_cur; -- Current instruction (under execution)
      dbg_inst2_o   <= inst_i;   -- 2nd part of a 32 bits instruction
      dbg_exec_o    <= '1' when ((cyc_1_r='1' and cyc_2_lds_next='0' and cyc_2_sts_next='0') or -- 16 bits instruction cycle 1
                                cyc_2_lds_r='1' or cyc_2_sts_r='1') -- 32 bits instruction cycle 2
                                and ena_i='1' else '0';
      dbg_is32_o    <= cyc_2_lds_r or cyc_2_sts_r;
      dbg_stopped_o <= stopped;
      dbg_rd_data_o <= std_logic_vector(alu_s);
      dbg_rd_we_o   <= rd_we;

      do_dbg_pc:
      process (clk_i)
      begin
         if rising_edge(clk_i) then
            if rst_i='1' then
               dbg_pc_o <= (others => '0');
            elsif cyc_1_next='1' then
               dbg_pc_o <= pc;
            end if;
         end if;
      end process do_dbg_pc;
   end generate do_debug_stuff;

   stopped <= '1' when ENA_DEBUG='1' and dbg_stop_i='1' else '0';
   enabled <= ena_i and not(stopped);

   do_was_ena_r:
   process (clk_i)
   begin
      if rising_edge(clk_i) then
         if rst_i='1' or enabled='1' then
            was_ena_r <= '1';
         else
            was_ena_r <= '0';
         end if;
      end if;
   end process do_was_ena_r;

   -- I/O instructions can block the CPU in the middle of an instruction (Wait States)
   -- In this case we could be doing a new fetch and will lose the inst_i content.
   -- This signal retains inst_i value during a stop condition.
   do_inst_stop_r:
   process (clk_i)
   begin
      if rising_edge(clk_i) then
         if rst_i='1' then
            inst_stop_r <= (others => '0');
         elsif was_ena_r='1' then
            inst_stop_r <= inst_i;
         end if;
      end if;
   end process do_inst_stop_r;
   inst_stop <= inst_i when was_ena_r='1' else inst_stop_r;
end architecture RTL; -- Entity: CPU

