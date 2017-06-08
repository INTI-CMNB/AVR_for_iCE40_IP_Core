/***********************************************************************

  AVR CPU

  This file is part FPGA Libre project http://fpgalibre.sf.net/

  Description:
  AVR 2.0 and 2.5 CPU implementation.

  To Do:
  -

  Author:
  Salvador E. Tropea

------------------------------------------------------------------------------

 Copyright (c) 2016-2017  <salvador en inti.gob.ar>
 Copyright (c) 2016-2017 Instituto Nacional de Tecnología Industrial

 Distributed under the GPL v2 or newer license

------------------------------------------------------------------------------

 Design unit:      CPU
 File name:        cpu.v
 Note:             None
 Limitations:      None known
 Errors:           None known
 Library:          avr
 Dependencies:     IEEE.std_logic_1164
                   IEEE.numeric_std
                   avr.Constants
                   avr.Types
 Target FPGA:      iCE40HX4K-TQ144
 Language:         Verilog
 Wishbone:         None
 Synthesis tools:  Lattice iCECube2 2016.02.27810
 Simulation tools: GHDL [Sokcho edition] (0.2x)
 Text editor:      SETEdit 0.5.x

***********************************************************************/
// TODO:
// Posibles optimizaciones.
// * Si en un ciclo tiene que leer un registro Rx y ese registro ya estaba direccionado en la
//   instrucción anterior ... es posible saltear la lectura, ya se hizo.
// * Hay un tema con las instrucciones de lectura de I/O. En principio las pensé sincrónicas, e
//   pero en realidad son asincrónicas. Así hay varias instrucciones que se acotarían en 1 ciclo
//   Lo que hice fue sincronizarlas con io_data_r, en la esperanza que eso les de más tiempo para
//   decodificar el espacio de I/O, pero quizás no valga la pena. Evaluar.
//

module CPU
  #(
    parameter IRQ_ID_W=2,  // Width of the ID
    parameter ENA_AVR25=0, // Enable AVR25 instructions (MOVW/LPM Rd,Z)
    parameter ENA_AVR3=0,  // Enable AVR3 instructions
    parameter ENA_AVR4=0,  // Enable AVR4 instructions
    parameter ENA_SPM=0,   // AVR4 Store Program Memory
    parameter SP_W=8,      // SP register width (8:16)
    parameter RAM_ADR_W=8, // RAM address width (8:16)
    parameter RESET_JUMP=0,// Address of the reset vector
    parameter ENA_DEBUG=0, // Enable debug interface
    parameter IRQ_LINES=2) // Number of IRQ lines
   (
    // Clock and reset
    input clk_i, 
    input ena_i, 
    input rst_i, 
    // Program memory
    output [15:0] pc_o,
    input  [15:0] inst_i,
    output [15:0] inst_o,
    output        pgm_we_o,
    // I/O control
    output [5:0] io_adr_o,
    output       io_re_o,
    output       io_we_o,
    input  [7:0] io_data_i,
    // Data memory control
    output [RAM_ADR_W-1:0] ram_adr_o, 
    output                 ram_re_o,
    output                 ram_we_o,
    input  [7:0]           ram_data_i,
    // Shared between I/O and RAM
    output [7:0] data_o,
    // I/O register file interface
    output [7:0]      sreg_o,
    input  [7:0]      sreg_i,    // Current SREG
    output [7:0]      sreg_we_o, // Flags write enable
    input  [SP_W-1:0] sp_i,      // Stack Pointer
    output            sp_pop_o,  // Direction of changing of stack pointer 0->up(+) 1->down(-)
    output            sp_we_o,   // Write enable(count enable) for SP
    input  [7:0]      rampz_i,
    // Interrupt
    input  [IRQ_LINES-1:0] irq_lines_i,
    output [IRQ_LINES-1:0] irq_acks_o, 
    // Sleep
    output sleep_o, 
    output irq_ok_o, 
    // Watchdog
    output wdr_o, 
    // Debug
    input         dbg_stop_i,    // Stop request
    output [15:0] dbg_pc_o,
    output [15:0] dbg_inst_o, 
    output [15:0] dbg_inst2_o, 
    output        dbg_exec_o,
    output        dbg_is32_o,
    output        dbg_stopped_o, // CPU is stopped
    // Debug used for Test_ALU_1_TB
    input         dbg_rf_fake_i,
    input  [7:0]  dbg_rr_data_i,
    input  [7:0]  dbg_rd_data_i,
    output [7:0]  dbg_rd_data_o,
    output        dbg_rd_we_o,
    output        dbg_cyc_last_o); // Last cycle in the instruction

/////////////////////
// Program counter //
/////////////////////
// Registers
reg [7:0] pc_l_r;
reg [7:0] pc_h_r;
// The whole PC
wire [15:0] pc;
// Future value
wire [15:0] pc_next;
wire [7:0]  pc_l_next;
wire [7:0]  pc_h_next;
// Sources
wire do_pc_inc;
wire do_pc_imm1;
wire do_pc_imm2;
wire do_pc_z;
wire do_pc_irq;
wire do_pc_pl;
wire do_pc_ph;
wire do_pc_def;
// Instruction Register
reg  [15:0] inst_r;
wire [15:0] inst_cur;    // Current instruction under execution
wire [15:0] inst_stop;   // inst_i at the moment we stopped
reg  [15:0] inst_stop_r; // Register to remmember inst_i when we stop
wire [15:0] inst;        // Used to disable the decoder when we have an interrupt
wire        ir_we;
/////////////////////////
// Instruction Decoder //
/////////////////////////
wire idc_add;            // ADD/ADC
wire idc_add_wc_r;       // With Carry
wire idc_xxiw;           // ADIW/SBIW
wire idc_is_sub_r;       // 1 for SBIW, '0' for ADIW
wire idc_ando;           // AND
wire idc_andi;           // ANDI
wire idc_asr;            // ASR
wire idc_bc_bs;          // BCLR/BSET
wire idc_bclr;           // Clear
wire idc_bld;            // BLD
wire idc_brbx;           // BRBC/BRBS
wire idc_brbc;           // Is Clear
wire idc_bst;            // BST
wire idc_sbix;           // SBIC/SBIS
wire idc_sbrx;           // SBRC/SBRS
wire idc_xbi;            // CBI/SBI
wire idc_set_r;          // Set (CBI/SBI/SBIC/SBIS/SBRC/SBRS)
wire idc_com;            // COM
wire idc_cp;             // CP
wire idc_cpi;            // CPI
wire idc_cpse;           // CPSE
wire idc_dec;            // DEC
wire idc_lpm;            // ELPM/LPM
wire idc_elpm_r;         // Extended LPM
wire idc_spm;            // SPM
wire idc_eor;            // EOR
wire idc_icall;          // ICALL
wire idc_ijmp;           // IJMP
wire idc_ino;            // IN
wire idc_inc;            // INC
wire idc_ld;             // LD Rd,REG REG=X/X+/-X/Y+/-Y/Z+/-Z
wire [1:0] idc_ld_reg;   // 11=X 10=Y 00=Z
wire [1:0] idc_ld_op_r;  // 00 nop 01 X+ 10 -X
wire idc_ldd;            // LDD Rd,Y+q/Z+q
wire idc_ldd_y;          // Y variant (same for STD)
wire [5:0] idc_q_r;      // registered q offset
wire idc_ldi;            // LDI
wire idc_lds;            // LDS
wire idc_lsr;            // LSR
wire idc_mov;            // MOV
wire idc_neg;            // NEG
wire idc_oro;            // OR
wire idc_ori;            // ORI
wire idc_outo;           // OUT
wire idc_pop;            // POP
wire idc_push;           // PUSH
wire idc_rcall;          // RCALL
wire idc_ret;            // RET
wire idc_reti_r;         // RETI variant
wire idc_rjmp;           // RJMP
wire idc_roro;           // ROR
wire [1:0] idc_shf_op_r; // ASR,ASR,LSR,ROR
wire idc_sleep;          // SLEEP
wire idc_st;             // ST REG,Rr REG=X/X+/-X/Y+/-Y/Z+/-Z
wire idc_stdo;           // STD Y+q/Z+q,Rr
wire idc_sts;            // STS
wire idc_subo;           // SUB
wire idc_subic;          // SUBI/SBCI
wire idc_sub_nc;         // No Carry (SBC/SBCI/CPC)
wire idc_swap;           // SWAP
wire idc_wdr;            // WDR
// AVR3 instructions
wire idc_call;           // CALL
wire idc_jmp;            // JMP
// AVR4 instructions
wire idc_mul;            // MUL   (unsigned*unsigned) [8*8=16]
wire idc_muls;           // MULS  (signed*signed)
wire idc_mulsu;          // MULSU (signed*unsigned)
wire idc_fmul;           // FMUL   (unsigned*unsigned) [1.7*1.7=2.14]
wire idc_fmuls;          // FMULS  (signed*signed)
wire idc_fmulsu;         // FMULSU (signed*unsigned)
wire idc_movw;           // MOVW

// Multicycle FSM
reg  cyc_1_r;
wire cyc_1_next;
wire cyc_fetch_next;   // PC changed, wait for a fetch
wire cyc_2_rjmp_next;  // RJMP
reg  cyc_2_rcall_r;    // RCALL k (PUSH PC High)
wire cyc_2_rcall_next; // RCALL
reg  cyc_2_icall_r;    // ICALL (PUSH PC High)
wire cyc_2_icall_next; // ICALL
reg  cyc_2_ijmp_r;     // IJMP (Write PC)
wire cyc_2_ijmp_next;  // IJMP
wire cyc_1_brbx;       // BRBC/BRBS k
wire cyc_2_brbx_next;  // BRBC/BRBS k
reg  cyc_2_ret_r;      // RET/RETI (2 Read PCL)
wire cyc_2_ret_next;   // RET/RETI
reg  cyc_3_ret_r;      // RET/RETI (3 Read PCH)
wire cyc_3_ret_next;   // RET/RETI
reg  cyc_4_ret_r;      // RET/RETI (4 Fetch)
wire cyc_4_ret_next;   // RET/RETI
reg  cyc_2_out_r;      // OUT A,Rr (Write to I/O space from mem)
wire cyc_2_out_next;   // OUT A,Rr
reg  cyc_2_in_r;       // IN Rd,A (Write to Rd from I/O space from mem) TODO: From ALU?
wire cyc_2_in_next;    // IN Rd,A
reg  cyc_2_wioa_r;     // SBI/CBI A,b (Write to I/O space from ALU)
wire cyc_2_wioa_next;  // SBI/CBI A,b
reg  cyc_2_sbix_r;     // SBIS/SBIC A,b (2 Check condition)
wire cyc_2_sbix_next;  // SBIS/SBIC A,b
reg  cyc_2_cpse_r;     // CPSE Rd,Rr (2 Check condition)
wire cyc_2_cpse_next;  // CPSE Rd,Rr
reg  cyc_2_sbrx_r;     // SBRC/SBRS Rr,b (2 Check condition)
wire cyc_2_sbrx_next;  // SBRC/SBRS Rr,b
reg  cyc_3_skip_r;     // SBIS/SBIC A,b/CPSE (3 Skip 1 word)
wire cyc_3_skip_next;  // SBIS/SBIC A,b/CPSE
wire cyc_4_skip_next;  // SBIS/SBIC A,b/CPSE
reg  cyc_2_sts_r;      // STS k,Rr (Write to full address space)
wire cyc_2_sts_next;   // STS k,Rr
reg  cyc_2_st_r;       // ST X,Rr (Write to full address space)
wire cyc_2_st_next;    // ST X,Rr
reg  cyc_3_st_r;       // ST -X/X+,Rr (Update pointer) Only when inc/dec
wire cyc_3_st_next;    // ST -X/X+,Rr
reg  cyc_2_std_r;      // STD Y+q/Z+q,Rr (Write to full address space+q)
wire cyc_2_std_next;   // STD Y+q/Z+q,Rr
reg  cyc_2_push_r;     // PUSH Rr (Write to full address space)
wire cyc_2_push_next;  // PUSH Rr
reg  cyc_2_ld_r;       // LD Rd,X/Y/Z (Read from full address space)
wire cyc_2_ld_next;    // LD Rd,X/Y/Z
reg  cyc_3_ld_r;       // LD Rd,X/Y/Z (Write to Rd from memory)
wire cyc_3_ld_next;    // LD Rd,X/Y/Z
reg  cyc_2_ldd_r;      // LDD Rd,Y+q/Z+q (Read from full address space+q)
wire cyc_2_ldd_next;   // LDD Rd,Y+q/Z+q
reg  cyc_2_lds_r;      // LDS Rd,k (Read from full address space)
wire cyc_2_lds_next;   // LDS Rd,k
reg  cyc_2_xxiw_r;     // ADIW/SBIW (2 Add+Write Rd)
wire cyc_2_xxiw_next;  // ADIW/SBIW (2 Add+Write Rd)
reg  cyc_3_xxiw_r;     // ADIW/SBIW (3 Read Rd+1)
wire cyc_3_xxiw_next;  // ADIW/SBIW (3 Read Rd+1)
reg  cyc_4_xxiw_r;     // ADIW/SBIW (4 Write Rd+1)
wire cyc_4_xxiw_next;  // ADIW/SBIW (4 Write Rd+1)
reg  cyc_2_add_r;      // ADD/ADC
reg  cyc_2_sub_r;      // SUB/SBC/CP/CPC/SUBI/SBCI/CPI/NEG/DEC/CPSE
reg  cyc_2_sbc_r;      // SBC/CPC/SBCI
reg  cyc_2_and_r;      // AND
reg  cyc_2_or_r;       // OR
reg  cyc_2_eor_r;      // EOR
reg  cyc_2_com_r;      // COM
reg  cyc_2_neg_r;      // NEG
reg  cyc_2_inc_dec_r;  // INC/DEC
reg  cyc_2_shf_r;      // ASR/LSR/ROR
reg  cyc_2_swap_r;     // SWAP
reg  cyc_2_mov_r;      // MOV
wire cyc_2_movw_next;  // MOVW
reg  cyc_2_movw_r;     // MOVW
reg  cyc_2_bld_clr_r;  // BLD T=0
reg  cyc_2_bld_set_r;  // BLD T=1
reg  cyc_2_bld_r;      // BLD T=x
wire cyc_2_bld_next;   // BLD T=x
reg  cyc_2_walu_r;     // ALU Write (ADD/ADC/SUB/SBC/AND/OR/EOR)+(SUBI/SBCI/ANDI/ORI)+(LSR/ASR/ROR)+(COM/NEG/SWAP/MOV)+BLD
wire cyc_2_walu_next;  // ALU Write (ADD/ADC/SUB/SBC/AND/OR/EOR)+(SUBI/SBCI/ANDI/ORI)+(LSR/ASR/ROR)+(COM/NEG/SWAP/MOV)+BLD
reg  cyc_2_alu_r;      // ALU (CP/CPC/CPSE)+ all the ALU write stuff
wire cyc_2_alu_next;   // ALU (CP/CPC/CPSE)+ all the ALU write stuff
reg  cyc_2_ialu_r;     // ALU Immediate (SUBI/SBCI/ANDI/ORI)
wire cyc_2_ialu_next;  // ALU Immediate (SUBI/SBCI/ANDI/ORI)
reg  cyc_2_bst_r;      // BST (2 write SREG)
wire cyc_2_bst_next;   // BST (2 write SREG)
reg  cyc_2_lpm_r;      // LPM (2 read PROM)
wire cyc_2_lpm_next;   // LPM (2 read PROM)
reg  cyc_3_lpm_r;      // LPM (3 write R0)
wire cyc_3_lpm_next;   // LPM (3 write R0)
reg  cyc_2_spm_r;      // SPM (2 read Z)
wire cyc_2_spm_next;   // SPM (2 read Z)
reg  cyc_3_spm_r;      // SPM (3 write PROM)
wire cyc_3_spm_next;   // SPM (3 write PROM)
wire cyc_1_bc_bs;      // BCLR & BSET (1 Read/Modify/Write)
wire cyc_1_bclr;       // BCLR (1 Read/Modify/Write)
wire cyc_1_bset;       // BCLR (1 Read/Modify/Write)
wire cyc_2_pop_next;   // POP Rd (1 Read [SP+1])
wire cyc_1_ldi;        // LDI Rd,k
// Rd address
wire do_adr_x;         // Address X reg
wire do_adr_y;         // Address Y reg
wire do_adr_z;         // Address Z reg
wire do_adr_rhi;       // Address r16 to r31
wire do_adr_mem;       // Address from memory map
wire do_adr_iw0;       // Address low xxIW reg
wire do_adr_iw1;       // Address high xxIW reg
wire do_adr_r0;        // Address R0
wire do_adr_rr0;       // Address Rr&'0'
wire do_adr_rd0;       // Address Rd&'0'
wire do_adr_rd;        // Address Rd
// Rr address
wire do_adr2_mem;      // Address from memory map
wire do_adr2_rd;       // Address Rd
wire do_adr2_rr;       // Address Rr
// Rd data write source
wire rd_dat_alu;       // From ALU
wire rd_dat_imm;       // Immediate
wire rd_dat_pr0;       // From ROM, even address
wire rd_dat_pr1;       // From ROM, odd address
// Memory address source
wire fadr_imm;         // Immediate
wire fadr_p_q;         // X/Y/Z Pointer + q(imm)
wire fadr_p;           // X/Y/Z Pointer (optionally -1)
// I/O address sources
wire io_adr_imm1;      // Immediate 1 (IN/OUT)
wire io_adr_imm2;      // Immediate Bit manipulation cyc 1
wire io_adr_mem;       // From RAM
// Data outputs sources
wire data_from_alu;    // From ALU
wire data_from_pcl;    // From PC Low
wire data_from_pch;    // From PC High
// Some actions
wire do_push;          // Use SP as RAM adr, store a value, SP--
wire do_pop;           // Use SP+1 as RAM adr, load a value, SP++
wire do_store;         // Store a value in memory (ST/STD/STS)
wire do_load;          // Load a value from memory (LD/LDD/LDS)
wire do_rd_pointer1;   // Read X/Y/Z LD/ST
wire do_rd_pointer2;   // Read X/Y/Z LDD/STD
wire do_pre_dec;       // Pre-decrement pointer
wire do_post_inc;      // Post-increment pointer
wire do_upd_pointer;   // Pre-decrement/post-increment pointer
// Registers file
// Main 8 bits port R/W (Rd)
wire [4:0] rd_adr;
wire       rd_we;
wire [7:0] rd_write;
wire [7:0] rd_read;
wire [7:0] rd_read_aux;
// Main 16 bits port R/W (Rd)
wire        rd16_we;
wire [15:0] rd16_write;
wire [15:0] rd16_read;
// Secondary 8 bits port R (Rr)
wire [4:0] rr_adr;
wire [7:0] rr_read;
wire [7:0] rr_read_aux;
// Store address range
wire [15:0] full_adr;
wire        full_adr_is_reg;
wire        full_adr_is_ram;
wire        full_adr_is_io;
reg         full_adr_is_ram_r;
reg         full_adr_is_io_r;
// Pointer offset (-X; X+q; X)
wire [15:0] fadr_p_off1;
wire [15:0] fadr_p_off;
// Stack pointer offset
wire [15:0] fadr_sp_off;
wire [15:0] sp16;
// SREG  I T H S V N Z C
wire sreg_if;
wire sreg_i_next; // I flag in the next clock cycle
wire sreg_t;
wire sreg_s;
wire sreg_v;
wire sreg_n;
wire sreg_z;
wire sreg_c;
`define sreg_i_r sreg_i[7] // Interrupt
`define sreg_t_r sreg_i[6] // Test
`define sreg_z_r sreg_i[1] // Zero
`define sreg_c_r sreg_i[0] // Carry
wire [7:0] sreg_out;     // sreg_o read-out
wire [7:0] sreg_we_out;  // sreg_we_o read-out
wire [7:0] sreg_we_out1; // sreg_we_o read-out
wire do_add_sub_flag;
wire do_shift_flag;
wire do_arith_flag;
wire do_alu_flag;
/////////////////////////////////////
// ALU
/////////////////////////////////////
wire [0:0] cin; // Adder/Sub Carry In
// Adder
wire [4:0] add_lo; // Adder Low Nibble
wire [4:0] add_hi; // Adder High Nibble
// Substractor
wire [4:0] sub_lo; // Adder Low Nibble
wire [4:0] sub_hi; // Adder High Nibble
reg  [7:0] bit_mask; // Logic
// ALU Operands
wire [7:0] alu_a_1;
wire [7:0] alu_a;
wire [7:0] alu_b;
// ALU Input flags
wire alu_ci_1;
wire alu_ci;
// ALU Operation (only one active at the time)
wire op_add;
wire op_sub;
wire op_and;
wire op_or;
wire op_xor;
wire op_shf;
wire op_swp;
// ALU Result
wire [7:0] alu_s;
// ALU Flags
wire alu_c;
wire alu_h;
wire alu_z;
wire alu_v;
// ALU misc
wire is_ror;
wire is_asr;
// ALU A sources
wire a_from_mask; // A is a bitmask
wire a_from_rd;   // A is Rd
// ALU B sources
wire b_from_io;   // From I/O
wire b_from_imm2; // Immediate (small)
wire b_from_sreg; // From SREG
wire b_from_imm1; // Immediate (full)
wire b_from_rr;   // From Rr
wire b_from_0;    // Just 0x00
wire b_from_mem;  // From SRAM
// I/O signals
wire       io_re;
reg  [7:0] io_data_r;
wire [6:0] io_map_a; // Memory address translated to I/O space
// X,Y,Z memorized value for ST with pre-dec/post-inc
wire [15:0] cur_ptr;
reg  [15:0] rd16_r;
// Interrupts
wire irq_req; // At least one source requesting
reg [IRQ_ID_W-1:0] irq_vector_adr; // Logic
wire was_irq_next;   // Last operation was interrupted
wire cyc_1_irq_next; // Next instruction will be interrupted
reg  cyc_1_irq_r;    // IRQ (1 Push PC Low)
`define cyc_2_irq_next cyc_1_irq_r
reg  cyc_2_irq_r;    // IRQ (2 Push PC High, ACK, Clear SREG(I), Load PC)
// ELPM's address bit 15 (reg_z_b16&Z)
wire reg_z_b16;
wire lpm_r0;
wire lpm_inc;
// Debug Control
wire enabled;
wire stopped;
reg  was_ena_r;

////////////////////////////////////////////////////////////////////////////
// Program Counter (PC)
////////////////////////////////////////////////////////////////////////////
// Next PC sources
// The most common case (PC increment)
assign do_pc_inc=(cyc_1_next |                         // We are finishing this instruction OR
                  cyc_2_sts_next  | cyc_2_lds_next |   // 32 bits instruction
                  cyc_3_skip_next | cyc_4_skip_next) & // We are skipping the next AND
                  ~cyc_1_irq_next;                     // Won't interrupt
// Immediate 12 bits relative (+/-2k) (RJMP/RCALL)
assign do_pc_imm1=cyc_2_rjmp_next | cyc_2_rcall_r;
// Immediate 7 bits relative (+-63) (BRBC/BRBS)
assign do_pc_imm2=cyc_2_brbx_next;
// Z pointer (IJMP/ICALL)
assign do_pc_z=cyc_2_ijmp_r | cyc_2_icall_r;
// IRQ vector (IRQ)
assign do_pc_irq=cyc_2_irq_r;
// POP low PC (RET/RETI)
assign do_pc_pl=cyc_3_ret_r;
// POP high PC (RET/RETI)
assign do_pc_ph=cyc_2_ret_r;
// Default case (no change)
assign do_pc_def=~(do_pc_inc | do_pc_imm1 | do_pc_imm2 | do_pc_z | do_pc_irq | do_pc_pl | do_pc_ph);
// Next PC mux
assign pc_next=(pc+1                                  & {16{do_pc_inc}})  |
               (rd16_read                             & {16{do_pc_z}})    |
               (irq_vector_adr                        & {16{do_pc_irq}})  |
               ({pc_h_r,ram_data_i}                   & {16{do_pc_pl}})   |
               ({ram_data_i,pc_l_r}                   & {16{do_pc_ph}})   |
               // PC+K+1 (we already incremented 1)
               (pc+{{4{inst_cur[11]}},inst_cur[11:0]} & {16{do_pc_imm1}}) |
               (pc+{{9{inst_cur[ 9]}},inst_cur[9:3]}  & {16{do_pc_imm2}}) |
               // Default is to retain
               ({pc_h_r,pc_l_r}                       & {16{do_pc_def}});
// PC registers
assign pc_h_next=pc_next[15:8];
assign pc_l_next=pc_next[ 7:0];
always @(posedge clk_i)
begin : do_pc
  if (rst_i)
     begin
     pc_l_r <= RESET_JUMP[7:0];
     pc_h_r <= RESET_JUMP[15:8];
     end
  else if (enabled)
     begin
     pc_l_r <= pc_l_next;
     pc_h_r <= pc_h_next;
     end
end // do_pc
// 16 bits PC
assign pc={pc_h_r,pc_l_r};
// Mux for LPM, we don't change PC, just use Z
assign pc_o=cyc_2_lpm_r | cyc_3_spm_r ? {reg_z_b16,rd16_read[15:1]} : pc; // LPM
// rampz_i[0] for ELPM, 0 for LPM
assign reg_z_b16=ENA_AVR3 ? rampz_i[0] & idc_elpm_r : 0;
// SPM's inst_o
generate
if (ENA_SPM)
   begin : inst_o_implemented
   reg [15:0] insto_r;
   always @(posedge clk_i)
   begin : do_inst_o
     if (rst_i)
        insto_r <= 0;
     else if (cyc_2_spm_r)
        insto_r <= rd16_read;
   end // do_inst_o
   assign pgm_we_o=cyc_3_spm_r;
   assign inst_o=insto_r;
   end // inst_o_implemented;
else
   begin : inst_o_not_implemented
   assign inst_o=0;
   assign pgm_we_o=0;
   end // inst_o_not_implemented
endgenerate

////////////////////////////////////////////////////////////////////////////
// Instruction Register
////////////////////////////////////////////////////////////////////////////
assign ir_we=enabled & cyc_1_r;
always @(posedge clk_i)
begin : do_ir
  if (rst_i)
     inst_r <= 0; // NOP
  else if (ir_we)
     inst_r <= inst_stop;
end // do_ir
assign inst_cur=cyc_1_r ? inst_stop : inst_r;
assign dbg_inst_o=inst_cur;

////////////////////////////////////////////////////////////////////////////
// Instruction Decoder
////////////////////////////////////////////////////////////////////////////
// If we will serve an interrupt we must disable the decoder
assign inst=cyc_1_irq_r ? 0 : inst_stop;
begin : InstDec
  // ADD and ADC are similar, bit 12 indicates the carry
  assign idc_add={inst[15:13],inst[11:10]}==5'b00011; // 000C11XXXXXXXXXX
  assign idc_add_wc_r=inst_r[12]; // With Carry
  // ADIW and SBIW are similar, bit 8 indicates sub
  assign idc_xxiw=inst[15:9]==7'b1001011; // 1001011sXXXXXXXX
  assign idc_is_sub_r=inst_r[8];
  // SUB and SBC are similar, bit 12 is not carry
  assign idc_subo={inst[15:13],inst[11:10]}==5'b00010; // 000c10XXXXXXXXXX
  // SUBI and SBCI are similar, bit 12 is not carry (same as SUB/SBC)
  assign idc_subic=inst[15:13]==3'b010; // 010cXXXXXXXXXXXX
  assign idc_sub_nc=inst[12]; // No Carry

  assign idc_ando=inst[15:10]==6'b001000; // 001000XXXXXXXXXX
  assign idc_andi=inst[15:12]==4'b0111;   // 0111XXXXXXXXXXXX
  // Shift operations TODO: 1 OP
  assign idc_asr ={inst[15:9],inst[3:1]}==10'b1001010010;  // 1001010XXXXX010_
  assign idc_roro={inst[15:9],inst[3:0]}==11'b10010100111; // 1001010XXXXX0111
  assign idc_lsr ={inst[15:9],inst[3:0]}==11'b10010100110; // 1001010XXXXX0110
  assign idc_shf_op_r=inst_r[1:0]; // 0x ASR, 10 LSR, 11 ROR
  // BCLR and BSET are similar, bit 7 indicates clear
  assign idc_bc_bs={inst[15:8],inst[3:0]}==12'b100101001000; // 10010100CXXX1000
  assign idc_bclr=inst[7]; // Clear
  assign idc_bld =inst[15:9]==7'b1111100; // 1111100XXXXX_XXX
  // BRBC and BRBS are similar, bit 10 indicates if clear
  assign idc_brbx=inst[15:11]==5'b11110; // 11110CXXXXXXXXXX
  assign idc_brbc=inst[10]; // Is Clear

  assign idc_bst =inst[15:9]==7'b1111101; // 1111101XXXXX_XXX

  assign idc_com ={inst[15:9],inst[3:0]}==11'b10010100000; // 1001010XXXXX0000
  // CP and CPC are similar, bit 12 is no carry
  assign idc_cp  ={inst[15:13],inst[11:10]}==5'b00001; // 000c01XXXXXXXXXX
  // CPI is coherent with CP: bit 12==1
  assign idc_cpi =inst[15:12]==4'b0011; // 0011XXXXXXXXXXXX
  assign idc_cpse=inst[15:10]==6'b000100; // 000100XXXXXXXXXX
  assign idc_lpm=inst==16'b1001010111001000 || // LPM
                 inst==16'b1001010111011000 || // ELPM
                 //inst==16'b1001010111101000 || // LPM image
                 //inst==16'b1001010111111000 || // ELPM image
                 ({inst[15:10],inst[3:1]}==9'b100100010 && (ENA_AVR4 || ENA_AVR25)); // LPM Rd,Z[+]
  assign idc_spm =inst==16'b1001010111101000; // SPM 0x95E8 AV4
  assign idc_elpm_r=inst_r[4]; // Extended LPM
  assign idc_eor =inst[15:10]==6'b001001; // 001001XXXXXXXXXX
  assign idc_ino =inst[15:11]==5'b10110; // 10110XXXXXXXXXXX
  assign idc_inc ={inst[15:9],inst[3:0]}==11'b10010100011; // 1001010XXXXX0011
  assign idc_dec ={inst[15:9],inst[3:1]}==10'b1001010101; // 1001010XXXXX101_

  // LD Rd,X; LD Rd,X+; LD Rd,-X
  //          LD Rd,Y+; LD Rd,-Y
  //          LD Rd,Z+; LD Rd,-Z
  // Note: LD Rd,Y and LD Rd,Z are just LDD Rd,Y+q and LDD Rd,Z+q with q=0
  // 1001 000d dddd RRoo
  // Note: LDS and POP shares the same decoding space
  assign idc_ld=inst[15:9]==7'b1001000 &&
                (inst[3:0]==4'b0001 || // Z+
                 inst[3:0]==4'b0010 || // -Z
                 //inst[3:0]==4'b0011 || // not used
                 //inst[3:0]==4'b1000 || // not used
                 inst[3:0]==4'b1001 || // Y+
                 inst[3:0]==4'b1010 || // -Y
                 //inst[3:0]==4'b1011 || // not used
                 inst[3:0]==4'b1100 || // X
                 inst[3:0]==4'b1101 || // X+
                 inst[3:0]==4'b1110)  // -X
                     ;
  // inst[3:2] 11=X 10=Y 00=Z
  assign idc_ld_reg=inst[3:2];
  // inst[1:0] 00 nop 01 X+ 10 -X
  assign idc_ld_op_r=inst_r[1:0];

  // ST X,Rr; ST X+,Rr; ST -X,Rr
  //          ST Y+,Rr; ST -Y,Rr
  //          ST Z+,Rr; ST -Z,Rr
  // Note: ST Y,Rr and ST Z,Rr are just STD Rd,Y+q and STD Rd,Z+q with q=0
  // 1001 001r rrrr RRoo
  // Note: STS and PUSH shares the same decoding space
  assign idc_st=inst[15:9]==7'b1001001 &&
                (inst[3:0]==4'b0001 || // Z+
                 inst[3:0]==4'b0010 || // -Z
                 //inst[3:0]==4'b0011 || // not used
                 //inst[3:2]==2'b01   || // not used
                 //inst[3:0]==4'b1000 || // not used
                 inst[3:0]==4'b1001 || // Y+
                 inst[3:0]==4'b1010 || // -Y
                 //inst[3:0]==4'b1011 || // not used
                 inst[3:0]==4'b1100 || // X
                 inst[3:0]==4'b1101 || // X+
                 inst[3:0]==4'b1110)   // -X
              ;
  // ld_reg, ld_reg_r and ld_op_r are the same here

  // LDD Rd,Y+q; LDD Rd,Z+q
  // q=0 implements LD Rd,Y and LD Rd,Z
  // 10q0qq0dddddRqqq R: 1 Y, 0 Z
  assign idc_ldd  ={inst[15:14],inst[12],inst[9]}==4'b1000;
  assign idc_ldd_y=inst[3];
  assign idc_q_r  ={inst_r[13],inst_r[11:10],inst_r[2:0]};
  // STD Y+q,Rr; STD Z+q,Rr
  // q=0 implements STD Y,Rr and STD Z,Rr
  // 10q0qq1dddddRqqq R: 1 Y, 0 Z
  assign idc_stdo={inst[15:14],inst[12],inst[9]}==4'b1001; // 10X0XX1XXXXX1XXX

  assign idc_ldi =inst[15:12]==4'b1110; // 1110XXXXXXXXXXXX
  assign idc_lds ={inst[15:9],inst[3:0]}==11'b10010000000; // 1001000XXXXX0000
  assign idc_mov =inst[15:10]==6'b001011; // 001011XXXXXXXXXX
  // NEG is coherent with SUB: bit[12]=1 => No carry
  assign idc_neg ={inst[15:9],inst[3:0]}==11'b10010100001; // 1001010XXXXX0001
  assign idc_oro =inst[15:10]==6'b001010; // 001010XXXXXXXXXX
  assign idc_ori =inst[15:12]==4'b0110; // 0110XXXXXXXXXXXX
  assign idc_outo=inst[15:11]==5'b10111; // 10111XXXXXXXXXXX
  assign idc_pop ={inst[15:9],inst[3:0]}==11'b10010001111; // 1001000XXXXX1111
  assign idc_push={inst[15:9],inst[3:0]}==11'b10010011111; // 1001001XXXXX1111
  // RET and RETI are similar, bit 4 indicates "from interrupt"
  assign idc_ret ={inst[15:7],inst[3:0]}==13'b1001010101000; // 100101010XXi1000
  assign idc_reti_r=inst_r[4];

  assign idc_rjmp =inst[15:12]==4'b1100; // 1100XXXXXXXXXXXX
  assign idc_rcall=inst[15:12]==4'b1101; // 1101XXXXXXXXXXXX
  assign idc_ijmp ={inst[15:8],inst[3:0]}==12'b100101001001; // 10010100XXXX1001
  assign idc_icall={inst[15:8],inst[3:0]}==12'b100101011001; // 10010101XXXX1001

  // CBI and SBI are similar, bit 9 indicates if set
  assign idc_xbi ={inst[15:10],inst[8]}==7'b1001100; // 100110S0XXXXXXXX
  // SBIC and SBIS are similar, bit 9 indicates if set
  assign idc_sbix={inst[15:10],inst[8]}==7'b1001101; // 100110S1XXXXXXXX
  // SBRC and SBRS are similar, bit 9 indicates if set
  assign idc_sbrx=inst[15:10]==6'b111111; // 111111SXXXXXXXXX
  assign idc_set_r=inst_r[9]; // Set

  assign idc_sleep={inst[15:5],inst[3:0]}==15'b100101011001000; // 10010101100X1000

  assign idc_sts ={inst[15:9],inst[3:0]}==11'b10010010000; // 1001001XXXXX0000
  assign idc_swap={inst[15:9],inst[3:0]}==11'b10010100010; // 1001010XXXXX0010
  assign idc_wdr ={inst[15:5],inst[3:0]}==15'b100101011011000; // 10010101101X1000
  ///////////////////////
  // AVR3 instructions //
  ///////////////////////
  assign idc_call={inst[15:9],inst[3:1]}==10'b1001010111 && ENA_AVR3; // 1001010XXXXX111X
  assign idc_jmp ={inst[15:9],inst[3:1]}==10'b1001010110 && ENA_AVR3; // 1001010XXXXX110X 0x94xC-0x95xD
  ///////////////////////
  // AVR4 instructions //
  ///////////////////////
  assign idc_mul   =inst[15:10]==6'b100111   && ENA_AVR4;  // 100111RDDDDDRRRR
  assign idc_muls  =inst[15:8]==8'b00000010  && ENA_AVR4; // 00000010DDDDRRRR
  assign idc_mulsu =inst[15:7]==9'b000000110 && !inst[3] && ENA_AVR4; // 000000110DDD0RRR
  assign idc_fmul  =inst[15:7]==9'b000000110 &&  inst[3] && ENA_AVR4; // 000000110DDD1RRR
  assign idc_fmuls =inst[15:7]==9'b000000111 && !inst[3] && ENA_AVR4; // 000000111DDD0RRR
  assign idc_fmulsu=inst[15:7]==9'b000000111 &&  inst[3] && ENA_AVR4; // 000000111DDD1RRR
  // AVR4 and AVR25(2.5)
  assign idc_movw  =inst[15:8]==8'b00000001 && (ENA_AVR4 || ENA_AVR25); // 00000001DDDDRRRR
  // LPM Rd,Z decoding is in LPM decoding
  // Most invalid opcodes are decoded as NOP
end // InstDec

////////////////////////////////////////////////////////////////////////////
// Multicycle FSM
////////////////////////////////////////////////////////////////////////////
always @(posedge clk_i)
begin : do_fsm
  if (rst_i)
     begin
     // First cycle for any instruction
     cyc_1_r         <= 0;
     cyc_2_ijmp_r    <= 0;
     cyc_2_rcall_r   <= 0;
     cyc_2_icall_r   <= 0;
     cyc_2_ret_r     <= 0;
     cyc_3_ret_r     <= 0;
     cyc_4_ret_r     <= 0;
     cyc_2_out_r     <= 0;
     cyc_2_in_r      <= 0;
     cyc_2_wioa_r    <= 0;
     cyc_2_sts_r     <= 0;
     cyc_2_st_r      <= 0;
     cyc_3_st_r      <= 0;
     cyc_2_std_r     <= 0;
     cyc_2_push_r    <= 0;
     cyc_2_ld_r      <= 0;
     cyc_3_ld_r      <= 0;
     cyc_2_lds_r     <= 0;
     cyc_2_ldd_r     <= 0;
     cyc_2_sbix_r    <= 0;
     cyc_2_cpse_r    <= 0;
     cyc_2_sbrx_r    <= 0;
     cyc_3_skip_r    <= 0;
     cyc_2_xxiw_r    <= 0;
     cyc_3_xxiw_r    <= 0;
     cyc_4_xxiw_r    <= 0;
     cyc_2_add_r     <= 0;
     cyc_2_sub_r     <= 0;
     cyc_2_sbc_r     <= 0;
     cyc_2_and_r     <= 0;
     cyc_2_or_r      <= 0;
     cyc_2_eor_r     <= 0;
     cyc_2_com_r     <= 0;
     cyc_2_neg_r     <= 0;
     cyc_2_inc_dec_r <= 0;
     cyc_2_shf_r     <= 0;
     cyc_2_swap_r    <= 0;
     cyc_2_mov_r     <= 0;
     cyc_2_movw_r    <= 0;
     cyc_2_walu_r    <= 0;
     cyc_2_alu_r     <= 0;
     cyc_2_ialu_r    <= 0;
     cyc_2_bst_r     <= 0;
     cyc_2_lpm_r     <= 0;
     cyc_3_lpm_r     <= 0;
     cyc_2_spm_r     <= 0;
     cyc_3_spm_r     <= 0;
     cyc_1_irq_r     <= 0;
     cyc_2_irq_r     <= 0;
     end // rst_i
  else if (enabled)
     begin
     cyc_1_r         <= cyc_1_next;
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
     cyc_2_add_r     <= (idc_add | idc_inc) & cyc_1_r;
     cyc_2_sub_r     <= (idc_subo | idc_cp | idc_subic | idc_cpi | idc_neg | idc_dec | idc_cpse) & cyc_1_r;
     cyc_2_sbc_r     <= (idc_subo | idc_cp | idc_subic) & ~idc_sub_nc & cyc_1_r; // SBC/CPC/SBCI
     cyc_2_and_r     <= (idc_ando | idc_andi) & cyc_1_r;
     cyc_2_or_r      <= (idc_oro | idc_ori) & cyc_1_r;
     cyc_2_eor_r     <= idc_eor & cyc_1_r;
     cyc_2_com_r     <= idc_com & cyc_1_r;
     cyc_2_neg_r     <= idc_neg & cyc_1_r; // TODO: needed?
     cyc_2_inc_dec_r <= (idc_inc | idc_dec) & cyc_1_r;
     cyc_2_shf_r     <= (idc_asr | idc_lsr | idc_roro) & cyc_1_r;
     cyc_2_swap_r    <= idc_swap & cyc_1_r;
     cyc_2_mov_r     <= idc_mov & cyc_1_r;
     cyc_2_movw_r    <= cyc_2_movw_next;
     cyc_2_bld_clr_r <= cyc_2_bld_next & ~`sreg_t_r;
     cyc_2_bld_set_r <= cyc_2_bld_next & `sreg_t_r;
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
     cyc_2_irq_r     <= `cyc_2_irq_next;
     end // enabled
end // do_fsm
// Indicates if we are at the last cycle of an instruction
assign cyc_1_next=~(cyc_2_out_next   | cyc_2_sts_next  |
                    cyc_2_ld_next    | cyc_3_ld_next   | cyc_2_lds_next  |
                    cyc_2_st_next    | cyc_2_push_next | cyc_2_wioa_next |
                    cyc_2_sbix_next  | cyc_3_skip_next | cyc_2_movw_next |
                    cyc_2_rcall_next | cyc_2_xxiw_next |
                    cyc_3_xxiw_next  | cyc_4_xxiw_next | cyc_2_ret_next  |
                    cyc_3_ret_next   | cyc_4_ret_next  |
                    cyc_2_alu_next   | cyc_2_ijmp_next |
                    cyc_2_icall_next | cyc_2_sbrx_next | cyc_fetch_next  |
                    cyc_2_bst_next   | cyc_2_in_next   | cyc_3_st_next   |
                    cyc_2_std_next   | cyc_2_lpm_next  | cyc_3_lpm_next  |
                    cyc_2_spm_next   | cyc_3_spm_next  |
                    `cyc_2_irq_next  | cyc_2_ldd_next);
assign dbg_cyc_last_o=cyc_1_next;
// Indicates the instruction finished, but PC changed & we need to wait for a fetch
assign cyc_fetch_next=cyc_2_brbx_next | // BRBC/BRBS w/branch
                      cyc_2_ijmp_r    | // IJMP
                      cyc_2_rjmp_next | // RJMP
                      cyc_4_skip_next | // SBIC/SBIS
                      cyc_2_rcall_r   | // RCALL
                      cyc_2_icall_r   | // ICALL
                      cyc_2_irq_r     | // IRQ
                      cyc_3_lpm_r     | // LPM
                      cyc_3_spm_r;      // SPM
assign cyc_2_rjmp_next =cyc_1_r & idc_rjmp; // RJMP
assign cyc_2_ijmp_next =cyc_1_r & idc_ijmp; // IJMP
assign cyc_2_brbx_next =cyc_1_r & idc_brbx & (alu_z ~^ idc_brbc); // BRBC/BRBS
assign cyc_2_rcall_next=cyc_1_r & idc_rcall; // RCALL
assign cyc_2_icall_next=cyc_1_r & idc_icall; // ICALL
assign cyc_2_ret_next  =cyc_1_r & idc_ret; // RET/RETI
assign cyc_3_ret_next  =cyc_2_ret_r; // RET/RETI
assign cyc_4_ret_next  =cyc_3_ret_r; // RET/RETI
assign cyc_2_out_next  =cyc_1_r & idc_outo; // OUT
assign cyc_2_in_next   =cyc_1_r & idc_ino; // IN
assign cyc_2_wioa_next =cyc_1_r & idc_xbi;  // SBI/CBI
assign cyc_2_sts_next  =cyc_1_r & idc_sts;  // STS
assign cyc_2_st_next   =cyc_1_r & idc_st; // ST X/X+/-X/Y+/-Y/Z+/-Z,Rr
assign cyc_3_st_next   =cyc_2_st_r & do_upd_pointer; // 3rd cycle to update X/Y/Z
assign cyc_2_std_next  =cyc_1_r & idc_stdo; // STD
assign cyc_2_push_next =cyc_1_r & idc_push; // PUSH
assign cyc_2_ld_next   =cyc_1_r & idc_ld; // LD Rd,X/X+/-X/Y+/-Y/Z+/-Z
assign cyc_2_pop_next  =cyc_1_r & idc_pop;
assign cyc_3_ld_next   =do_load | cyc_2_pop_next; // LD/LDS/LDD/POP (Write Rd from memory)
assign cyc_2_lds_next  =cyc_1_r & idc_lds;  // LDS Rd,K
assign cyc_2_ldd_next  =cyc_1_r & idc_ldd; // LDD Rd,Y+q/Z+q
assign cyc_2_sbix_next =cyc_1_r & idc_sbix; // SBIC/SBIS
assign cyc_2_sbrx_next =cyc_1_r & idc_sbrx; // SBRC/SBRS
assign cyc_2_cpse_next =cyc_1_r & idc_cpse; // CPSE
assign cyc_3_skip_next =((cyc_2_sbix_r | cyc_2_sbrx_r) & (alu_z ^ idc_set_r)) | // SBIC/SBIS/SBRC/SBRS
                         (cyc_2_cpse_r & alu_z); // CPSE
assign cyc_4_skip_next =cyc_3_skip_r & (idc_sts | idc_lds); // SBIC/SBIS
assign cyc_2_xxiw_next =cyc_1_r & idc_xxiw; // ADIW/SBIW
assign cyc_3_xxiw_next =cyc_2_xxiw_r; // ADIW/SBIW
assign cyc_4_xxiw_next =cyc_3_xxiw_r; // ADIW/SBIW
// ALU operations that write alu_s to Rd
assign cyc_2_walu_next =(idc_add | idc_subo | idc_ando | idc_oro | idc_eor | // ADD/ADC/SUB/SBC/AND/OR/EOR
                         idc_subic | idc_andi | idc_ori | idc_com | idc_neg |// SUBI/SBCI/ANDI/ORI/COM/NEG
                         idc_inc | idc_dec | idc_asr | idc_lsr | idc_roro |  // INC/DEC/ASR/LSR/ROR
                         idc_swap | idc_mov | idc_bld) // SWAP/MOV/BLD
                         & cyc_1_r;
// ALU operations with immediate operand
assign cyc_2_ialu_next=(idc_subic | idc_andi | idc_ori | idc_cpi) & cyc_1_r;// SUBI/SBCI/ANDI/ORI/CPI
// ALU operations (all)
assign cyc_2_alu_next =((idc_cp | idc_cpi | idc_cpse) & cyc_1_r) // CP/CPC/CPI/CPSE
                       | cyc_2_walu_next; // Also all the ALU Write stuff
assign cyc_2_bst_next =cyc_1_r & idc_bst; // BST
assign cyc_2_lpm_next =cyc_1_r & idc_lpm; // LPM
assign cyc_3_lpm_next =cyc_2_lpm_r;
assign cyc_2_spm_next =ENA_SPM ? cyc_1_r & idc_spm : 0; // SPM
assign cyc_3_spm_next =ENA_SPM ? cyc_2_spm_r : 0;
assign cyc_1_bc_bs    =idc_bc_bs & cyc_1_r; // BSET & BCLR
assign cyc_1_bclr     =cyc_1_bc_bs & idc_bclr; // BCLR
assign cyc_1_bset     =cyc_1_bc_bs & ~idc_bclr; // BSET
assign cyc_1_brbx     =idc_brbx & cyc_1_r; // BRBC/BRBS
assign cyc_1_ldi      =idc_ldi & cyc_1_r; // LDI
assign cyc_2_bld_next =idc_bld & cyc_1_r; // BLD
assign cyc_2_movw_next=idc_movw & cyc_1_r; // MOVW (AVR25 | AVR4)
// Instruction groups
assign do_push       =cyc_2_push_r |                     // PUSH Rr
                      cyc_2_rcall_next | cyc_2_rcall_r | // RCALL K (1&2)
                      cyc_2_icall_next | cyc_2_icall_r | // ICALL K (1&2)
                      cyc_1_irq_r | cyc_2_irq_r;         // IRQ (1&2)
assign do_pop        =cyc_2_pop_next |              // POP Rd
                      cyc_2_ret_next | cyc_2_ret_r; // RET/RETI (1&2)
assign do_store      =cyc_2_st_r | cyc_2_std_r | cyc_2_sts_r | // ST X/Y/Z,Rr; STD Y/Z+q,Rr; STS K,Rr (2 Memory write)
                      cyc_2_push_r; // PUSH Rr
assign do_load       =cyc_2_ld_r | cyc_2_ldd_r | cyc_2_lds_r; // LD Rd,X/Y/Z; LDD Rd,Y/Z+q; LDS Rd,K (2 Memory read)
assign do_pre_dec    =idc_ld_op_r==`LD_PRE_DEC;  // Pre-decrement pointer
assign do_post_inc   =idc_ld_op_r==`LD_POST_INC; // Post-increment pointer
assign do_upd_pointer=idc_ld_op_r==`LD_PRE_DEC || idc_ld_op_r==`LD_POST_INC; // Pre-decrement/post-increment pointer

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// MEMORY SPACE
// 0x0000 to 0x001F Registers
// 0x0020 to 0x005F I/O (maps to I/O 0x00 to 0x3F)
// 0x0060 to 0xFFFF SRAM
// X reg is r26+r27 => 0x1A/0x1B
// Y reg is r28+r29 => 0x1C/0x1D
// Z reg is r30+r31 => 0x1E/0x1F
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// Memory address (RF+I/O+SRAM)
// Store address source
// Immediate
assign fadr_imm=cyc_2_sts_r | cyc_2_lds_r; // STS K,Rr; LDS Rd,K
// X/Y/Z Pointer + q(imm)
assign fadr_p_q=cyc_2_ldd_r | cyc_2_std_r; // LDD/STD Rd,Y+q/Z+q
// X/Y/Z Pointer (LD/ST)
assign fadr_p  =cyc_2_ld_r  | cyc_2_st_r; // LD Rd,X/Y/Z - ST X/Y/Z,Rr

// Pointer offset (-1,0 for LD/ST | Q for LDD/STDD)
assign fadr_p_off1={16{do_pre_dec}}; // LD/ST w/pre-dec vs no pre-dec
assign fadr_p_off =fadr_p_q ? idc_q_r : fadr_p_off1;

// Stack pointer & its offset
assign sp16       =sp_i;
assign fadr_sp_off=do_pop ? 16'd1 : 16'd0;

assign full_adr=(inst_stop            & {16{fadr_imm}}) |
                (sp16+fadr_sp_off     & {16{do_push | do_pop}}) |
                (rd16_read+fadr_p_off & {16{fadr_p  | fadr_p_q}});
assign full_adr_is_reg=full_adr<32;
assign full_adr_is_io =full_adr>=32 && full_adr<96;
assign full_adr_is_ram=full_adr>=96;
always @(posedge clk_i)
begin : full_adr_src
  if (rst_i)
     begin
     full_adr_is_ram_r <= 0;
     full_adr_is_io_r  <= 0;
     end
  else
     begin
     full_adr_is_ram_r <= full_adr_is_ram;
     full_adr_is_io_r  <= full_adr_is_io;
     end
end // full_adr_src

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// Register File
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
RegisterFile RegFile
   (.clk_i(clk_i),
    .rd_adr_i(rd_adr),  .rd_i(rd_write),     .rd_o(rd_read_aux),
    .rd_we_i(rd_we),    .rd16_we_i(rd16_we), .rd16_i(rd16_write),
    .rd16_o(rd16_read), .rr_adr_i(rr_adr),   .rr_o(rr_read_aux));
// Data from RF, can be faked from the debug unit
assign rd_read=dbg_rf_fake_i ? dbg_rd_data_i : rd_read_aux;
assign rr_read=dbg_rf_fake_i ? dbg_rr_data_i : rr_read_aux;
//////////////////////
// Rd data to write //
//////////////////////
// From ALU
assign rd_dat_alu=cyc_2_xxiw_r | cyc_4_xxiw_r |  // ADIW/SBIW Rd,k
                  cyc_2_walu_r |                 // ADD/ADC/SUB/SBC/AND/OR/EOR/LSR/ASR/ROR/COM/NEG/SWAP/MOV
                  (do_store & full_adr_is_reg) | // ST* & destination is a Reg address
                  cyc_3_ld_r |                   // LD Rd,X/Y/Z; LDS Rd,K
                  cyc_2_in_r;                    // IN Rd,A (2 Write Rd)
// Immediate
assign rd_dat_imm=cyc_1_ldi; // LDI Rd,K
// From ROM, even address
assign rd_dat_pr0=cyc_3_lpm_r & ~rd16_r[0]; // LPM Z is even
// From ROM, odd address
assign rd_dat_pr1=cyc_3_lpm_r & rd16_r[0]; // LPM Z is odd
assign rd_write  =(alu_s                      & {8{rd_dat_alu}}) |
                  ({inst_i[11:8],inst_i[3:0]} & {8{rd_dat_imm}}) |
                  ( inst_i[15:8]              & {8{rd_dat_pr1}}) |
                  ( inst_i[ 7:0]              & {8{rd_dat_pr0}});
///////////
// Rd WE //
///////////
assign rd_we=rd_dat_alu | rd_dat_imm | rd_dat_pr1 | rd_dat_pr0;
////////////////
// Rd address //
////////////////
assign do_rd_pointer1=cyc_2_ld_next |  // LD Rd,xx/xx+/-xx (1 Read pointer)
                      cyc_2_st_next |  // ST xx,Rr (1 Read pointer)
                      cyc_2_ld_r |     // LD Rd,xx/xx+/-xx (2 Update pointer)
                      cyc_3_st_r;      // ST xx,Rr (3 Update pointer)
assign do_rd_pointer2=cyc_2_ldd_next | // LDD Rd,xx+q (1 Read pointer)
                      cyc_2_std_next;  // STD xx+q,Rr (1 Read pointer)
assign do_adr_x= inst_cur[3:2]==`LD_X && do_rd_pointer1; // ST/LD
assign do_adr_y=(inst_cur[3:2]==`LD_Y && do_rd_pointer1) || // ST/LD
                (idc_ldd_y  && do_rd_pointer2);             // STD/LDD
assign do_adr_z=(inst_cur[3:2]==`LD_Z && do_rd_pointer1) || // ST/LD
                (~idc_ldd_y && do_rd_pointer2) ||           // STD/LDD
                cyc_2_lpm_next ||                           // LPM (1 Read pointer)
                cyc_2_lpm_r ||                              // LPM (2 Read PROM) we need Zlsb
                cyc_2_spm_r ||                              // SPM (2 Read pointer)
                //cyc_3_spm_r ||                            // SPM (3 Write PROM) we need Zlsb
                cyc_2_icall_next ||                         // ICALL (1 Read pointer)
                cyc_2_ijmp_next;                            // IJMP (1 Read pointer)
// Address r16 to r31
assign do_adr_rhi=cyc_1_ldi | // LDI Rd,K
                  cyc_2_ialu_next | cyc_2_ialu_r; // SUBI/SBCI/ANDI/ORI/CPI
// Address from memory map
assign do_adr_mem=do_store;
// Address low xxIW reg
assign do_adr_iw0=cyc_2_xxiw_next | cyc_2_xxiw_r; // ADIW/SBIW Rd,k (1/2 R/W Rd)
// Address high xxIW reg
assign do_adr_iw1=cyc_3_xxiw_r | cyc_4_xxiw_r;    // ADIW/SBIW Rd,k (3/4 R/W Rd+1)
// Address R0
assign lpm_r0    =!(ENA_AVR25 || ENA_AVR4) || // No Rd |
                  !inst_cur[2];               // Simple version
assign do_adr_r0 =(cyc_3_lpm_r & lpm_r0) | // LPM (3 Write R0)
                   cyc_2_spm_next;         // SPM (1 Read R1:R0)
// Address Rr&'0' (MOVW)
assign do_adr_rr0=cyc_2_movw_next;
// Address Rd&'0' (MOVW)
assign do_adr_rd0=cyc_2_movw_r;
// Address Rd
assign do_adr_rd =~(do_adr_rhi | do_adr_mem | do_adr_iw0 | do_adr_iw1 |
                    do_adr_r0  | do_adr_x   | do_adr_y   | do_adr_z |
                    do_adr_rr0 | do_adr_rd0); // Default
              // ADD/ADC/SUB/SBC/AND/OR/EOR/COM
              // CP/CPC/CPSE
              // BLD
              // cyc_3_ld_r='1' | cyc_2_in_r='1' LD Rd,X/Y/Z (write Rd); IN
              // LPM (3 Write Rd)
assign rd_adr=({1'b1,inst_cur[7:4]}       & {5{do_adr_rhi}}) |
              (full_adr[4:0]              & {5{do_adr_mem}}) |
              (5'd26                      & {5{do_adr_x}})   |
              (5'd28                      & {5{do_adr_y}})   |
              (5'd30                      & {5{do_adr_z}})   |
              ({2'b11,inst_cur[5:4],1'b0} & {5{do_adr_iw0}}) |
              ({2'b11,inst_cur[5:4],1'b1} & {5{do_adr_iw1}}) |
              ({inst_cur[3:0],1'b0}       & {5{do_adr_rr0}}) |
              ({inst_cur[7:4],1'b0}       & {5{do_adr_rd0}}) |
              ( inst_cur[8:4]             & {5{do_adr_rd}});
///////////////////
// Rd 16 bits WE //
///////////////////
assign lpm_inc=inst_cur[0] && (ENA_AVR25 || ENA_AVR4);
assign rd16_we=(cyc_2_ld_r & do_upd_pointer) | // LD -R/R+
                cyc_3_st_r |                   // ST -R/R+
                cyc_2_movw_r |                 // MOVW
               (cyc_2_lpm_r & lpm_inc);        // LPM Rd,Z+
//////////////////////////////
// Rd 16 bits data to write //
//////////////////////////////
always @(posedge clk_i)
begin : do_rd16_reg
  if (rst_i)
     rd16_r <= 0;
  else
     rd16_r <= rd16_read;
end // do_rd16_reg
assign cur_ptr   =cyc_3_st_r ? rd16_r : rd16_read;
assign rd16_write=cyc_2_movw_r ? cur_ptr : (do_pre_dec ? cur_ptr-1 : cur_ptr+1);
////////////////
// Rr address //
////////////////
assign do_adr2_mem=do_load;
//assign do_adr2_rd=((idc_neg | idc_sbrx | idc_bst | idc_bld | idc_st |
//                    idc_push | idc_stdo | idc_sts | idc_outo) & cyc_1_r) |
//                    cyc_2_sts_r | cyc_2_out_r; -- ST X/Y/Z,Rr; PUSH Rr; NEG; STS TODO: esto es para 0-Rd ¿y /Rd+1? ¿& cyc_1_r?
assign do_adr2_rd=~(do_adr2_rr | do_adr2_mem);
//assign do_adr2_rr=~(do_adr2_rd | do_adr2_mem);
assign do_adr2_rr=cyc_2_alu_next & // (CP/CPC/CPSE)+(ADD/ADC/SUB/SBC/AND/OR/EOR)+(SUBI/SBCI/ANDI/ORI)+(LSR/ASR/ROR)+(COM/NEG/SWAP/MOV)+BLD
                  ~idc_neg &       // !NEG
                  ~idc_bld;        // !BLD
assign rr_adr=(full_adr[4:0]               & {5{do_adr2_mem}}) |
              (inst_cur[8:4]               & {5{do_adr2_rd}})  |
              ({inst_cur[9],inst_cur[3:0]} & {5{do_adr2_rr}});
           // Note: using inst_cur for do_adr2_rr saves a couple of LUTs,
           // but the final result is worst (more used in routing).

////////////////////////////////////////////////////////////////////////////
// I/O Control
////////////////////////////////////////////////////////////////////////////
/////////////////
// I/O Address //
/////////////////
assign io_map_a=full_adr[6:0]-7'd32;
// Immediate 1 (IN/OUT)
assign io_adr_imm1=cyc_2_out_r | cyc_2_in_next; // OUT A,Rr (cyc 2) - IN Rd,A (cyc 1)
// Immediate 2 (Bit manipulation)
assign io_adr_imm2=cyc_2_wioa_next | // SBI/CBI (1 Read)
                   cyc_2_sbix_next | // SBIS/SBIC (1 Read)
                   cyc_2_wioa_r;     // SBI/CBI (2 Write)
// From memory address
assign io_adr_mem=do_store | do_load; // ST*; LD*
assign io_adr_o  =({inst_cur[10:9],inst_cur[3:0]} & {6{io_adr_imm1}}) |
                  ({1'b0,inst_cur[7:3]}           & {6{io_adr_imm2}}) |
                  (io_map_a[5:0]                  & {6{io_adr_mem}});
////////////
// I/O WE //
////////////
assign io_we_o=cyc_2_out_r |  // OUT A,Rr (2 Write)
               cyc_2_wioa_r | // SBI/CBI (2 Write)
               (do_store & full_adr_is_io); // ST* (2 Write)
////////////
// I/O RE //
////////////
assign io_re=cyc_2_wioa_next | // SBI/CBI (1 Read)
             cyc_2_sbix_next | // SBIS/SBIC (1 Read)
             cyc_2_in_next |   // IN (1 Read)
             (do_load & full_adr_is_io); // LD* (2 Read)
assign io_re_o=io_re;
/////////////////////
// I/O & SRAM data //
/////////////////////
// From PC Low
assign data_from_pcl=cyc_2_rcall_next | cyc_2_icall_next | cyc_1_irq_r; // xCALL/IRQ (1 push PC low)
// From PC High
assign data_from_pch=cyc_2_rcall_r | cyc_2_icall_r | cyc_2_irq_r; // xCALL/IRQ (2 push PC high)
// From ALU (Almost all is routed thru the ALU)
assign data_from_alu=~(data_from_pcl | data_from_pch);
//assign data_from_alu=do_store | cyc_2_out_r | cyc_2_wioa_r; -- ST*; PUSH Rr (2 Write); OUT (2 Write); SBI/CBI (2 Write)
assign data_o=(pc_l_r & {8{data_from_pcl}}) |
              (pc_h_r & {8{data_from_pch}}) |
              (alu_s  & {8{data_from_alu}});

always @(posedge clk_i)
begin : io_register
  if (rst_i)
     io_data_r <= 0;
  else if (io_re)
     io_data_r <= io_data_i;
end // io_register
////////////////////////////////////////////////////////////////////////////
// Data memory control
////////////////////////////////////////////////////////////////////////////
assign ram_adr_o=full_adr[RAM_ADR_W-1:0];
assign ram_we_o =(do_store | do_push) & full_adr_is_ram; // ST*;PUSH;xCALL;IRQ
assign ram_re_o =(do_load  | do_pop)  & full_adr_is_ram; // LD*;POP;RET*

////////////////////////////////////////////////////////////////////////////
// Registers implemented in the I/O space
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// Status Register
////////////////////////////////////////////////////////////////////////////
assign sreg_out=cyc_1_bc_bs ? alu_s : {sreg_if,sreg_t,alu_h,sreg_s,sreg_v,sreg_n,sreg_z,sreg_c};
assign sreg_o  =sreg_out;
// Flags computed here:
assign sreg_s  =sreg_n ^ sreg_v;
assign sreg_n  =alu_s[7];
assign sreg_v  =((sreg_n ^ alu_c) & cyc_2_shf_r) |
                 (alu_v           & ~(cyc_2_and_r | cyc_2_or_r | cyc_2_eor_r | cyc_2_com_r | cyc_2_shf_r));
assign sreg_if =~cyc_2_irq_r; // IRQ => 0; RETI => 1
assign sreg_c  =cyc_2_com_r ? 1 : alu_c;
assign sreg_t  =~alu_z; // BST
// ADIW & SUBIW computes the Z of the 16 bits result, so we must check its previous value
// SBC/SBCI/CPC also computes Z using its previous result
assign sreg_z  =!cyc_4_xxiw_r && !cyc_2_sbc_r ? alu_z : alu_z & `sreg_z_r;
// Which flags are altered
assign do_add_sub_flag=(cyc_2_add_r | cyc_2_sub_r) & ~cyc_2_cpse_r & ~cyc_2_inc_dec_r; // ADD/ADC/SUB/SBC/CP/CPC/SUBI/SBCI/NEG (& not inc/dec)
assign do_shift_flag  =(cyc_2_xxiw_r | cyc_4_xxiw_r | cyc_2_com_r | cyc_2_shf_r); // ADIW/SBIW/COM/ASR/LSR/ROR
assign do_arith_flag  =do_shift_flag | do_add_sub_flag;
assign do_alu_flag    =do_arith_flag | cyc_2_and_r | cyc_2_or_r | cyc_2_eor_r | cyc_2_inc_dec_r; // AND/OR/EOR/ANDI/ORI/INC/DEC
// I
assign sreg_we_out1[7]=((cyc_3_ret_r & idc_reti_r) | cyc_2_irq_r); // RETI/IRQ
// T
assign sreg_we_out1[6]=cyc_2_bst_r;
// H
assign sreg_we_out1[5]=do_add_sub_flag;
// S, V, N, Z
assign sreg_we_out1[4]=do_alu_flag;
assign sreg_we_out1[3]=do_alu_flag;
assign sreg_we_out1[2]=do_alu_flag;
assign sreg_we_out1[1]=do_alu_flag;
// C
assign sreg_we_out1[0]=do_arith_flag;
assign sreg_we_out=cyc_1_bc_bs ? 8'hFF : // BSET/BCLR
                                 sreg_we_out1;
assign sreg_we_o=sreg_we_out;
// Future value for SREG(I)
assign sreg_i_next=~sreg_we_out[7] ? `sreg_i_r : sreg_out[7];
////////////////////////////////////////////////////////////////////////////
// Stack Pointer
////////////////////////////////////////////////////////////////////////////
// Direction of changing of stack pointer 0->up(+) 1->down(-)
assign sp_pop_o=~do_push;
assign sp_we_o =do_push | do_pop;


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// ALU
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
assign cin[0]=alu_ci;
///////////////////////
// Adder: A+B        --
///////////////////////
// Low nibble
assign add_lo={1'b0,alu_a[3:0]}+alu_b[3:0]+cin;
// High nibble
assign add_hi={1'b0,alu_a[7:4]}+alu_b[7:4]+add_lo[4:4];
///////////////////////
// Substractor: A-B  //
///////////////////////
// Low nibble
assign sub_lo={1'b0,alu_a[3:0]}-alu_b[3:0]-cin;
// High nibble
assign sub_hi={1'b0,alu_a[7:4]}-alu_b[7:4]-sub_lo[4:4];

// Result
assign alu_s=({add_hi[3:0],add_lo[3:0]} & {8{op_add}}) |
             ({sub_hi[3:0],sub_lo[3:0]} & {8{op_sub}}) |
             ({alu_ci,alu_a[7:1]}       & {8{op_shf}}) |
             ((alu_a & alu_b)           & {8{op_and}}) |
             ((alu_a |  alu_b)          & {8{op_or}})  |
             ((alu_a ^ alu_b)           & {8{op_xor}}) |
             ({alu_a[3:0],alu_a[7:4]}   & {8{op_swp}});
// Zero flag
assign alu_z=alu_s==8'h00;
// Carry flag
assign alu_c=(add_hi[4] & op_add) | // Cy from the high nibble
             (sub_hi[4] & op_sub) | // Borrow from the high nibble
             (alu_a[0]  & op_shf);
// Half carry flag
assign alu_h=(add_lo[4] & op_add) | // Cy from the low nibble
             (sub_lo[4] & op_sub);   // Borrow from the low nibble
// Overflow flag
assign alu_v=((( alu_a[7] &  alu_b[7] & ~add_hi[3]) | // (-)+(-)=(+)
               (~alu_a[7] & ~alu_b[7] &  add_hi[3]))  // (+)+(+)=(-)
              & op_add) |
             ((( alu_a[7] & ~alu_b[7] & ~sub_hi[3]) | // (-)-(+)=(+)
               (~alu_a[7] &  alu_b[7] &  sub_hi[3]))  // (+)-(-)=(-)
              & op_sub) |
             ((alu_s[7] ^ alu_s[0]) & op_shf);

///////////
// ALU A //
///////////
// Bitmask, Rd | 0. Optional: complemented
// Is a bitmask
assign a_from_mask=cyc_2_wioa_r | // CBI/SBI A,b
                   cyc_2_sbix_r | // SBIS/SBIC (2 Check condition)
                   cyc_2_sbrx_r | // SBRS/SBRC (2 Check condition)
                   cyc_2_bst_r  | // BST Rd,b (2 Write SREG)
                   cyc_2_bld_r  | // BLD
                   cyc_1_brbx   | // BCLR/BSET b
                   cyc_1_bc_bs;   // BRBC/BRBS s,k
// Is Rd
assign a_from_rd=(cyc_2_xxiw_r | cyc_4_xxiw_r | // ADIW/SBIW (2&4)
                  cyc_2_alu_r) // CP/CPC/CPI + ADD/ADC/SUB/SBC/AND/OR/EOR/SUBI/SBCI/ANDI/ORI/COM/NEG/LSR/ASR/ROR/SWAP/MOV
                 & ~cyc_2_neg_r  // !NEG
                 & ~cyc_2_mov_r  // !MOV
                 & ~cyc_2_bld_r; // !BLD
assign alu_a_1=(bit_mask & {8{a_from_mask}}) |
               (rd_read  & {8{a_from_rd}});
       // 0 for: NEG (cyc_2_neg_r) MOV (cyc_2_mov_r) do_store | cyc_2_out_r | cyc_2_in_r (cyc_3_ld_r & full_adr_is_reg_r)
assign alu_a=(cyc_2_wioa_r && !idc_set_r) || // CBI A,b
              cyc_1_bclr ||                  // BCLR b (1 cycle)
              cyc_2_bld_clr_r ||             // BLD w/T=0
              cyc_2_com_r                    // COM Rd
              ? ~alu_a_1 : alu_a_1;

///////////
// ALU B //
///////////
// I/O, Immediate(small/full), SREG, Rr | 0.
assign b_from_io  =cyc_2_wioa_r | cyc_2_sbix_r | cyc_2_in_r | // SBI/CBI A,b; SBIS/SBIC (2 Check condition)
                   (cyc_3_ld_r & full_adr_is_io_r);
assign b_from_imm1=cyc_2_ialu_r; // SUBI/SBCI/ANDI/ORI/CPI
assign b_from_imm2=cyc_2_xxiw_r; // ADIW/SBIW [2]
assign b_from_sreg=cyc_1_bc_bs | cyc_1_brbx; // BCLR/BSET b; BRBC/BRBS s,k
assign b_from_0   =cyc_2_inc_dec_r | cyc_4_xxiw_r | cyc_2_com_r; // INC/DEC (cyc_2_inc_dec_r) ADIW[4] (cyc_4_xxiw_r) COM
assign b_from_mem =cyc_3_ld_r & full_adr_is_ram_r; // LD Rd,X/Y/Z; LDS Rd,K (write Rd from SRAM)
assign b_from_rr  =~(b_from_io | b_from_imm1 | b_from_imm2 | b_from_sreg | b_from_0 | b_from_mem);
assign alu_b=(io_data_r                       & {8{b_from_io}}) |
             ({inst_r[11:8],inst_r[3:0]}      & {8{b_from_imm1}}) |
             ({2'b00,inst_r[7:6],inst_r[3:0]} & {8{b_from_imm2}}) |
             (sreg_i                          & {8{b_from_sreg}}) |
             (8'h00                           & {8{b_from_0}}) |
             (ram_data_i                      & {8{b_from_mem}}) |
             (rr_read                         & {8{b_from_rr}});
// Shifts differ in the Cin, clasify them
assign is_ror=idc_shf_op_r==2'b11;
assign is_asr=~idc_shf_op_r[1];
// is_lsr => Cin=0
assign alu_ci_1=(`sreg_c_r &                    // Carry flag when:
                 (cyc_4_xxiw_r |                // ADIW/SBIW (4, 2nd add)
                 (cyc_2_add_r & idc_add_wc_r) | // ADC
                  cyc_2_sbc_r |                 // SBC/CPC/SBCI
                 (cyc_2_shf_r & is_ror)))       // ROR
                |
                (rd_read[7] &             // Rd[7] when:
                 (cyc_2_shf_r & is_asr)); // ASR
assign alu_ci=alu_ci_1 | cyc_2_inc_dec_r; // INC/DEC always uses Cin='1'

assign op_add=((cyc_2_xxiw_r | cyc_4_xxiw_r) & ~idc_is_sub_r) | cyc_2_add_r; // ADIW/ADC/ADD/INC
assign op_sub=cyc_2_sub_r |  // SUB/SBC/CP/CPC/SUBI/SBCI/CPI/NEG/DEC/CPSE
              ((cyc_2_xxiw_r | cyc_4_xxiw_r) & idc_is_sub_r); // SBIW
assign op_and=(cyc_2_wioa_r & ~idc_set_r) | // CBI
               cyc_2_sbix_r |               // SBIS/SBIC
               cyc_2_sbrx_r |               // SBRS/SBRC
               cyc_1_bclr |                 // BCLR
               cyc_1_brbx |                 // BRBC/BRBS
               cyc_2_bst_r |                // BST
               cyc_2_bld_clr_r |            // BLD w/T=0
               cyc_2_and_r;                 // AND
assign op_or=(cyc_2_wioa_r & idc_set_r) |   // SBI
              cyc_1_bset |                  // BSET
              cyc_2_or_r |                  // OR
              cyc_2_bld_set_r |             // BLD w/T=1
              cyc_2_mov_r |                 // MOV
              cyc_3_ld_r |                  // LD*
              do_store |                    // ST*
              cyc_2_com_r |                 // COM
              cyc_2_out_r | cyc_2_in_r;     // IN/OUT
assign op_xor=cyc_2_eor_r;  // EOR
assign op_shf=cyc_2_shf_r;  // ASR/LSR/ROR
assign op_swp=cyc_2_swap_r; // SWAP

reg [2:0] b; // Logic
always @(inst_cur, cyc_1_bc_bs)
begin : do_bit_mask
   bit_mask <= 0;
   if (cyc_1_bc_bs)
      b=inst_cur[6:4];
   else
      b=inst_cur[2:0];
   bit_mask[b] <= 1;
end // do_bit_mask

/////////////////////////////////////
// Interrupt logic & state machine //
/////////////////////////////////////
assign irq_req=irq_lines_i!=0;

// Compute the IRQ jump
integer i;
always @(irq_lines_i)
begin : comp_vector
  irq_vector_adr <= 0;
  for (i=IRQ_LINES-1; i>=0; i=i-1)
      if (irq_lines_i[i])
         irq_vector_adr <= i+1;
end // comp_vector

// Interrupts can be attended when all these conditions are met:
// 1) We are at the beggining of an instruction (not in the middle)
// 2) The interrupt flag is 1
// 3) We executed at least one instruction after the last interrupt
assign cyc_1_irq_next=irq_req & sreg_i_next & ~was_irq_next & cyc_1_next;

// ACK the request during the 2nd cycle
genvar j;
generate
for (j=IRQ_LINES-1; j>=0; j=j-1)
    begin : ack_decoder
    assign irq_acks_o[j]=irq_vector_adr==j+1 ? cyc_2_irq_r : 0;
    end
endgenerate

// When returning from an IRQ we must execute at least one instruction
// before serving another interrupt. This flag indicates we returned
// from an interrupt & blocks the interrupts.
assign was_irq_next=~rst_i & cyc_4_ret_r & idc_reti_r;
assign irq_ok_o    = irq_req;
// ########################################################################################

// Sleep
assign sleep_o=idc_sleep;
// Watchdog
assign wdr_o=idc_wdr;

/////////////////////
// Debug Interface //
/////////////////////
// For 32 bits instructions we wait until we have the second part fetched
generate
if (ENA_DEBUG)
   begin : do_debug_stuff
   assign dbg_inst_o   =inst_cur; // Current instruction (under execution)
   assign dbg_inst2_o  =inst_i;   // 2nd part of a 32 bits instruction
   assign dbg_exec_o   =((cyc_1_r && ~cyc_2_lds_next && ~cyc_2_sts_next) || // 16 bits instruction cycle 1
                          cyc_2_lds_r || cyc_2_sts_r) // 32 bits instruction cycle 2
                          && ena_i;
   assign dbg_is32_o   =cyc_2_lds_r | cyc_2_sts_r;
   assign dbg_stopped_o=stopped;
   assign dbg_rd_data_o=alu_s;
   assign dbg_rd_we_o  =rd_we;

   reg [15:0] dbg_pco_r;
   always @(posedge clk_i)
   begin : do_dbg_pc
     if (rst_i)
        dbg_pco_r <= 0;
     else if (cyc_1_next)
        dbg_pco_r <= pc;
   end // do_dbg_pc
   assign dbg_pc_o=dbg_pco_r;
   end // do_debug_stuff
else
   begin
   assign dbg_pc_o=0;
   assign dbg_inst2_o=0;
   assign dbg_exec_o=0;
   assign dbg_is32_o=0;
   assign dbg_stopped_o=0;
   assign dbg_rd_data_o=0;
   assign dbg_rd_we_o=0;
   end
endgenerate

assign stopped=ENA_DEBUG && dbg_stop_i;
assign enabled=ena_i & ~stopped;

always @(posedge clk_i)
begin : do_was_ena_r
  if (rst_i | enabled)
     was_ena_r <= 1;
  else
     was_ena_r <= 0;
end // do_was_ena_r

// I/O instructions can block the CPU in the middle of an instruction (Wait States)
// In this case we could be doing a new fetch & will lose the inst_i content.
// This signal retains inst_i value during a stop condition.
always @(posedge clk_i)
begin : do_inst_stop_r
  if (rst_i)
     inst_stop_r <= 0;
  else if (was_ena_r)
     inst_stop_r <= inst_i;
end // do_inst_stop_r
assign inst_stop=was_ena_r ? inst_i : inst_stop_r;

endmodule // CPU

