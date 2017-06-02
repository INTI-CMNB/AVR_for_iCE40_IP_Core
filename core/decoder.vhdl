------------------------------------------------------------------------------
----                                                                      ----
----  Instruction Decoder for the AVR core                                ----
----                                                                      ----
----  This file is part FPGA Libre project http://fpgalibre.sf.net/       ----
----                                                                      ----
----  Description:                                                        ----
----  Decodes the instructions.                                           ----
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
---- Design unit:      Decoder(RTL) (Entity and architecture)             ----
---- File name:        decoder.vhdl                                       ----
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

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

library avr;
use avr.Constants.all;
use avr.Types.all;

entity Decoder is
   generic(
      ENA_AVR25 : std_logic:='0';  -- Enable AVR25 instructions (MOVW/LPM Rd,Z)
      ENA_AVR3  : boolean:=false;  -- Enable AVR3 instructions
      ENA_AVR4  : boolean:=false); -- Enable AVR4 instructions
   port(
      -- Instruction to decode
      inst_i   : in  std_logic_vector(15 downto 0);
      inst_r_i : in  std_logic_vector(15 downto 0);
      -- Decoded lines
      idc_o    : out idc_t);
end entity Decoder;

architecture RTL of Decoder is
begin
   -- ADD and ADC are similar, bit 12 indicates the carry
   idc_o.add <= '1' when inst_i(15 downto 13)&inst_i(11 downto 10)="00011" else '0'; -- 000C11XXXXXXXXXX
   idc_o.add_wc_r <= inst_r_i(12); -- With Carry
   -- ADIW and SBIW are similar, bit 8 indicates sub
   idc_o.xxiw <= '1' when inst_i(15 downto 9)="1001011" else '0'; -- 1001011sXXXXXXXX
   idc_o.is_sub_r <= inst_r_i(8);
   -- SUB and SBC are similar, bit 12 is not carry
   idc_o.subo <= '1' when inst_i(15 downto 13)&inst_i(11 downto 10)="00010" else '0'; -- 000c10XXXXXXXXXX
   -- SUBI and SBCI are similar, bit 12 is not carry (same as SUB/SBC)
   idc_o.subic <= '1' when inst_i(15 downto 13)="010" else '0'; -- 010cXXXXXXXXXXXX
   idc_o.sub_nc <= inst_i(12); -- No Carry

   idc_o.ando <= '1' when inst_i(15 downto 10)="001000" else '0'; -- 001000XXXXXXXXXX
   idc_o.andi <= '1' when inst_i(15 downto 12)="0111" else '0'; -- 0111XXXXXXXXXXXX
   -- Shift operations TODO: 1 OP
   idc_o.asr  <= '1' when inst_i(15 downto 9)&inst_i(3 downto 1)="1001010010" else '0'; -- 1001010XXXXX010_
   idc_o.roro <= '1' when inst_i(15 downto 9)&inst_i(3 downto 0)="10010100111" else '0'; -- 1001010XXXXX0111
   idc_o.lsr  <= '1' when inst_i(15 downto 9)&inst_i(3 downto 0)="10010100110" else '0'; -- 1001010XXXXX0110
   idc_o.shf_op_r <= inst_r_i(1 downto 0); -- 0x ASR, 10 LSR, 11 ROR
   -- BCLR and BSET are similar, bit 7 indicates clear
   idc_o.bc_bs<= '1' when inst_i(15 downto 8)&inst_i(3 downto 0)="100101001000" else '0'; -- 10010100CXXX1000
   idc_o.bclr <= inst_i(7); -- Clear
   idc_o.bld  <= '1' when inst_i(15 downto 9)="1111100" else '0'; -- 1111100XXXXX_XXX
   -- BRBC and BRBS are similar, bit 10 indicates if clear
   idc_o.brbx <= '1' when inst_i(15 downto 11)="11110" else '0'; -- 11110CXXXXXXXXXX
   idc_o.brbc <= inst_i(10); -- Is Clear

   idc_o.bst  <= '1' when inst_i(15 downto 9)="1111101" else '0'; -- 1111101XXXXX_XXX

   idc_o.com  <= '1' when inst_i(15 downto 9)&inst_i(3 downto 0)="10010100000" else '0'; -- 1001010XXXXX0000
   -- CP and CPC are similar, bit 12 is no carry
   idc_o.cp   <= '1' when inst_i(15 downto 13)&inst_i(11 downto 10)="00001" else '0'; -- 000c01XXXXXXXXXX
   -- CPI is coherent with CP: bit 12=='1'
   idc_o.cpi  <= '1' when inst_i(15 downto 12)="0011" else '0'; -- 0011XXXXXXXXXXXX
   idc_o.cpse <= '1' when inst_i(15 downto 10)="000100" else '0'; -- 000100XXXXXXXXXX
   idc_o.lpm <= '1' when inst_i="1001010111001000" or -- LPM
                         inst_i="1001010111011000" or -- ELPM
                         --inst_i="1001010111101000" or -- LPM image
                         --inst_i="1001010111111000" or -- ELPM image
                         (inst_i(15 downto 10)&inst_i(3 downto 1)="100100010" and
                             (ENA_AVR4 or ENA_AVR25='1')) -- LPM Rd,Z[+]
                         else '0';
   idc_o.spm  <= '1' when inst_i="1001010111101000" else '0'; -- SPM 0x95E8 AV4
   idc_o.elpm_r <= inst_r_i(4); -- Extended LPM
   idc_o.eor  <= '1' when inst_i(15 downto 10)="001001" else '0'; -- 001001XXXXXXXXXX
   idc_o.ino  <= '1' when inst_i(15 downto 11)="10110" else '0'; -- 10110XXXXXXXXXXX
   idc_o.inc  <= '1' when inst_i(15 downto 9)&inst_i(3 downto 0)="10010100011" else '0'; -- 1001010XXXXX0011
   idc_o.dec  <= '1' when inst_i(15 downto 9)&inst_i(3 downto 1)="1001010101" else '0'; -- 1001010XXXXX101_

   -- LD Rd,X; LD Rd,X+; LD Rd,-X
   --          LD Rd,Y+; LD Rd,-Y
   --          LD Rd,Z+; LD Rd,-Z
   -- Note: LD Rd,Y and LD Rd,Z are just LDD Rd,Y+q and LDD Rd,Z+q with q=0
   -- 1001 000d dddd RRoo
   -- Note: LDS and POP shares the same decoding space
   idc_o.ld <= '1' when inst_i(15 downto 9)="1001000" and
                        (inst_i(3 downto 0)="0001" or -- Z+
                         inst_i(3 downto 0)="0010" or -- -Z
                         --inst_i(3 downto 0)="0011" or -- not used
                         --inst_i(3 downto 0)="1000" or -- not used
                         inst_i(3 downto 0)="1001" or -- Y+
                         inst_i(3 downto 0)="1010" or -- -Y
                         --inst_i(3 downto 0)="1011" or -- not used
                         inst_i(3 downto 0)="1100" or -- X
                         inst_i(3 downto 0)="1101" or -- X+
                         inst_i(3 downto 0)="1110")   -- -X
                       else '0';
   -- inst_i(3 downto 2) 11=X 10=Y 00=Z
   idc_o.ld_reg <= inst_i(3 downto 2);
   -- inst_i(1 downto 0) 00 nop 01 X+ 10 -X
   idc_o.ld_op_r  <= inst_r_i(1 downto 0);

   -- ST X,Rr; ST X+,Rr; ST -X,Rr
   --          ST Y+,Rr; ST -Y,Rr
   --          ST Z+,Rr; ST -Z,Rr
   -- Note: ST Y,Rr and ST Z,Rr are just STD Rd,Y+q and STD Rd,Z+q with q=0
   -- 1001 001r rrrr RRoo
   -- Note: STS and PUSH shares the same decoding space
   idc_o.st <= '1' when inst_i(15 downto 9)="1001001" and
                        (inst_i(3 downto 0)="0001" or -- Z+
                         inst_i(3 downto 0)="0010" or -- -Z
                         --inst_i(3 downto 0)="0011" or -- not used
                         --inst_i(3 downto 2)="01"   or -- not used
                         --inst_i(3 downto 0)="1000" or -- not used
                         inst_i(3 downto 0)="1001" or -- Y+
                         inst_i(3 downto 0)="1010" or -- -Y
                         --inst_i(3 downto 0)="1011" or -- not used
                         inst_i(3 downto 0)="1100" or -- X
                         inst_i(3 downto 0)="1101" or -- X+
                         inst_i(3 downto 0)="1110")   -- -X
                       else '0';
   -- ld_reg, ld_reg_r and ld_op_r are the same here

   -- LDD Rd,Y+q; LDD Rd,Z+q
   -- q=0 implements LD Rd,Y and LD Rd,Z
   -- 10q0qq0dddddRqqq R: 1 Y, 0 Z
   idc_o.ldd   <= '1' when inst_i(15 downto 14)&inst_i(12)&inst_i(9)="1000" else '0';
   idc_o.ldd_y <= inst_i(3);
   idc_o.q_r   <= inst_r_i(13)&unsigned(inst_r_i(11 downto 10))&unsigned(inst_r_i(2 downto 0));
   -- STD Y+q,Rr; STD Z+q,Rr
   -- q=0 implements STD Y,Rr and STD Z,Rr
   -- 10q0qq1dddddRqqq R: 1 Y, 0 Z
   idc_o.stdo <= '1' when inst_i(15 downto 14)&inst_i(12)&inst_i(9)="1001" else '0'; -- 10X0XX1XXXXX1XXX

   idc_o.ldi  <= '1' when inst_i(15 downto 12)="1110" else '0'; -- 1110XXXXXXXXXXXX
   idc_o.lds  <= '1' when inst_i(15 downto 9)&inst_i(3 downto 0)="10010000000" else '0'; -- 1001000XXXXX0000
   idc_o.mov  <= '1' when inst_i(15 downto 10)="001011" else '0'; -- 001011XXXXXXXXXX
   -- NEG is coherent with SUB: bit(12)='1' => No carry
   idc_o.neg  <= '1' when inst_i(15 downto 9)&inst_i(3 downto 0)="10010100001" else '0'; -- 1001010XXXXX0001
   idc_o.oro  <= '1' when inst_i(15 downto 10)="001010" else '0'; -- 001010XXXXXXXXXX
   idc_o.ori  <= '1' when inst_i(15 downto 12)="0110" else '0'; -- 0110XXXXXXXXXXXX
   idc_o.outo <= '1' when inst_i(15 downto 11)="10111" else '0'; -- 10111XXXXXXXXXXX
   idc_o.pop  <= '1' when inst_i(15 downto 9)&inst_i(3 downto 0)="10010001111" else '0'; -- 1001000XXXXX1111
   idc_o.push <= '1' when inst_i(15 downto 9)&inst_i(3 downto 0)="10010011111" else '0'; -- 1001001XXXXX1111
   -- RET and RETI are similar, bit 4 indicates "from interrupt"
   idc_o.ret  <= '1' when inst_i(15 downto 7)&inst_i(3 downto 0)="1001010101000" else '0'; -- 100101010XXi1000
   idc_o.reti_r <= inst_r_i(4);

   idc_o.rjmp <= '1' when inst_i(15 downto 12)="1100" else '0'; -- 1100XXXXXXXXXXXX
   idc_o.rcall<= '1' when inst_i(15 downto 12)="1101" else '0'; -- 1101XXXXXXXXXXXX
   idc_o.ijmp <= '1' when inst_i(15 downto 8)&inst_i(3 downto 0)="100101001001" else '0'; -- 10010100XXXX1001
   idc_o.icall<= '1' when inst_i(15 downto 8)&inst_i(3 downto 0)="100101011001" else '0'; -- 10010101XXXX1001

   -- CBI and SBI are similar, bit 9 indicates if set
   idc_o.xbi  <= '1' when inst_i(15 downto 10)&inst_i(8)="1001100" else '0'; -- 100110S0XXXXXXXX
   -- SBIC and SBIS are similar, bit 9 indicates if set
   idc_o.sbix <= '1' when inst_i(15 downto 10)&inst_i(8)="1001101" else '0'; -- 100110S1XXXXXXXX
   -- SBRC and SBRS are similar, bit 9 indicates if set
   idc_o.sbrx <= '1' when inst_i(15 downto 10)="111111" else '0'; -- 111111SXXXXXXXXX
   idc_o.set_r <= inst_r_i(9); -- Set

   idc_o.sleep<= '1' when inst_i(15 downto 5)&inst_i(3 downto 0)="100101011001000" else '0'; -- 10010101100X1000

   idc_o.sts  <= '1' when inst_i(15 downto 9)&inst_i(3 downto 0)="10010010000" else '0'; -- 1001001XXXXX0000
   idc_o.swap <= '1' when inst_i(15 downto 9)&inst_i(3 downto 0)="10010100010" else '0'; -- 1001010XXXXX0010
   idc_o.wdr  <= '1' when inst_i(15 downto 5)&inst_i(3 downto 0)="100101011011000" else '0'; -- 10010101101X1000
   -----------------------
   -- AVR3 instructions --
   -----------------------
   idc_o.call <= '1' when inst_i(15 downto 9)&inst_i(3 downto 1)="1001010111" and
                          ENA_AVR3 else '0'; -- 1001010XXXXX111X
   idc_o.jmp  <= '1' when inst_i(15 downto 9)&inst_i(3 downto 1)="1001010110"
                          and ENA_AVR3 else '0'; -- 1001010XXXXX110X 0x94xC-0x95xD
   -----------------------
   -- AVR4 instructions --
   -----------------------
   idc_o.mul    <= '1' when inst_i(15 downto 10)="100111" and ENA_AVR4 else '0';  -- 100111RDDDDDRRRR
   idc_o.muls   <= '1' when inst_i(15 downto 8)="00000010" and ENA_AVR4 else '0'; -- 00000010DDDDRRRR
   idc_o.mulsu  <= '1' when inst_i(15 downto 7)="000000110" and inst_i(3)='0'
                            and ENA_AVR4 else '0';                              -- 000000110DDD0RRR
   idc_o.fmul   <= '1' when inst_i(15 downto 7)="000000110" and inst_i(3)='1'
                            and ENA_AVR4 else '0';                              -- 000000110DDD1RRR
   idc_o.fmuls  <= '1' when inst_i(15 downto 7)="000000111" and inst_i(3)='0'
                            and ENA_AVR4 else '0';                              -- 000000111DDD0RRR
   idc_o.fmulsu <= '1' when inst_i(15 downto 7)="000000111" and inst_i(3)='1'
                            and ENA_AVR4 else '0';                              -- 000000111DDD1RRR
   -- AVR4 and AVR25(2.5)
   idc_o.movw   <= '1' when inst_i(15 downto 8)="00000001" and
                            (ENA_AVR4 or ENA_AVR25='1') else '0'; -- 00000001DDDDRRRR
   -- LPM Rd,Z decoding is in LPM decoding
   -- Most invalid opcodes are decoded as NOP
end architecture RTL; -- Entity: Decoder

