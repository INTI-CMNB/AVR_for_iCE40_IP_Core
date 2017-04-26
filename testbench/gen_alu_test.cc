/**[txh]********************************************************************

  Copyright (c) 2008 Salvador E. Tropea <salvador en inti gov ar>
  Copyright (c) 2008 Instituto Nacional de Tecnología Industrial

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; version 2.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
  02111-1307, USA

  Description:
  AVR ALU emulator. Generates 4466688 test cases.
  Without AVR4: 3940352 combinations, real 12m14.124s, i5-2310 CPU @ 2.90GHz

***************************************************************************/
/*****************************************************************************

 Target:      Any
 Language:    C++
 Compiler:    GNU g++ 4.1.1 (Debian GNU/Linux)
 Text editor: SETEdit 0.5.5
 Dependence:  cppfio library.
    
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

// H is the carry from the 3:0 nibble to the 7:4 nibble.
// S sign, it takes care about the overflow (N^V)
// V is the 2's complement overflow. You assume the arguments are 2's
//   complement encoded and then verify if the result was outside the range.
// N the result is negative (b7=1), doesn't take care of the overflow.
// Z result is 0. Some operations needs that the previous was also 0 (CPC/SBC/SBCI)
// C is the full carry.

static unsigned Rd, Rr, Cin, Zin;
static unsigned R, H, S, V, N, Z, C, opcode;
static const char *name;
static FILE *out;
const unsigned DontCare=2;
const int DebugMeanings=1;
typedef char i8;
typedef unsigned char u8;
typedef short i16;
typedef unsigned short u16;


static
void prHexa8(unsigned v)
{
 unsigned mask;
 for (mask=0x80; mask; mask>>=1)
     putc(v&mask ? '1' : '0',out);
 putc(' ',out);
}

static
void prHexa16(unsigned v)
{
 unsigned mask;
 for (mask=0x8000; mask; mask>>=1)
     putc(v&mask ? '1' : '0',out);
 putc(' ',out);
}

static
void pr01(unsigned v)
{
 putc(v ? '1' : '0',out);
 putc(' ',out);
}

static
void pr01X(unsigned v)
{
 if (v==1)
    putc('1',out);
 else if (v==0)
    putc('0',out);
 else
    putc('-',out);
 putc(' ',out);
}

static
void prLine()
{
 fprintf(out,"%-6s ",name);
 prHexa16(opcode);
 prHexa8(Rd);
 prHexa8(Rr);
 pr01(Cin);
 pr01(Zin);
 prHexa8(R);
 pr01X(H);
 pr01X(S);
 pr01X(V);
 pr01X(N);
 pr01X(Z);
 pr01X(C);
 putc('\n',out);
}

// H and C for +
static
void CyAdd(unsigned d, unsigned r, unsigned Cy)
{// By meaning
 unsigned dl=d&0xF;
 unsigned rl=r&0xF;
 H=(dl+rl+Cy)>0xF ? 1 : 0;
 C=(d+r+Cy)>0xFF ? 1 : 0;
 if (DebugMeanings)
   {// By logic
    unsigned Rd3, Rr3, R3;
    Rd3=d&0x8;
    Rr3=r&0x8;
    R3=R&0x8;
    unsigned Hl=(Rd3&Rr3)|((~R3)&(Rd3|Rr3)) ? 1 : 0;
    if (H!=Hl)
      {
       out=stderr;
       prLine();
       fprintf(stderr,"H: %d Hl: %d\n",H,Hl);
       exit(1);
      }
    unsigned Rd7, Rr7, R7;
    Rd7=d&0x80;
    Rr7=r&0x80;
    R7=R&0x80;
    unsigned Cl=(Rd7&Rr7)|((~R7)&(Rd7|Rr7)) ? 1 : 0;
    if (C!=Cl)
      {
       out=stderr;
       prLine();
       fprintf(stderr,"C: %d Cl: %d\n",C,Cl);
       exit(1);
      }
   }
}

static
void CySub(int d, int r, int Cy)
{// By meaning
 int dl=d&0xF;
 int rl=r&0xF;
 H=(dl-rl-Cy)<0 ? 1 : 0;
 C=(d-r-Cy)<0 ? 1 : 0;
 if (DebugMeanings)
   {// By logic
    unsigned Rd3, Rr3, R3;
    Rd3=d&0x8;
    Rr3=r&0x8;
    R3=R&0x8;
    unsigned Hl=((~Rd3)&Rr3)|(R3&((~Rd3)|Rr3)) ? 1 : 0;
    if (H!=Hl)
      {
       out=stderr;
       prLine();
       fprintf(stderr,"H: %d Hl: %d (d:%d r:%d R:%d Cy:%d)\n",H,Hl,d,r,R,Cy);
       exit(1);
      }
    unsigned Rd7, Rr7, R7;
    Rd7=d&0x80;
    Rr7=r&0x80;
    R7=R&0x80;
    unsigned Cl=((~Rd7)&Rr7)|(R7&((~Rd7)|Rr7)) ? 1 : 0;
    if (C!=Cl)
      {
       out=stderr;
       prLine();
       fprintf(stderr,"C: %d Cl: %d (d:%d r:%d R:%d Cy:%d)\n",C,Cl,d,r,R,Cy);
       exit(1);
      }
   }
}

static
void OverflowAdd(unsigned d, unsigned r)
{
 bool dPositive=d&0x80;
 bool rPositive=r&0x80;
 bool RPositive=R&0x80;
 if (dPositive && rPositive)
    V=RPositive ? 0 : 1; // If d and r are positive the result must be positive
 else if ((!dPositive) && (!rPositive))
    V=RPositive ? 1 : 0; // If d and r are negative the result must be negative
 else
    V=0; // We can't overflow while adding a positive to a negative
}

static
void OverflowSub(unsigned d, unsigned r)
{// This is the same as OverflowAdd, just thinking: D+(-R)
 bool dPositive=d&0x80;
 bool rPositive=r&0x80;
 bool RPositive=R&0x80;
 if (dPositive && (!rPositive))
    V=RPositive ? 0 : 1;
 else if ((!dPositive) && rPositive)
    V=RPositive ? 1 : 0;
 else
    V=0;
}

static
void idc_add()
{
 name="add";
 R=Rd+Rr;
 CyAdd(Rd,Rr,0);
 OverflowAdd(Rd,Rr);
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 0000 11rd dddd rrrr
 opcode=0x0C01;
 prLine();
}

static
void idc_adc()
{
 name="adc";
 R=Rd+Rr+Cin;
 CyAdd(Rd,Rr,Cin);
 OverflowAdd(Rd,Rr);
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 0001 11rd dddd rrrr
 opcode=0x1C01;
 prLine();
}

static
void idc_adiw1()
{
 name="adiw1";
 R=Rd+Rr;
 CyAdd(Rd,Rr,0);
 H=DontCare;
 // V,N and S don't care, but we compute them anyways (easier)
 //V=DontCare;
 //N=DontCare;
 //S=DontCare;
 OverflowAdd(Rd,Rr);
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 1001 0110 KKdd KKKK
 opcode=0x9600 | ((Rr&0x30)<<2) | (Rr&0xF);
 prLine();
}

static
void idc_adiw2()
{
 name="adiw2";
 R=Rd+0+Cin;
 CyAdd(Rd,0,Cin);
 H=DontCare;
 OverflowAdd(Rd,0);
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 && Zin==1 ? 1 : 0; // The 16 bits are 0
 prLine();
}

static
void idc_sub()
{
 name="sub";
 R=Rd-Rr;
 CySub(Rd,Rr,0);
 OverflowSub(Rd,Rr);
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 0001 10rd dddd rrrr
 opcode=0x1801;
 prLine();
}

static
void idc_subi()
{
 name="subi";
 R=Rd-Rr;
 CySub(Rd,Rr,0);
 OverflowSub(Rd,Rr);
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 0101 kkkk dddd kkkk
 opcode=0x5000 | ((Rr&0xF0)<<4) | (Rr&0xF);
 prLine();
}

static
void idc_sbc()
{
 name="sbc";
 R=Rd-Rr-Cin;
 CySub(Rd,Rr,Cin);
 OverflowSub(Rd,Rr);
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 && Zin==1 ? 1 : 0; // Used for >8bits
 // 0000 10r ddddd rrrr
 opcode=0x0801;
 prLine();
}

static
void idc_sbci()
{
 name="sbci";
 R=Rd-Rr-Cin;
 CySub(Rd,Rr,Cin);
 OverflowSub(Rd,Rr);
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 && Zin==1 ? 1 : 0; // Used for >8bits
 // 0100 kkkk dddd kkkk
 opcode=0x4000 | ((Rr&0xF0)<<4) | (Rr&0xF);
 prLine();
}

static
void idc_sbiw1()
{
 name="sbiw1";
 R=Rd-Rr;
 CySub(Rd,Rr,0);
 H=DontCare;
 // V,N and S don't care, but we compute them anyways (easier)
 //V=DontCare;
 //N=DontCare;
 //S=DontCare;
 OverflowSub(Rd,Rr);
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 1001 0111 KKdd KKKK
 opcode=0x9700 | ((Rr&0x30)<<2) | (Rr&0xF);
 prLine();
}

static
void idc_sbiw2()
{
 name="sbiw2";
 R=Rd-0-Cin;
 CySub(Rd,0,Cin);
 H=DontCare;
 OverflowSub(Rd,0);
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 && Zin==1 ? 1 : 0;
 prLine();
}

static
void idc_and()
{
 name="and";
 R=Rd&Rr;
 H=DontCare;
 C=DontCare;
 V=0;
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 0010 00rd dddd rrrr
 opcode=0x2001;
 prLine();
}

static
void idc_andi()
{
 name="andi";
 R=Rd&Rr;
 H=DontCare;
 C=DontCare;
 V=0;
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 0111 kkkk dddd kkkk
 opcode=0x7000 | ((Rr&0xF0)<<4) | (Rr&0xF);
 prLine();
}

static
void idc_or()
{
 name="or";
 R=Rd|Rr;
 H=DontCare;
 C=DontCare;
 V=0;
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 0010 10rd dddd rrrr
 opcode=0x2801;
 prLine();
}

static
void idc_ori()
{
 name="ori";
 R=Rd|Rr;
 H=DontCare;
 C=DontCare;
 V=0;
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 0110 kkkk dddd kkkk
 opcode=0x6000 | ((Rr&0xF0)<<4) | (Rr&0xF);
 prLine();
}

static
void idc_eor()
{
 name="eor";
 R=Rd^Rr;
 H=DontCare;
 C=DontCare;
 V=0;
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 0010 01rd dddd rrrr
 opcode=0x2401;
 prLine();
}

static
void idc_com()
{
 name="com";
 R=0xFF-Rd;
 H=DontCare;
 C=1; // Special case, isn't the carry for 0xFF-Rd
 // Note -1-X can't overflow, you can verify it anyways ...
 OverflowSub(0xFF,Rd);
 if (DebugMeanings)
   {
    if (V)
      {
       out=stderr;
       prLine();
       exit(1);
      }
   }
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 1001 010d dddd 0000
 opcode=0x9400;
 prLine();
}

static
void idc_neg()
{
 name="neg";
 R=0x00-Rd;
 Z=(R&0xFF)==0 ? 1 : 0;
 CySub(0,Rd,0);
 if (DebugMeanings)
   {// By logic
    unsigned Hl=(R&0x8)|(Rd&0x8) ? 1 : 0;
    if (H!=Hl)
      {
       out=stderr;
       prLine();
       fprintf(stderr,"H: %d Hl: %d\n",H,Hl);
       exit(1);
      }
    unsigned Cl=Z ? 0 : 1;
    if (C!=Cl)
      {
       out=stderr;
       prLine();
       fprintf(stderr,"C: %d Cl: %d\n",C,Cl);
       exit(1);
      }
   }
 OverflowSub(0,Rd);
 if (DebugMeanings)
   {// By logic
    unsigned Vl=(R&0xFF)==0x80;
    if (V!=Vl)
      {
       out=stderr;
       prLine();
       fprintf(stderr,"V: %d Vl: %d\n",V,Vl);
       exit(1);
      }
   }
 N=(R>>7)&1;
 S=N^V;
 // 1001 010d dddd 0001
 opcode=0x9401;
 prLine();
}

static
void idc_inc()
{
 name="inc";
 R=Rd+1;
 H=DontCare;
 C=DontCare;
 OverflowAdd(Rd,1);
 if (DebugMeanings)
   {// By logic
    unsigned Vl=(R&0xFF)==0x80; // 127+1 => Overflow
    if (V!=Vl)
      {
       out=stderr;
       prLine();
       fprintf(stderr,"V: %d Vl: %d\n",V,Vl);
       exit(1);
      }
   }
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 1001 010d dddd 0011
 opcode=0x9403;
 prLine();
}

static
void idc_dec()
{
 name="dec";
 R=Rd-1;
 H=DontCare;
 C=DontCare;
 OverflowSub(Rd,1);
 if (DebugMeanings)
   {// By logic
    unsigned Vl=(R&0xFF)==0x7F; // -128-1 => overflow
    if (V!=Vl)
      {
       out=stderr;
       prLine();
       fprintf(stderr,"V: %d Vl: %d\n",V,Vl);
       exit(1);
      }
   }
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 1001 010d dddd 1010
 opcode=0x940A;
 prLine();
}

static
void idc_cp()
{
 name="cp";
 R=Rd-Rr;
 CySub(Rd,Rr,0);
 OverflowSub(Rd,Rr);
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 0001 01rd dddd rrrr
 opcode=0x1401;
 prLine();
}

static
void idc_cpc()
{
 name="cpc";
 R=Rd-Rr-Cin;
 CySub(Rd,Rr,Cin);
 OverflowSub(Rd,Rr);
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 && Zin==1 ? 1 : 0; // This is used for more than 8 bits compares
 // 0000 01rd dddd rrrr
 opcode=0x0401;
 prLine();
}

static
void idc_cpi()
{
 name="cpi";
 R=Rd-Rr;
 CySub(Rd,Rr,0);
 OverflowSub(Rd,Rr);
 N=(R>>7)&1;
 S=N^V;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 0011 kkkk dddd kkkk
 opcode=0x3000 | ((Rr&0xF0)<<4) | (Rr&0xF);
 prLine();
}

static
void idc_cpse()
{
 name="cpse";
 R=Rd-Rr;
 H=DontCare;
 C=DontCare;
 V=DontCare;
 N=DontCare;
 S=DontCare;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 0001 00rd dddd rrrr
 opcode=0x1001;
 prLine();
}

static
void idc_lsr()
{
 name="lsr";
 R=Rd>>1;
 H=DontCare;
 C=Rd&1;
 N=(R>>7)&1;
 V=N^C;
 S=N^V;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 1001 010d dddd 0110
 opcode=0x9406;
 prLine();
}

static
void idc_ror()
{
 name="ror";
 R=(Cin<<7)|(Rd>>1);
 H=DontCare;
 C=Rd&1;
 N=(R>>7)&1;
 V=N^C;
 S=N^V;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 1001 010d dddd 0111
 opcode=0x9407;
 prLine();
}

static
void idc_asr()
{
 name="asr";
 R=(Rd&0x80)|(Rd>>1);
 H=DontCare;
 C=Rd&1;
 N=(R>>7)&1;
 V=N^C;
 S=N^V;
 Z=(R&0xFF)==0 ? 1 : 0;
 // 1001 010d dddd 0101
 opcode=0x9405;
 prLine();
}

static
void idc_swap()
{
 name="swap";
 R=((Rd&0xF)<<4)|(Rd>>4);
 H=DontCare;
 C=DontCare;
 V=DontCare;
 N=DontCare;
 S=DontCare;
 Z=DontCare;
 // 1001 010d dddd 0010
 opcode=0x9402;
 prLine();
}

static
void idc_mul()
{
 name="mul";
 u16 r=u8(Rd)*u8(Rr);
 H=DontCare;
 V=DontCare;
 C=r&0x8000 ? 1 : 0;
 Z=r==0 ? 1 : 0;
 N=DontCare;
 S=DontCare;
 R=r&0xFF;
 // 1001 11rd dddd rrrr
 opcode=0x9C01;
 prLine();
 C=DontCare;
 Z=DontCare;
 R=r>>8;
 prLine();
}

static
void idc_muls()
{
 name="muls";
 i16 r=i8(Rd)*i8(Rr);
 H=DontCare;
 V=DontCare;
 N=DontCare;
 S=DontCare;
 C=r&0x8000 ? 1 : 0;
 Z=r==0 ? 1 : 0;
 R=r&0xFF;
 // 0000 0010 dddd rrrr
 opcode=0x0201;
 prLine();
 C=DontCare;
 Z=DontCare;
 R=r>>8;
 prLine();
}

static
void idc_mulsu()
{
 name="mulsu";
 i16 r=i8(Rd)*u8(Rr);
 H=DontCare;
 V=DontCare;
 N=DontCare;
 S=DontCare;
 C=r&0x8000 ? 1 : 0;
 Z=r==0 ? 1 : 0;
 R=r&0xFF;
 // 0000 0011 0ddd 0rrr
 opcode=0x0301;
 prLine();
 C=DontCare;
 Z=DontCare;
 R=r>>8;
 prLine();
}

static
void idc_fmul()
{
 name="fmul";
 u16 r=u8(Rd)*u8(Rr);
 H=DontCare;
 V=DontCare;
 N=DontCare;
 S=DontCare;
 C=r&0x8000 ? 1 : 0;
 Z=r==0 ? 1 : 0;
 R=(r<<1)&0xFF;
 // 0000 0011 0ddd 1rrr
 opcode=0x0309;
 prLine();
 C=DontCare;
 Z=DontCare;
 R=(r<<1)>>8;
 prLine();
}

static
void idc_fmuls()
{
 name="fmuls";
 i16 r=i8(Rd)*i8(Rr);
 H=DontCare;
 V=DontCare;
 N=DontCare;
 S=DontCare;
 C=r&0x8000 ? 1 : 0;
 Z=r==0 ? 1 : 0;
 R=(r<<1)&0xFF;
 // 0000 0011 1ddd 0rrr
 opcode=0x0381;
 prLine();
 C=DontCare;
 Z=DontCare;
 R=(r<<1)>>8;
 prLine();
}

static
void idc_fmulsu()
{
 name="fmulsu";
 i16 r=i8(Rd)*u8(Rr);
 H=DontCare;
 V=DontCare;
 N=DontCare;
 S=DontCare;
 C=r&0x8000 ? 1 : 0;
 Z=r==0 ? 1 : 0;
 R=(r<<1)&0xFF;
 // 0000 0011 1ddd 1rrr
 opcode=0x0389;
 prLine();
 C=DontCare;
 Z=DontCare;
 R=(r<<1)>>8;
 prLine();
}


int main(int argc, char *argv[])
{
 out=stdout;
 S=DontCare;
 for (Cin=0; Cin<2; Cin++)
    {
     for (Zin=0; Zin<2; Zin++)
        {
         for (Rd=0; Rd<256; Rd++)
            {
             for (Rr=0; Rr<256; Rr++)
                {
                 if (Rd==Rr)
                   {// 1 operand, can be read from Rr or Rd port
                    idc_com();
                    idc_neg();
                    idc_swap();
                    idc_inc();
                    idc_asr();
                    idc_lsr();
                    idc_ror();
                    idc_dec();
                   }
                 // 2 operands
                 if (Rr<64)
                   {
                    idc_adiw1();
                    idc_adiw2();
                    idc_sbiw1();
                    idc_sbiw2();
                   }
                 idc_add();
                 idc_adc();
                 idc_sub();
                 idc_subi();
                 idc_sbc();
                 idc_sbci();
                 idc_and();   
                 idc_andi();
                 idc_or();
                 idc_ori();
                 idc_eor();   
                 idc_cp();    
                 idc_cpc();
                 idc_cpi();
                 // TODO idc_cpse();
                 // Multiplications
                 #ifndef NO_AVR4
                 idc_mul();
                 idc_muls();
                 idc_mulsu();
                 idc_fmul();
                 idc_fmuls();
                 idc_fmulsu();
                 #endif
                }
            }
        }
    }
 return 0;
}
