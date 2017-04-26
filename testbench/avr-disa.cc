/**[txh]********************************************************************

  Copyright (c) 2009 Salvador E. Tropea <salvador at inti gob ar>
  Copyright (c) 2009 Instituto Nacional de Tecnología Industrial

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, version 2 of the License.

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

  Title: arvr-disa

  Description:
  Helper to disassemble the testbenches output.

***************************************************************************/

#define CPPFIO_I
#define CPPFIO_I_stdlib
#define CPPFIO_I_unistd
#define CPPFIO_CmdLine
#define CPPFIO_ErrorLog
#define CPPFIO_TextFile
#include <cppfio.h>

// We are inside the cppfio namespace
using namespace cppfio;

ErrorLog error(stderr);
static bool bSync=false;

/*****************************************************************************
  Command line stuff
*****************************************************************************/

#define VERSION "1.0.0"
#define CMD_NAME "avr-disa"

CmdLineParser clp;

static
void PrintCopy(FILE *f)
{
 fprintf(f,"Copyright (c) 2009 Salvador E. Tropea <salvador at inti gob ar>\n");
 fprintf(f,"Copyright (c) 2009 Instituto Nacional de Tecnología Industrial\n");
}

static
void PrintPrgInfo()
{
 if (clp.GetVerbosity()<0) // Disabled when using --quiet
    return;
 fputs("AVR disassembler v"VERSION"\n",stderr);
 PrintCopy(stderr);
 fputc('\n',stderr);
}

static
void PrintHelp()
{
 puts("Usage:\n"CMD_NAME" [options] [input_file] [output_file]\n");
 puts("Options:");
 puts("-s, --sync                  flush the output when it can't be parsed");
 clp.PrDefaultOptions(stdout);
 puts("");
 exit(1);
}

static
void PrintVersion()
{
 FILE *f=stdout;

 fputs(CMD_NAME" v"VERSION"\n",f);
 PrintCopy(f);
 clp.PrintGPL(f);
 fprintf(f,"\nAuthor: Salvador E. Tropea.\n");
}

static
struct clpOption longopts[]=
{
  { "sync",          's', argNo,       &bSync,         argBool },
  { NULL, 0, argNo, NULL, argBool }
};

cmdLineOpts opts=
{
 PrintPrgInfo, PrintHelp, PrintVersion, longopts, -1
};

/*****************************************************************************
  End of command line stuff
*****************************************************************************/
static const char *br_c[8]=
{ "brcc", "brne", "brpl", "brvc", "brge", "brhc", "brtc", "brid" };
static const char *br_s[8]=
{ "brcs", "breq", "brmi", "brvs", "brlt", "brhs", "brts", "brie" };
static const char *flags="cznvshti";

static
void Disassemble(File *f, unsigned op1, unsigned op2)
{
 unsigned rd1=(op1&0x01F0)>>4;             // xxxx xxxD DDDD xxxx
 unsigned rd2=((op1>>3)&6)|0x18;           // xxxx xxxx xxDD xxxx |0x18
 unsigned rd3=((op1&0xF0)>>4)|0x10;        // xxxx xxxx DDDD xxxx +16
 unsigned rd4=((op1&0x70)>>4)|0x10;        // xxxx xxxx xDDD xxxx +16
 unsigned rr1=(op1&0xF)|((op1&0x0200)>>5); // xxxx xxRx xxxx RRRR
 unsigned rr2=(op1&0x7)|0x10;              // xxxx xxxx xxxx xRRR +16
 unsigned k1=(op1&0xF)|((op1>>2)&0x30);    // xxxx xxxx KKxx KKKK
 unsigned k2=(op1&0xF)|((op1>>4)&0xF0);    // xxxx KKKK xxxx KKKK
 unsigned s1=(op1>>4)&0x7;                 // xxxx xxxx xSSS xxxx
 unsigned s2=op1&7;
 unsigned rel1=(op1>>3)&0x7F;              // xxxx xxKK KKKK Kxxx
 unsigned rel2=op1&0xFFF;                  // xxxx KKKK KKKK KKKK
 if (rel1>63)
    rel1=-128+rel1;
 if (rel2>2047)
    rel2=-4096+rel2;

 if ((op1 & 0xFC00)==0x1C00)
   {
    if (rd1!=rr1)
       f->Print("adc    r%-2d,r%-2d\n",rd1,rr1);
    else
       f->Print("rol    r%-2d\n",rd1);
   }
 else if ((op1 & 0xFC00)==0x0C00)
   {
    if (rd1!=rr1)
       f->Print("add    r%-2d,r%-2d\n",rd1,rr1);
    else
       f->Print("lsl    r%-2d\n",rd1);
   }
 else if ((op1 & 0xFF00)==0x9600)
    f->Print("adiw   r%-2d,%d\n",rd2,k1);
 else if ((op1 & 0xFC00)==0x2000)
   {
    if (rd1!=rr1)
       f->Print("and    r%-2d,r%-2d\n",rd1,rr1);
    else
       f->Print("tst    r%-2d\n",rd1);
   }
 else if ((op1 & 0xF000)==0x7000)
    f->Print("andi   r%-2d,%-3d        ; aka cbr\n",rd3,k2);
 else if ((op1 & 0xFE0F)==0x9405)
    f->Print("asr    r%d\n",rd1);
 else if ((op1 & 0xFF8F)==0x9488)
    f->Print("cl%c                   ; aka bclr %d\n",flags[s1],s1);
 else if ((op1 & 0xFE08)==0xF800)
    f->Print("bld    r%-2d,%d\n",rd1,s2);
 else if ((op1 & 0xFC00)==0xF400)
   {
    f->Print("%s   %-3d            ; brbc %d,%d",br_c[s2],rel1,s2,rel1);
    if (!s2)
       f->Print(" aka brsh");
    f->Write('\n');
   }
 else if ((op1 & 0xFC00)==0xF000)
   {
    f->Print("%s   %-3d            ; brbs %d,%d",br_s[s2],rel1,s2,rel1);
    if (!s2)
       f->Print(" aka brlo");
    f->Write('\n');
   }
 else if (op1==0x9598)
    f->Print("break\n");
 else if ((op1 & 0xFE08)==0xFA00)
    f->Print("bst    r%-2d,%d\n",rd1,s2);
 else if ((op1 & 0xFE0E)==0x940E)
    f->Print("call   0x%04X\n",((((op1>>3)&0x3E)|(op1&1))<<16)|op2);
 else if ((op1 & 0xFF00)==0x9800)
    f->Print("cbi    0x%02X,%d\n",(op1>>3)&0x1F,s2);
 else if ((op1 & 0xFE0F)==0x9400)
    f->Print("com    r%-2d\n",rd1);
 else if ((op1 & 0xFC00)==0x1400)
    f->Print("cp     r%-2d,r%-2d\n",rd1,rr1);
 else if ((op1 & 0xFC00)==0x0400)
    f->Print("cpc    r%-2d,r%-2d\n",rd1,rr1);
 else if ((op1 & 0xF000)==0x3000)
    f->Print("cpi    r%-2d,%d\n",rd3,k2);
 else if ((op1 & 0xFC00)==0x1000)
    f->Print("cpse   r%-2d,r%-2d\n",rd1,rr1);
 else if ((op1 & 0xFE0F)==0x940A)
    f->Print("dec    r%-2d\n",rd1);
 else if (op1==0x9519)
    f->Print("eicall\n");
 else if (op1==0x9419)
    f->Print("eijmp\n");
 else if (op1==0x95D8)
    f->Print("elpm\n");
 else if ((op1 & 0xFE0F)==0x9006)
    f->Print("elpm   r%-2d,Z\n",rd1);
 else if ((op1 & 0xFE0F)==0x9007)
    f->Print("elpm   r%-2d,Z+\n",rd1);
 else if ((op1 & 0xFC00)==0x2400)
   {
    if (rd1!=rr1)
       f->Print("eor    r%-2d,r%-2d\n",rd1,rr1);
    else
       f->Print("clr    r%-2d\n",rd1);
   }
 else if (op1==0x95F8)
    f->Print("espm\n");
 else if ((op1 & 0xFF88)==0x0308)
    f->Print("fmul   r%-2d,r%-2d\n",rd4,rr2);
 else if ((op1 & 0xFF88)==0x0380)
    f->Print("fmuls  r%-2d,r%-2d\n",rd4,rr2);
 else if ((op1 & 0xFF88)==0x0388)
    f->Print("fmulsu r%-2d,r%-2d\n",rd4,rr2);
 else if (op1==0x9509)
    f->Print("icall\n");
 else if (op1==0x9409)
    f->Print("ijmp\n");
 else if ((op1 & 0xF800)==0xB000)
    f->Print("in     r%-2d,0x%02X\n",rd1,(op1&0xF)|((op1>>5)&0x30));
 else if ((op1 & 0xFE0F)==0x9403)
    f->Print("inc    r%-2d\n",rd1);
 else if ((op1 & 0xFE0E)==0x940C)
    f->Print("jmp    0x%04X\n",((((op1>>3)&0x3E)|(op1&1))<<16)|op2);
 else if ((op1 & 0xFE0F)==0x8008)
    f->Print("ld     r%-2d,Y\n",rd1);
 else if ((op1 & 0xFE0F)==0x9009)
    f->Print("ld     r%-2d,Y+\n",rd1);
 else if ((op1 & 0xFE0F)==0x8000)
    f->Print("ld     r%-2d,Z\n",rd1);
 else if ((op1 & 0xFE0F)==0x9001)
    f->Print("ld     r%-2d,Z+\n",rd1);
 else if ((op1 & 0xFE0F)==0x900E)
    f->Print("ld     r%-2d,-X\n",rd1);
 else if ((op1 & 0xFE0F)==0x900A)
    f->Print("ld     r%-2d,-Y\n",rd1);
 else if ((op1 & 0xFE0F)==0x9002)
    f->Print("ld     r%-2d,-Z\n",rd1);
 else if ((op1 & 0xFE0F)==0x900C)
    f->Print("ld     r%-2d,X\n",rd1);
 else if ((op1 & 0xFE0F)==0x900D)
    f->Print("ld     r%-2d,X+\n",rd1);
 else if ((op1 & 0xD208)==0x8008)
    f->Print("ldd    r%-2d,Y+%d\n",rd1,(op1&7)|((op1>>7)&0x18)|((op1>>8)&0x20));
 else if ((op1 & 0xD208)==0x8000)
    f->Print("ldd    r%-2d,Z+%d\n",rd1,(op1&7)|((op1>>7)&0x18)|((op1>>8)&0x20));
 else if ((op1 & 0xF000)==0xE000)
   {
    if (k2!=0xFF)
       f->Print("ldi    r%-2d,0x%02X       ; %d\n",rd3,k2,k2);
    else
       f->Print("ser    r%-2d\n",rd3);
   }
 else if ((op1 & 0xFE0F)==0x9000)
    f->Print("lds    r%-2d,0x%04X     ; %d\n",rd1,op2,op2);
 else if (op1==0x95C8)
    f->Print("lpm\n");
 else if ((op1 & 0xFE0F)==0x9004)
    f->Print("lpm    r%-2d,Z\n",rd1);
 else if ((op1 & 0xFE0F)==0x9005)
    f->Print("lpm    r%-2d,Z+\n",rd1);
 else if ((op1 & 0xFE0F)==0x9406)
    f->Print("lsr    r%-2d\n",rd1);
 else if ((op1 & 0xFC00)==0x2C00)
    f->Print("mov    r%-2d,r%-2d\n",rd1,rr1);
 else if ((op1 & 0xFF00)==0x0100)
    f->Print("movw   r%-2d,r%-2d\n",(rd1&0xF)<<1,(rr1&0xF)<<1);
 else if ((op1 & 0xFC00)==0x9C00)
    f->Print("mul    r%-2d,r%-2d\n",rd1,rr1);
 else if ((op1 & 0xFF00)==0x0200)
    f->Print("muls   r%-2d,r%-2d\n",(rd1&7)+16,(rr1&7)+16);
 else if ((op1 & 0xFF88)==0x0300)
    f->Print("mulsu  r%-2d,r%-2d\n",rd4,rr2);
 else if ((op1 & 0xFE0F)==0x9401)
    f->Print("neg    r%-2d\n",rd1);
 else if (op1==0)
    f->Print("nop\n");
 else if ((op1 & 0xFC00)==0x2800)
    f->Print("or     r%-2d,r%-2d\n",rd1,rr1);
 else if ((op1 & 0xF000)==0x6000)
    f->Print("ori    r%-2d,%d            ; aka sbr\n",rd3,k2);
 else if ((op1 & 0xF800)==0xB800)
    f->Print("out    0x%02X,r%-2d\n",(op1&0xF)|((op1>>5)&0x30),rd1);
 else if ((op1 & 0xFE0F)==0x900F)
    f->Print("pop    r%d\n",rd1);
 else if ((op1 & 0xFE0F)==0x920F)
    f->Print("push   r%d\n",rd1);
 else if ((op1 & 0xF000)==0xD000)
    f->Print("rcall  %d\n",rel2);
 else if (op1==0x9508)
    f->Print("ret\n");
 else if (op1==0x9518)
    f->Print("reti\n");
 else if ((op1 & 0xF000)==0xC000)
    f->Print("rjmp   %d\n",rel2);
 else if ((op1 & 0xFE0F)==0x9407)
    f->Print("ror    r%d\n",rd1);
 else if ((op1 & 0xFC00)==0x0800)
    f->Print("sbc    r%-2d,r%-2d\n",rd1,rr1);
 else if ((op1 & 0xF000)==0x4000)
    f->Print("sbci   r%-2d,%d\n",rd3,k2);
 else if ((op1 & 0xFF00)==0x9A00)
    f->Print("sbi    0x%02X,%d\n",(op1>>3)&0x1F,s2);
 else if ((op1 & 0xFF00)==0x9900)
    f->Print("sbic   0x%02X,%d\n",(op1>>3)&0x1F,s2);
 else if ((op1 & 0xFF00)==0x9B00)
    f->Print("sbis   0x%02X,%d\n",(op1>>3)&0x1F,s2);
 else if ((op1 & 0xFF00)==0x9700)
    f->Print("sbiw   r%-2d,%d\n",rd2,k1);
 else if ((op1 & 0xFE08)==0xFC00)
    f->Print("sbrc   r%-2d,%d\n",rd1,s2);
 else if ((op1 & 0xFE08)==0xFE00)
    f->Print("sbrs   r%-2d,%d\n",rd1,s2);
 else if ((op1 & 0xFF8F)==0x9408)
    f->Print("se%c                   ; aka bset %d\n",flags[s1],s1);
 else if (op1==0x9588)
    f->Print("sleep\n");
 else if (op1==0x95E8)
    f->Print("spm\n");
 else if ((op1 & 0xFE0F)==0x920C)
    f->Print("st     r%-2d,X\n",rd1);
 else if ((op1 & 0xFE0F)==0x920D)
    f->Print("st     r%-2d,X+\n",rd1);
 else if ((op1 & 0xFE0F)==0x8208)
    f->Print("st     r%-2d,Y\n",rd1);
 else if ((op1 & 0xFE0F)==0x9209)
    f->Print("st     r%-2d,Y+\n",rd1);
 else if ((op1 & 0xFE0F)==0x8200)
    f->Print("st     r%-2d,Z\n",rd1);
 else if ((op1 & 0xFE0F)==0x9201)
    f->Print("st     r%-2d,Z+\n",rd1);
 else if ((op1 & 0xFE0F)==0x920E)
    f->Print("st     r%-2d,-X\n",rd1);
 else if ((op1 & 0xFE0F)==0x920A)
    f->Print("st     r%-2d,-Y\n",rd1);
 else if ((op1 & 0xFE0F)==0x9202)
    f->Print("st     r%-2d,-Z\n",rd1);
 else if ((op1 & 0xD208)==0x8208)
    f->Print("std    r%-2d,Y+%d\n",rd1,(op1&7)|((op1>>7)&0x18)|((op1>>8)&0x20));
 else if ((op1 & 0xD208)==0x8200)
    f->Print("std    r%-2d,Z+%d\n",rd1,(op1&7)|((op1>>7)&0x18)|((op1>>8)&0x20));
 else if ((op1 & 0xFE0F)==0x9200)
    f->Print("sts    r%-2d,0x%04X     ; %d\n",rd1,op2,op2);
 else if ((op1 & 0xFC00)==0x1800)
    f->Print("sub    r%-2d,r%-2d\n",rd1,rr1);
 else if ((op1 & 0xF000)==0x5000)
    f->Print("subi   r%-2d,%d\n",rd3,k2);
 else if ((op1 & 0xFE0F)==0x9402)
    f->Print("swap   r%-2d\n",rd1);
 else if (op1==0x95A8)
    f->Print("wdr\n");
 else
    f->Print("**** unknown ****\n");
}

static
int ParseTracer(TextFile &f, char *s, int len, void *data)
{
 File *fo=(File *)data;
 unsigned address, op1, op2=0;
 int r=sscanf(s,"0x%04X 0x%04X 0x%04X",&address,&op1,&op2);
 if (r!=2 && r!=3)
   {
    fo->Write(s,len);
    fo->Write('\n');
    if (bSync)
       fo->Flush();
    return 0;
   }
 if (r==2)
    fo->Print("0x%04X 0x%04X        ",address,op1);
 else
    fo->Print("0x%04X 0x%04X 0x%04X ",address,op1,op2);
 Disassemble(fo,op1,op2);
 return 0;
}

int main(int argc, char *argv[])
{
 // Parse the command line
 clp.Run(argc,argv,&opts);
 int firstFile=clp.GetFirstFile();
 int numFiles=argc-firstFile;

 // Arguments validation
 if (numFiles>2)
    error.Abort("Extra arguments");
 const char *inputFName=NULL;
 const char *outputFName=NULL;
 if (numFiles>0)
   {
    inputFName=argv[firstFile++];
    if (numFiles>1)
       outputFName=argv[firstFile];
   }
 if (!inputFName && isatty(fileno(stdin)))
    error.Abort("Refusing to use a console as input");

 // Do the job
 TextFile fi(inputFName);
 File fo(outputFName,"wt");

 fi.Process(ParseTracer,error,&fo);

 return 0;
}

