OBJDIR=../Work
SPIPATH=../../../SPI
UTILSPATH=../../../utils
GHDLFLAGS=--work=avr --workdir=$(OBJDIR) -P../Work -P../../../utils/Work/ \
	-P../../../wb_handler/Work/ -P../../../mems/Work/ -P../../../miniuart/Work/ \
	-P$(SPIPATH)/Work
CPPFIO=../../../../fpgatools/cppfio/src
CFLAGS=-O2 -Wall -Werror -g3
CXXFLAGS=$(CFLAGS) -I$(CPPFIO) -L$(CPPFIO)

all: $(OBJDIR) $(OBJDIR)/alu_tb $(OBJDIR)/cpu_rl_tb $(OBJDIR)/attX5_tb

$(OBJDIR):
	mkdir $(OBJDIR)

$(OBJDIR)/%.o : %.vhdl
	ghdl -a $(GHDLFLAGS) -o $@ $<

$(OBJDIR)/%.o : $(OBJDIR)/%.vhdl
	ghdl -a $(GHDLFLAGS) -o $@ $<

$(OBJDIR)/%.o : ../prgs/hello/%.vhdl
	ghdl -a $(GHDLFLAGS) -o $@ $<

$(OBJDIR)/%.o : ../prgs/%.vhdl
	ghdl -a $(GHDLFLAGS) -o $@ $<

$(OBJDIR)/cpu_rl_tb.o: $(OBJDIR)/avr_pkg.o $(OBJDIR)/tracer.o $(OBJDIR)/attX5.o
$(OBJDIR)/attX5_tb.o: $(OBJDIR)/pm_pkg.o $(OBJDIR)/micros_pkg.o $(OBJDIR)/avr_pkg.o \
	$(OBJDIR)/tracer.o $(OBJDIR)/attX5.o $(OBJDIR)/core.o
$(OBJDIR)/tracer.o: $(OBJDIR)/avr_pkg.o

$(OBJDIR)/attX5_tb: $(OBJDIR)/hello.o $(OBJDIR)/attX5_tb.o
	ghdl -e $(GHDLFLAGS) -o $@ ATtX5_Tb

$(OBJDIR)/cpu_rl_tb: $(OBJDIR)/cpu_rl_tb.o
	ghdl -e $(GHDLFLAGS) -o $@ CPU_RL_Tb

$(OBJDIR)/alu_tb.o: $(OBJDIR)/avr_pkg.o $(OBJDIR)/cpu.o

$(OBJDIR)/alu_tb: $(OBJDIR)/alu_tb.o
	ghdl -e $(GHDLFLAGS) -o $@ alu_tb

lib:
	$(MAKE) -C ../ lib

test: lib prgs all test_cpu_rl test_attX5

prgs:
	$(MAKE) -C ../prgs

roms_avr2:
	$(MAKE) -C ../prgs
	# Use simple names for the ROM loader
	ln -fs $(OBJDIR)/tldi_out.hex $(OBJDIR)/x1.hex
	ln -fs $(OBJDIR)/tadd_1.hex $(OBJDIR)/x2.hex
	ln -fs $(OBJDIR)/tsub_1.hex $(OBJDIR)/x3.hex
	ln -fs $(OBJDIR)/tlogic_1.hex $(OBJDIR)/x4.hex
	ln -fs $(OBJDIR)/timm_1.hex $(OBJDIR)/x5.hex
	ln -fs $(OBJDIR)/tcom_1.hex $(OBJDIR)/x6.hex
	ln -fs $(OBJDIR)/tneg_1.hex $(OBJDIR)/x7.hex
	ln -fs $(OBJDIR)/tinc_1.hex $(OBJDIR)/x8.hex
	ln -fs $(OBJDIR)/tlsr_1.hex $(OBJDIR)/x9.hex
	ln -fs $(OBJDIR)/tror_1.hex $(OBJDIR)/xa.hex
	ln -fs $(OBJDIR)/tasr_1.hex $(OBJDIR)/xb.hex
	ln -fs $(OBJDIR)/tswap_mov_1.hex $(OBJDIR)/xc.hex
	ln -fs $(OBJDIR)/tadiw_1.hex $(OBJDIR)/xd.hex
	ln -fs $(OBJDIR)/tsbiw_1.hex $(OBJDIR)/xe.hex
	ln -fs $(OBJDIR)/tlds_sts_1.hex $(OBJDIR)/xf.hex
	ln -fs $(OBJDIR)/tld_st_1.hex $(OBJDIR)/xg.hex
	ln -fs $(OBJDIR)/tld_st_2.hex $(OBJDIR)/xh.hex
	ln -fs $(OBJDIR)/trcall_1.hex $(OBJDIR)/xi.hex
	ln -fs $(OBJDIR)/ticall_1.hex $(OBJDIR)/xj.hex
	ln -fs $(OBJDIR)/trjmp_1.hex $(OBJDIR)/xk.hex
	ln -fs $(OBJDIR)/tijmp_1.hex $(OBJDIR)/xl.hex
	ln -fs $(OBJDIR)/tbrbx_bset_1.hex $(OBJDIR)/xm.hex
	ln -fs $(OBJDIR)/tsbix_sbi_1.hex $(OBJDIR)/xn.hex
	ln -fs $(OBJDIR)/tsbrx_1.hex $(OBJDIR)/xo.hex
	ln -fs $(OBJDIR)/tcp_1.hex $(OBJDIR)/xp.hex
	ln -fs $(OBJDIR)/tbst_bld_1.hex $(OBJDIR)/xq.hex
	ln -fs $(OBJDIR)/tpush_pop.hex $(OBJDIR)/xr.hex
	ln -fs $(OBJDIR)/tlpm_1.hex $(OBJDIR)/xs.hex
	ln -fs $(OBJDIR)/tirq_1.hex $(OBJDIR)/xt.hex
	ln -fs $(OBJDIR)/t0_avr2.hex $(OBJDIR)/xu.hex
	ln -fs $(OBJDIR)/tlds_sts_2.hex $(OBJDIR)/xv.hex
	ln -fs $(OBJDIR)/trjmp_2.hex $(OBJDIR)/xw.hex
	ln -fs $(OBJDIR)/wbtest.hex $(OBJDIR)/xx.hex
	ln -fs $(OBJDIR)/tlpm_spm_1.hex $(OBJDIR)/xy.hex

$(OBJDIR)/avr-disa: avr-disa.cc
	$(CXX) $(CXXFLAGS) -o $@ $< `$(CPPFIO)/cppfio_config --dlibs`

do_test_cpu_rl: lib all test_cpu_rl

test_cpu_rl: $(OBJDIR)/cpu_rl_tb roms_avr2 $(OBJDIR)/avr-disa
	cd $(OBJDIR) ; $< --ieee-asserts=disable --wave=$<.ghw | $(OBJDIR)/avr-disa -qs

$(OBJDIR)/gen_alu_test: gen_alu_test.cc
	$(CXX) -O3 -DNO_AVR4 -o $@ $<

do_test_attX5: lib prgs all test_attX5

test_attX5: $(OBJDIR)/attX5_tb $(OBJDIR)/avr-disa
	$(OBJDIR)/attX5_tb --wave=$(OBJDIR)/attX5_tb.ghw --ieee-asserts=disable > $(OBJDIR)/attX5.log
	cat $(OBJDIR)/attX5.log | $(OBJDIR)/avr-disa > $(OBJDIR)/attX5.asm

do_test_alu: lib all test_alu

test_alu: $(OBJDIR)/gen_alu_test $(OBJDIR)/alu_tb
	time $(OBJDIR)/gen_alu_test | $(OBJDIR)/alu_tb --ieee-asserts=disable-at-0 --wave=$(OBJDIR)/alu_tb.ghw

clean:

