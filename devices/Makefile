OBJDIR=../Work
LIB=avr
GHDLFLAGS=--work=$(LIB) --workdir=$(OBJDIR) -P../../../SPI/Work/

# Library Modules
LIB_MODS_1=portx wb_ctrl irq spi
LIB_MODS=$(addsuffix .o,$(addprefix $(OBJDIR)/,$(LIB_MODS_1)))
LIB_SRCS=$(addsuffix .vhdl,$(LIB_MODS_1))

$(OBJDIR)/%.o : %.vhdl
	ghdl -a $(GHDLFLAGS) -o $@ $<

$(OBJDIR)/%.o : $(OBJDIR)/%.vhdl
	ghdl -a $(GHDLFLAGS) -o $@ $<

all: lib

$(OBJDIR):
	mkdir $(OBJDIR)

$(OBJDIR)/dev_pkg.o: $(OBJDIR)/avr_pkg.o
$(OBJDIR)/portx.o: $(OBJDIR)/avr_pkg.o
$(OBJDIR)/wb_ctrl.o: $(OBJDIR)/avr_pkg.o
$(OBJDIR)/irq.o: $(OBJDIR)/avr_pkg.o
$(OBJDIR)/spi.o: $(OBJDIR)/avr_pkg.o

$(OBJDIR)/avr_pkg.o:
	$(MAKE) -C ../core

$(OBJDIR)/dev_pkg.vhdl: dev_pkg.in.vhdl $(LIB_SRCS)
	vhdlspp.pl $< $@

needed: $(OBJDIR) $(OBJDIR)/dev_pkg.vhdl

lib: needed $(LIB_MODS) $(OBJDIR)/dev_pkg.o


