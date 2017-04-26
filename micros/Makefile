OBJDIR=../Work
LIB=avr
GHDLFLAGS=--work=$(LIB) --workdir=$(OBJDIR)

# Library Modules
LIB_MODS_1=attX5
LIB_MODS=$(addsuffix .o,$(addprefix $(OBJDIR)/,$(LIB_MODS_1)))
LIB_SRCS=$(addsuffix .vhdl,$(LIB_MODS_1))

$(OBJDIR)/%.o : %.vhdl
	ghdl -a $(GHDLFLAGS) -o $@ $<

$(OBJDIR)/%.o : $(OBJDIR)/%.vhdl
	ghdl -a $(GHDLFLAGS) -o $@ $<

all: lib

$(OBJDIR):
	mkdir $(OBJDIR)

$(OBJDIR)/attX5.o: $(OBJDIR)/avr_pkg.o $(OBJDIR)/mem_pkg.o \
	$(OBJDIR)/dev_pkg.o $(OBJDIR)/core.o $(OBJDIR)/spi.o $(OBJDIR)/portx.o

$(OBJDIR)/micros_pkg.o: $(OBJDIR)/avr_pkg.o

$(OBJDIR)/avr_pkg.o:
	$(MAKE) -C ../core

$(OBJDIR)/mem_pkg.o:
	$(MAKE) -C ../memory

$(OBJDIR)/dev_pkg.o:
	$(MAKE) -C ../devices

$(OBJDIR)/micros_pkg.vhdl: micros_pkg.in.vhdl $(LIB_SRCS)
	vhdlspp.pl $< $@

needed: $(OBJDIR) $(OBJDIR)/micros_pkg.vhdl

lib: needed $(LIB_MODS) $(OBJDIR)/micros_pkg.o

