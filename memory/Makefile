OBJDIR=../Work
LIB=avr
GHDLFLAGS=--work=$(LIB) --workdir=$(OBJDIR)

# Library Modules
LIB_MODS_1=dm_s
LIB_MODS=$(addsuffix .o,$(addprefix $(OBJDIR)/,$(LIB_MODS_1)))
LIB_SRCS=$(addsuffix .vhdl,$(LIB_MODS_1))

$(OBJDIR)/%.o : %.vhdl
	ghdl -a $(GHDLFLAGS) -o $@ $<

$(OBJDIR)/%.o : $(OBJDIR)/%.vhdl
	ghdl -a $(GHDLFLAGS) -o $@ $<

all: lib

$(OBJDIR):
	mkdir $(OBJDIR)

$(OBJDIR)/mem_pkg.o: $(OBJDIR)/avr_pkg.o
$(OBJDIR)/dm_s.o: $(OBJDIR)/avr_pkg.o

$(OBJDIR)/avr_pkg.o:
	$(MAKE) -C ../core

$(OBJDIR)/mem_pkg.vhdl: mem_pkg.in.vhdl $(LIB_SRCS)
	vhdlspp.pl $< $@

needed: $(OBJDIR) $(OBJDIR)/mem_pkg.vhdl

lib: needed $(LIB_MODS) $(OBJDIR)/mem_pkg.o

