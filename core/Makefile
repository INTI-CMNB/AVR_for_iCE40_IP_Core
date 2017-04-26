OBJDIR=../Work
LIB=avr
GHDLFLAGS=--work=$(LIB) --workdir=$(OBJDIR)

# Library Modules
LIB_MODS_1=reg_file core alu decoder cpu io_reg_file
LIB_MODS=$(addsuffix .o,$(addprefix $(OBJDIR)/,$(LIB_MODS_1)))
LIB_SRCS=$(addsuffix .vhdl,$(LIB_MODS_1))

$(OBJDIR)/%.o : %.vhdl
	ghdl -a $(GHDLFLAGS) -o $@ $<

$(OBJDIR)/%.o : $(OBJDIR)/%.vhdl
	ghdl -a $(GHDLFLAGS) -o $@ $<

all: lib

$(OBJDIR):
	mkdir $(OBJDIR)

$(OBJDIR)/core.o: $(OBJDIR)/avr_pkg.o $(OBJDIR)/cpu.o $(OBJDIR)/io_reg_file.o
$(OBJDIR)/reg_file.o: $(OBJDIR)/avr_pkg.o
$(OBJDIR)/alu.o: $(OBJDIR)/avr_pkg.o
$(OBJDIR)/decoder.o: $(OBJDIR)/avr_pkg.o
$(OBJDIR)/io_reg_file.o: $(OBJDIR)/avr_pkg.o
$(OBJDIR)/cpu.o: $(OBJDIR)/avr_pkg.o $(OBJDIR)/alu.o $(OBJDIR)/decoder.o $(OBJDIR)/reg_file.o

$(OBJDIR)/avr_pkg.vhdl: avr_pkg.in.vhdl $(LIB_SRCS)
	vhdlspp.pl $< $@

needed: $(OBJDIR) $(OBJDIR)/avr_pkg.vhdl

lib: needed $(LIB_MODS)

