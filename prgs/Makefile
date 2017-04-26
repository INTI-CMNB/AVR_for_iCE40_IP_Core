OBJDIR=../Work
LIB=avr
GHDLFLAGS=--work=$(LIB) --workdir=$(OBJDIR)

.PHONY: t0_avr2 wbtest hello

# Library Modules
LIB_MODS_1=hello
LIB_MODS=$(addsuffix .o,$(addprefix $(OBJDIR)/,$(LIB_MODS_1)))
LIB_SRCS=$(addsuffix .vhdl,$(addprefix $(OBJDIR)/,$(LIB_MODS_1)))

$(OBJDIR)/%.o : $(OBJDIR)/%.vhdl
	ghdl -a $(GHDLFLAGS) -o $@ $<

all: lib

$(OBJDIR):
	mkdir $(OBJDIR)

$(OBJDIR)/pm_pkg.vhdl: pm_pkg.in.vhdl $(LIB_SRCS)
	vhdlspp.pl $< $@

$(OBJDIR)/hello.vhdl: hello

t0_avr2:
	$(MAKE) -C $@

wbtest:
	$(MAKE) -C $@

hello:
	$(MAKE) -C $@

needed: $(OBJDIR) t0_avr2 wbtest hello $(OBJDIR)/pm_pkg.vhdl

lib: needed $(LIB_MODS) $(OBJDIR)/pm_pkg.o

