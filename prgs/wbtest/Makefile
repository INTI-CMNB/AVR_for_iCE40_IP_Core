OBJDIR=../../Work
PRJ=wbtest

all: $(OBJDIR)/$(PRJ).hex

$(OBJDIR)/$(PRJ).hex: $(PRJ).asm
	cd $(OBJDIR); ln -fs ../prgs/$(PRJ)/$< $<
	avra -l $(OBJDIR)/$<.lst $(OBJDIR)/$<

