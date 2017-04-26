OBJDIR=../../Work
PRJ=hello

all: $(OBJDIR) $(OBJDIR)/$(PRJ).vhdl

$(OBJDIR):
	mkdir $(OBJDIR)

%.lst: $(OBJDIR)/%.elf
	avr-objdump -h -S $< > $@

$(OBJDIR)/$(PRJ).elf: hello.c
	avr-gcc -O3 -mmcu=attiny25 $< -o $@

$(OBJDIR)/$(PRJ).bin: $(OBJDIR)/$(PRJ).elf
	avr-objcopy -O binary $< $@

$(OBJDIR)/$(PRJ).dat: $(OBJDIR)/$(PRJ).bin
	bin2hex --vhdl -b 2 $< $@

$(OBJDIR)/$(PRJ).vhdl: ../pm_s.in.vhdl $(OBJDIR)/$(PRJ).dat
	printf $(PRJ)PM > entity.txt
	ln -s $(OBJDIR)/$(PRJ).dat rom.dat
	vhdlspp.pl $< $@
	rm entity.txt rom.dat


