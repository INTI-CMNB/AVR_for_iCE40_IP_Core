.PHONY: core memory devices micros prgs

all: dirs lib

dirs: Work

Work:
	-mkdir $@

lib: core memory devices micros

core:
	$(MAKE) -C core

memory:
	$(MAKE) -C memory

devices:
	$(MAKE) -C devices

micros:
	$(MAKE) -C micros

prgs: dirs
	$(MAKE) -C prgs

needed:
	$(MAKE) -C core needed
	$(MAKE) -C memory needed
	$(MAKE) -C devices needed
	$(MAKE) -C micros needed
	$(MAKE) -C prgs needed

test:
	$(MAKE) -C testbench test

clean:
	-rm -rf Work

