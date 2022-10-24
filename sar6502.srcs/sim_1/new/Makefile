TEST_HARNESS_PROJECT:=$(shell realpath ../../../../test_harness)
USBPORT=/dev/ttyUSB0

test_program.mem: test_program_wdc.mem
	cp -a $< $@

test_program_wdc.mem: test_program.s
	$(MAKE) -f $(TEST_HARNESS_PROJECT)/test_program/Makefile BASEDIR=$(TEST_HARNESS_PROJECT)/test_program $@

test_plan.mem: test_program.mem
	PYTHONPATH=$(TEST_HARNESS_PROJECT)/host $(TEST_HARNESS_PROJECT)/host/gen_test_plan $< $@.tmp -p $(USBPORT)
	mv $@.tmp $@

clean:
	$(RM) *.mem *.lst *.out

generate: test_plan.mem
.PHONY: generate clean
