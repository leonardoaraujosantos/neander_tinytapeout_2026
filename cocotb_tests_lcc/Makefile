# Makefile for LCC-generated program tests
# Tests compile C programs with LCC, assemble with neanderasm, and run on CPU

# Defaults
SIM ?= icarus
TOPLEVEL_LANG ?= verilog

# Source files (order matters for Icarus Verilog)
VERILOG_SOURCES += $(PWD)/../src/neander_x_alu.sv
VERILOG_SOURCES += $(PWD)/../src/neander_x_datapath.sv
VERILOG_SOURCES += $(PWD)/../src/neander_x_control_unit.sv
VERILOG_SOURCES += $(PWD)/../src/sequential_divider.sv
VERILOG_SOURCES += $(PWD)/../src/sequential_multiplier.sv
VERILOG_SOURCES += $(PWD)/../src/top_cpu_neander_x.sv
VERILOG_SOURCES += $(PWD)/../src/spi_memory_controller.sv
VERILOG_SOURCES += $(PWD)/../src/spi_sram_model.sv
VERILOG_SOURCES += $(PWD)/../cocotb_tests/neander_tb_wrapper.sv

# Top level module (the testbench wrapper)
TOPLEVEL = neander_tb_wrapper

# Python test module
MODULE ?= test_lcc_programs

# Include cocotb makefiles
include $(shell cocotb-config --makefiles)/Makefile.sim

# Extra flags for SystemVerilog
ifeq ($(SIM),icarus)
    COMPILE_ARGS += -g2012
endif

ifeq ($(SIM),verilator)
    EXTRA_ARGS += --timing
    EXTRA_ARGS += -Wno-WIDTHEXPAND
    EXTRA_ARGS += -Wno-WIDTHTRUNC
endif

# Python path for neander_assembly module
export PYTHONPATH := $(PWD)/..

# Clean target
clean::
	rm -rf __pycache__
	rm -rf sim_build
	rm -rf results.xml
	rm -f *.vcd
