# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer

# Simulated 32-byte external RAM
RAM = [0] * 32


def safe_int(value, default=0):
    """Safely convert a logic value to int, returning default if it contains X/Z."""
    try:
        return int(value)
    except ValueError:
        return default


def reset_ram():
    """Reset RAM and load a simple test program."""
    global RAM
    RAM = [0] * 32

    # Program: LDI 5, ADD 0x10, OUT 0, HLT
    # Expected result: AC = 5 + 3 = 8, output to port 0
    RAM[0x00] = 0xE0  # LDI (opcode 0xE)
    RAM[0x01] = 0x05  # immediate value = 5
    RAM[0x02] = 0x30  # ADD (opcode 0x3)
    RAM[0x03] = 0x10  # address = 0x10
    RAM[0x04] = 0xD0  # OUT (opcode 0xD)
    RAM[0x05] = 0x00  # port 0
    RAM[0x06] = 0xF0  # HLT (opcode 0xF)

    # Data at address 0x10
    RAM[0x10] = 0x03  # value = 3


async def ram_model(dut):
    """
    Simulates external RAM connected to the CPU.
    Responds combinationally to memory read/write signals.
    """
    while True:
        # Small delay for combinational response
        await Timer(1, unit="ns")

        # Extract signals from uo_out
        uo_val = safe_int(dut.uo_out.value, 0)
        addr = uo_val & 0x1F  # bits [4:0] = address
        we = (uo_val >> 5) & 1  # bit 5 = write enable
        oe = (uo_val >> 6) & 1  # bit 6 = output enable

        uio_oe_val = safe_int(dut.uio_oe.value, 0)

        if we and uio_oe_val == 0xFF:
            # Write: CPU -> RAM (data on uio_out when uio_oe is high)
            RAM[addr] = safe_int(dut.uio_out.value, 0)
        elif oe:
            # Read: RAM -> CPU (put data on uio_in)
            dut.uio_in.value = RAM[addr]


@cocotb.test()
async def test_neander_ldi_add_out(dut):
    """Test LDI, ADD, and OUT instructions."""
    dut._log.info("Start Neander-X CPU Test")

    # Initialize RAM with test program
    reset_ram()

    # Set the clock period to 10 us (100 KHz)
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    # Start RAM model
    cocotb.start_soon(ram_model(dut))

    # Reset
    dut._log.info("Reset")
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    dut._log.info("Running program: LDI 5, ADD [0x10], OUT 0, HLT")

    # Track IO write strobe
    io_output = None

    # Run for enough cycles to execute the program
    for cycle in range(200):
        await RisingEdge(dut.clk)

        # Check for IO write strobe (bit 7 of uo_out)
        uo_val = safe_int(dut.uo_out.value, 0)
        io_write = (uo_val >> 7) & 1
        if io_write:
            # Capture output value from uio_out
            io_output = safe_int(dut.uio_out.value, 0)
            dut._log.info(f"IO Write detected at cycle {cycle}! Output value: {io_output}")

    # Verify the output
    assert io_output is not None, "No IO write detected - program may not have executed correctly"
    assert io_output == 8, f"Expected output 8 (5+3), got {io_output}"

    dut._log.info("Test PASSED: Output value is correct (5 + 3 = 8)")


@cocotb.test()
async def test_neander_sta_lda(dut):
    """Test STA and LDA instructions."""
    dut._log.info("Start STA/LDA Test")

    # Initialize RAM
    global RAM
    RAM = [0] * 32

    # Program: LDI 42, STA 0x1F, LDI 0, LDA 0x1F, OUT 0, HLT
    # Store 42 at address 0x1F, clear AC, load it back, output
    RAM[0x00] = 0xE0  # LDI
    RAM[0x01] = 0x2A  # 42
    RAM[0x02] = 0x10  # STA
    RAM[0x03] = 0x1F  # addr 0x1F
    RAM[0x04] = 0xE0  # LDI
    RAM[0x05] = 0x00  # 0
    RAM[0x06] = 0x20  # LDA
    RAM[0x07] = 0x1F  # addr 0x1F
    RAM[0x08] = 0xD0  # OUT
    RAM[0x09] = 0x00  # port 0
    RAM[0x0A] = 0xF0  # HLT

    # Set the clock period to 10 us (100 KHz)
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    # Start RAM model
    cocotb.start_soon(ram_model(dut))

    # Reset
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    dut._log.info("Running program: LDI 42, STA 0x1F, LDI 0, LDA 0x1F, OUT 0, HLT")

    io_output = None

    for cycle in range(300):
        await RisingEdge(dut.clk)

        uo_val = safe_int(dut.uo_out.value, 0)
        io_write = (uo_val >> 7) & 1
        if io_write:
            io_output = safe_int(dut.uio_out.value, 0)
            dut._log.info(f"IO Write detected at cycle {cycle}! Output value: {io_output}")

    assert io_output is not None, "No IO write detected"
    assert io_output == 42, f"Expected output 42, got {io_output}"

    dut._log.info("Test PASSED: STA/LDA working correctly")


@cocotb.test()
async def test_neander_jump(dut):
    """Test JMP instruction."""
    dut._log.info("Start JMP Test")

    global RAM
    RAM = [0] * 32

    # Program: JMP 0x10, HLT, ..., (at 0x10) LDI 99, OUT 0, HLT
    RAM[0x00] = 0x80  # JMP
    RAM[0x01] = 0x10  # addr 0x10
    RAM[0x02] = 0xF0  # HLT (should be skipped)

    RAM[0x10] = 0xE0  # LDI
    RAM[0x11] = 0x63  # 99
    RAM[0x12] = 0xD0  # OUT
    RAM[0x13] = 0x00  # port 0
    RAM[0x14] = 0xF0  # HLT

    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())
    cocotb.start_soon(ram_model(dut))

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    dut._log.info("Running program: JMP 0x10, then LDI 99, OUT 0, HLT")

    io_output = None

    for cycle in range(200):
        await RisingEdge(dut.clk)

        uo_val = safe_int(dut.uo_out.value, 0)
        io_write = (uo_val >> 7) & 1
        if io_write:
            io_output = safe_int(dut.uio_out.value, 0)
            dut._log.info(f"IO Write detected at cycle {cycle}! Output value: {io_output}")

    assert io_output is not None, "No IO write detected - JMP may have failed"
    assert io_output == 99, f"Expected output 99, got {io_output}"

    dut._log.info("Test PASSED: JMP working correctly")
