"""
Minimal test to debug LCC hello program behavior.
"""

import os
import sys
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'cocotb_tests'))
from neander_common import NeanderTestbench, Op


@cocotb.test()
async def test_minimal_hello_mimic(dut):
    """Test that mimics LCC hello program - CALL main, main returns 42."""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # This is exactly what our assembled LCC hello program does:
    # 0x0000: CALL 0x0100
    # 0x0003: HLT
    # 0x0100: main function (prologue, return 42, epilogue)

    # Startup at 0x0000
    startup = [
        0x72, 0x00, 0x01,  # CALL 0x0100
        0xF0,              # HLT
    ]

    # main at 0x0100:
    # PUSH_FP, TSF, LDI 42, PUSH, LDI 0, TAY, POP, TFS, POP_FP, RET
    main_func = [
        0x0C,              # PUSH_FP
        0x0A,              # TSF (FP = SP)
        0xE0, 0x2A, 0x00,  # LDI 42 (16-bit immediate)
        0x70,              # PUSH
        0xE0, 0x00, 0x00,  # LDI 0 (16-bit immediate)
        0x03,              # TAY (Y = 0)
        0x71,              # POP (AC = 42)
        0x0B,              # TFS (SP = FP)
        0x0D,              # POP_FP
        0x73,              # RET
    ]

    # Load startup
    for i, byte in enumerate(startup):
        await tb.load_byte(i, byte)

    # Load main at 0x0100
    for i, byte in enumerate(main_func):
        await tb.load_byte(0x0100 + i, byte)

    await tb.reset()

    # Run until halt with tracing
    last_pc = -1
    cycles = 0
    max_cycles = 100000

    while cycles < max_cycles:
        await RisingEdge(dut.clk)
        cycles += 1

        try:
            pc = int(dut.dbg_pc.value) & 0xFFFF
            ac = int(dut.dbg_ac.value) & 0xFFFF
            y = int(dut.dbg_y.value) & 0xFFFF
            ri = int(dut.dbg_ri.value) & 0xFF
            sp = int(dut.dbg_sp.value) & 0xFFFF
            fp = int(dut.dbg_fp.value) & 0xFFFF
        except ValueError:
            continue

        if pc != last_pc:
            # Log state transitions (first 30)
            if cycles < 30000:
                dut._log.info(f"Cycle {cycles:5d}: PC=0x{pc:04X} RI=0x{ri:02X} AC={ac:5d} Y={y:5d} SP=0x{sp:04X} FP=0x{fp:04X}")
            last_pc = pc

        # Check for HLT - RI shows HLT opcode (0xF0), PC has advanced past it
        if (ri & 0xF0) == 0xF0 and pc > 0x0003:
            await ClockCycles(dut.clk, 20)  # Wait for stable state
            ac = int(dut.dbg_ac.value) & 0xFFFF
            y = int(dut.dbg_y.value) & 0xFFFF
            dut._log.info(f"HLT reached at PC=0x{pc:04X}: AC={ac}, Y={y}")
            assert ac == 42, f"Expected AC=42, got {ac}"
            dut._log.info("PASSED!")
            return

    raise TimeoutError(f"Did not halt within {max_cycles} cycles")


@cocotb.test()
async def test_simple_call_ret_with_frame(dut):
    """Simpler test - CALL function that just returns a value."""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # 0x0000: CALL 0x0010, HLT
    # 0x0010: LDI 42, RET (no frame pointer)
    program = [
        0x72, 0x10, 0x00,  # CALL 0x0010
        0xF0,              # HLT
    ]

    # Pad to 0x0010
    while len(program) < 0x10:
        program.append(0x00)

    # Function at 0x0010
    program.extend([
        0xE0, 0x2A, 0x00,  # LDI 42
        0x73,              # RET
    ])

    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    await tb.reset()

    cycles = await tb.run_until_halt(max_cycles=50000)
    ac = int(dut.dbg_ac.value) & 0xFFFF
    dut._log.info(f"Halted after {cycles} cycles, AC={ac}")
    assert ac == 42, f"Expected AC=42, got {ac}"
    dut._log.info("PASSED!")
