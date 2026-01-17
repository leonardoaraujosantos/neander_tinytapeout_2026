"""
Debug test for LCC 02_locals program.
"""

import os
import sys
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from neander_assembly import NeanderLoader


LCC_SAMPLES_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'lcc_samples'
)


@cocotb.test()
async def test_debug_locals(dut):
    """Debug test for 02_locals - trace execution."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())

    dut.reset.value = 1
    dut.mem_load_en.value = 0
    dut.mem_load_addr.value = 0
    dut.mem_load_data.value = 0
    dut.io_in.value = 0
    dut.io_status.value = 0
    dut.mem_read_addr.value = 0
    await ClockCycles(dut.clk, 5)

    loader = NeanderLoader(add_startup=True)
    asm_file = os.path.join(LCC_SAMPLES_DIR, "02_locals.s")
    program = loader.assemble_file(asm_file)
    main_addr = loader.get_symbol('_main')

    dut._log.info(f"Program loaded: {len(program)} bytes, _main at 0x{main_addr:04X}")
    dut._log.info(f"Startup: {[hex(b) for b in program[:4]]}")
    dut._log.info(f"_main first 32 bytes: {[hex(b) for b in program[main_addr:main_addr+32]]}")

    # Load program
    for i, byte in enumerate(program):
        dut.mem_load_en.value = 1
        dut.mem_load_addr.value = i
        dut.mem_load_data.value = byte
        await RisingEdge(dut.clk)
        dut.mem_load_en.value = 0
        await RisingEdge(dut.clk)

    # Reset
    dut.reset.value = 1
    await ClockCycles(dut.clk, 5)
    dut.reset.value = 0
    await ClockCycles(dut.clk, 2)

    # Trace execution
    last_pc = -1
    cycles = 0
    instruction_count = 0
    halt_counter = 0

    dut._log.info("Starting execution trace...")
    dut._log.info("Cycle | PC     | RI   | AC     | Y      | SP     | FP")
    dut._log.info("-" * 70)

    while cycles < 100000:
        await RisingEdge(dut.clk)
        cycles += 1

        try:
            pc = int(dut.dbg_pc.value) & 0xFFFF
            ac = int(dut.dbg_ac.value) & 0xFFFF
            y = int(dut.dbg_y.value) & 0xFFFF
            sp = int(dut.dbg_sp.value) & 0xFFFF
            fp = int(dut.dbg_fp.value) & 0xFFFF
            ri = int(dut.dbg_ri.value) & 0xFF
        except ValueError:
            continue

        # Log when PC changes (new instruction)
        if pc != last_pc:
            instruction_count += 1
            if instruction_count <= 80:  # Only log first 80 instructions
                dut._log.info(f"{cycles:5d} | 0x{pc:04X} | 0x{ri:02X} | 0x{ac:04X} | 0x{y:04X} | 0x{sp:04X} | 0x{fp:04X}")
            last_pc = pc
            halt_counter = 0
        else:
            # Check for HLT (PC stays same)
            if (ri & 0xF0) == 0xF0:
                halt_counter += 1
                if halt_counter > 10:
                    dut._log.info(f"HLT detected at PC=0x{pc:04X}")
                    dut._log.info(f"Final state: AC={ac} (0x{ac:04X}), Y={y} (0x{y:04X})")
                    dut._log.info(f"Expected: AC=300, Y=0 (or AC=44, Y=1 for 16-bit 300)")

                    # 300 in 16-bit could be: Y:AC = 0:300 (0x012C)
                    total = (y << 16) + ac
                    dut._log.info(f"Total Y:AC = {total}")

                    if ac == 300 or (y == 1 and ac == 44):
                        dut._log.info("PASSED!")
                    else:
                        dut._log.error(f"FAILED: Expected 300, got AC={ac}")
                    return

        if instruction_count > 150:
            dut._log.warning("Too many instructions, stopping")
            break

    dut._log.info(f"Executed {instruction_count} instructions in {cycles} cycles")
    raise TimeoutError("Did not halt properly")
