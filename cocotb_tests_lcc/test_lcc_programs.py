"""
LCC Program Tests for Neander-X CPU
====================================

Tests for programs compiled with LCC compiler and assembled with neanderasm.
"""

import os
import sys
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

# Add paths for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'cocotb_tests'))

from neander_assembly import NeanderLoader, AssemblyError
from neander_common import NeanderTestbench, Op


# =============================================================================
# Configuration
# =============================================================================

LCC_SAMPLES_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'lcc_samples'
)

EXPECTED_RESULTS = {
    '01_hello': 42,
    '02_locals': 300,
    '03_arithmetic': 100,
    '04_globals': 15,
    '05_loop': 55,
    '06_array': 150,
    '07_factorial': 120,
    '08_fibonacci': 55,
    '09_bitwise': 8190,
    '10_char': 145,
    '12_division': 23,
    '14_negative': 50,
    '15_array_idx': 45,
    '16_compare': 6,
    '17_multiarg': 150,
    '18_nested': 120,
    '19_bitwise2': 170,
    '20_ifelse': 100,
    '95_test_all3ops': 8190,
    '96_test_xor': 4080,
    '97_test_bitops': 4110,
    '98_test_3calls': 8190,
    '99_test_add3': 8190,
}


# =============================================================================
# LCC Testbench (extends NeanderTestbench)
# =============================================================================

class LCCTestbench(NeanderTestbench):
    """Extended testbench for LCC programs."""

    def __init__(self, dut):
        super().__init__(dut)
        self.loader = NeanderLoader(add_startup=False)  # LCC generates its own startup

    async def load_lcc_program(self, filepath):
        """Assemble and load an LCC assembly file."""
        program = self.loader.assemble_file(filepath)
        # Load without conversion - our assembler already produces 16-bit format
        for i, byte in enumerate(program):
            await self.load_byte(i, byte)
        return program

    def get_return_value(self):
        """Get 16-bit return value from AC register."""
        try:
            return int(self.dut.dbg_ac.value) & 0xFFFF
        except ValueError:
            return 0


# =============================================================================
# Test Helper
# =============================================================================

async def run_lcc_test(dut, program_name, expected_result, max_cycles=500000):
    """Run an LCC program test."""
    tb = LCCTestbench(dut)
    await tb.setup()

    asm_file = os.path.join(LCC_SAMPLES_DIR, f"{program_name}.s")
    dut._log.info(f"Testing {program_name}: expecting {expected_result}")

    try:
        program = await tb.load_lcc_program(asm_file)
        main_addr = tb.loader.get_symbol('_main')
        dut._log.info(f"Loaded {len(program)} bytes, _main at 0x{main_addr:04X}")
    except Exception as e:
        dut._log.error(f"Failed to load: {e}")
        raise

    await tb.reset()

    try:
        cycles = await tb.run_until_halt(max_cycles)
        dut._log.info(f"Halted after {cycles} cycles")
    except TimeoutError as e:
        pc = int(dut.dbg_pc.value) & 0xFFFF
        ac = int(dut.dbg_ac.value) & 0xFFFF
        dut._log.error(f"Timeout: PC=0x{pc:04X}, AC={ac}")
        raise

    result = tb.get_return_value()
    dut._log.info(f"Result: {result} (expected: {expected_result})")

    assert result == expected_result, \
        f"{program_name}: Expected {expected_result}, got {result}"

    dut._log.info(f"PASSED: {program_name}")


# =============================================================================
# Tests
# =============================================================================

@cocotb.test()
async def test_01_hello(dut):
    """Test 01_hello.c - return 42"""
    await run_lcc_test(dut, '01_hello', 42)


@cocotb.test()
async def test_02_locals(dut):
    """Test 02_locals.c - local variables (100 + 200 = 300)"""
    await run_lcc_test(dut, '02_locals', 300)


@cocotb.test()
async def test_03_arithmetic(dut):
    """Test 03_arithmetic.c - function calls (100)"""
    await run_lcc_test(dut, '03_arithmetic', 100)


@cocotb.test()
async def test_04_globals(dut):
    """Test 04_globals.c - global variables (15)"""
    await run_lcc_test(dut, '04_globals', 15)


@cocotb.test()
async def test_05_loop(dut):
    """Test 05_loop.c - while loop (55)"""
    await run_lcc_test(dut, '05_loop', 55)


@cocotb.test()
async def test_06_array(dut):
    """Test 06_array.c - arrays (150)"""
    await run_lcc_test(dut, '06_array', 150)


@cocotb.test()
async def test_07_factorial(dut):
    """Test 07_factorial.c - factorial(5) = 120"""
    await run_lcc_test(dut, '07_factorial', 120)


@cocotb.test()
async def test_08_fibonacci(dut):
    """Test 08_fibonacci.c - fib(10) = 55 (recursive)"""
    await run_lcc_test(dut, '08_fibonacci', 55, max_cycles=10000000)


@cocotb.test()
async def test_09_bitwise(dut):
    """Test 09_bitwise.c - bitwise ops (8190)"""
    await run_lcc_test(dut, '09_bitwise', 8190)


@cocotb.test()
async def test_10_char(dut):
    """Test 10_char.c - char ops (145)"""
    await run_lcc_test(dut, '10_char', 145)


@cocotb.test()
async def test_12_division(dut):
    """Test 12_division.c - division and modulo (23)"""
    await run_lcc_test(dut, '12_division', 23)


@cocotb.test()
async def test_14_negative(dut):
    """Test 14_negative.c - negative numbers (50)"""
    await run_lcc_test(dut, '14_negative', 50)


@cocotb.test()
async def test_15_array_idx(dut):
    """Test 15_array_idx.c - larger array with constant indices (45)"""
    await run_lcc_test(dut, '15_array_idx', 45)


@cocotb.test()
async def test_16_compare(dut):
    """Test 16_compare.c - comparison operators (6)"""
    await run_lcc_test(dut, '16_compare', 6)


@cocotb.test()
async def test_17_multiarg(dut):
    """Test 17_multiarg.c - multiple function arguments (150)"""
    await run_lcc_test(dut, '17_multiarg', 150)


@cocotb.test()
async def test_18_nested(dut):
    """Test 18_nested.c - nested function calls (120)"""
    await run_lcc_test(dut, '18_nested', 120)


@cocotb.test()
async def test_19_bitwise2(dut):
    """Test 19_bitwise2.c - bitwise NOT and XOR (170)"""
    await run_lcc_test(dut, '19_bitwise2', 170)


@cocotb.test()
async def test_20_ifelse(dut):
    """Test 20_ifelse.c - if-else chains (100)"""
    await run_lcc_test(dut, '20_ifelse', 100)


@cocotb.test()
async def test_95_all3ops(dut):
    """Test 95_test_all3ops.c - all 3 bitwise ops (8190)"""
    await run_lcc_test(dut, '95_test_all3ops', 8190)


@cocotb.test()
async def test_96_xor(dut):
    """Test 96_test_xor.c - XOR operation (4080)"""
    await run_lcc_test(dut, '96_test_xor', 4080)


@cocotb.test()
async def test_97_bitops(dut):
    """Test 97_test_bitops.c - AND and OR operations (4110)"""
    await run_lcc_test(dut, '97_test_bitops', 4110)


@cocotb.test()
async def test_98_3calls(dut):
    """Test 98_test_3calls.c - 3 function calls returning fixed values (8190)"""
    await run_lcc_test(dut, '98_test_3calls', 8190)


@cocotb.test()
async def test_99_add3(dut):
    """Test 99_test_add3.c - simple addition of 3 values (8190)"""
    await run_lcc_test(dut, '99_test_add3', 8190)
