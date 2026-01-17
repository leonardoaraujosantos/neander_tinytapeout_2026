"""
Test LCC-compiled samples using cocotb and the Neander-X simulator.
"""
import cocotb
from cocotb.triggers import Timer, ClockCycles, RisingEdge
import os
import sys

# Add paths
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'neander_assembly'))

from neander_common import NeanderTestbench
from neander_assembly import NeanderLoader


# Directory containing LCC-compiled samples
SAMPLES_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'lcc_samples')


async def run_lcc_sample(dut, sample_name, expected_result, max_cycles=50000):
    """
    Run an LCC-compiled sample and verify the result.

    Args:
        dut: Device under test
        sample_name: Name of the sample (e.g., '01_hello')
        expected_result: Expected return value in AC after program completes
        max_cycles: Maximum cycles before timeout

    Returns:
        int: Actual result value
    """
    # Initialize testbench
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Load and assemble the sample
    sample_path = os.path.join(SAMPLES_DIR, f'{sample_name}.s')
    cocotb.log.info(f"Loading sample: {sample_path}")

    loader = NeanderLoader(add_startup=True)
    binary = loader.assemble_file(sample_path)

    cocotb.log.info(f"Assembled {len(binary)} bytes")

    # Load program byte by byte
    for addr, byte_val in enumerate(binary):
        await tb.load_byte(addr, byte_val)

    # Release reset to start execution
    await tb.reset()

    # Run until HLT or timeout with debugging
    from cocotb.triggers import RisingEdge
    for cycle in range(max_cycles):
        await RisingEdge(dut.clk)
        pc = int(dut.dbg_pc.value)
        ri = int(dut.dbg_ri.value)
        ac = int(dut.dbg_ac.value)
        sp = int(dut.dbg_sp.value)
        fp = int(dut.dbg_fp.value)

        if cycle < 200 or cycle % 1000 == 0:
            cocotb.log.info(f"Cycle {cycle}: PC={pc:04X} RI={ri:02X} AC={ac:04X} SP={sp:04X} FP={fp:04X}")

        # Check for HLT (0xF0)
        if ri == 0xF0:
            cocotb.log.info(f"HLT detected at cycle {cycle}")
            break
    else:
        raise TimeoutError(f"CPU did not halt within {max_cycles} cycles")

    # Get AC value after halt
    ac_value = int(dut.dbg_ac.value)

    cocotb.log.info(f"Program halted with AC = {ac_value} (0x{ac_value:04X})")

    return ac_value


@cocotb.test()
async def test_01_hello(dut):
    """Test 01_hello: Should return 42"""
    result = await run_lcc_sample(dut, '01_hello', 42)
    assert result == 42, f"Expected 42, got {result}"


@cocotb.test()
async def test_02_locals(dut):
    """Test 02_locals: Should return 300"""
    result = await run_lcc_sample(dut, '02_locals', 300)
    assert result == 300, f"Expected 300, got {result}"


@cocotb.test()
async def test_03_arithmetic(dut):
    """Test 03_arithmetic: Should return 100"""
    result = await run_lcc_sample(dut, '03_arithmetic', 100)
    assert result == 100, f"Expected 100, got {result}"


@cocotb.test()
async def test_04_globals(dut):
    """Test 04_globals: Should return 15"""
    result = await run_lcc_sample(dut, '04_globals', 15)
    assert result == 15, f"Expected 15, got {result}"


@cocotb.test()
async def test_05_loop(dut):
    """Test 05_loop: sum_to_n(10) = 55"""
    result = await run_lcc_sample(dut, '05_loop', 55, max_cycles=100000)
    assert result == 55, f"Expected 55, got {result}"


@cocotb.test()
async def test_06_array(dut):
    """Test 06_array: Should return expected array sum (10+20+30+40+50=150)"""
    result = await run_lcc_sample(dut, '06_array', 150, max_cycles=100000)
    assert result == 150, f"Expected 150, got {result}"


@cocotb.test()
async def test_07_factorial(dut):
    """Test 07_factorial: factorial(5) = 120"""
    result = await run_lcc_sample(dut, '07_factorial', 120, max_cycles=100000)
    assert result == 120, f"Expected 120, got {result}"
