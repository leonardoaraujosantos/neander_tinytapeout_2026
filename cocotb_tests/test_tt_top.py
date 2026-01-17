"""
TinyTapeout Top Module (project.sv) Cocotb Tests
=================================================

Tests for the Neander CPU integrated with TinyTapeout interface.
These tests verify the external RAM interface and I/O functionality.

The external RAM is limited to 32 bytes (5-bit address), so programs
must fit within this constraint.

Pin mapping:
    ui_in[7:0]   - I/O Input (directly to CPU io_in)
    uo_out[4:0]  - RAM Address (5-bit for 32 bytes)
    uo_out[5]    - RAM_WE (write enable)
    uo_out[6]    - RAM_OE (output enable / read)
    uo_out[7]    - IO_WRITE (I/O write strobe)
    uio[7:0]     - RAM Data Bus (bidirectional)
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, FallingEdge


# Neander Opcodes
class Op:
    NOP = 0x00
    STA = 0x10
    LDA = 0x20
    ADD = 0x30
    OR  = 0x40
    AND = 0x50
    NOT = 0x60
    JMP = 0x80
    JN  = 0x90
    JZ  = 0xA0
    JNZ = 0xB0
    IN  = 0xC0
    OUT = 0xD0
    LDI = 0xE0
    HLT = 0xF0


class TinyTapeoutTestbench:
    """Helper class for TinyTapeout top module testing"""

    def __init__(self, dut):
        self.dut = dut
        self.clock = None

    async def setup(self, clock_period_ns=10):
        """Initialize clock and reset"""
        self.clock = Clock(self.dut.clk, clock_period_ns, units="ns")
        cocotb.start_soon(self.clock.start())

        # Initialize signals
        self.dut.rst_n.value = 0       # Active low reset (asserted)
        self.dut.mem_load_en.value = 0
        self.dut.mem_load_addr.value = 0
        self.dut.mem_load_data.value = 0
        self.dut.io_in.value = 0
        self.dut.mem_read_addr.value = 0

        await ClockCycles(self.dut.clk, 5)

    async def reset(self):
        """Assert and release reset (active low)"""
        self.dut.rst_n.value = 0  # Assert reset
        await ClockCycles(self.dut.clk, 5)
        self.dut.rst_n.value = 1  # Release reset
        await ClockCycles(self.dut.clk, 2)

    async def load_byte(self, addr, data):
        """Load a single byte into external RAM"""
        assert addr < 32, f"Address {addr} exceeds 32-byte RAM limit"
        self.dut.mem_load_en.value = 1
        self.dut.mem_load_addr.value = addr
        self.dut.mem_load_data.value = data
        await RisingEdge(self.dut.clk)
        self.dut.mem_load_en.value = 0
        await RisingEdge(self.dut.clk)

    async def load_program(self, program, start_addr=0):
        """Load a program (list of bytes) into RAM starting at start_addr"""
        assert start_addr + len(program) <= 32, \
            f"Program ({len(program)} bytes) exceeds 32-byte RAM at addr {start_addr}"
        for i, byte in enumerate(program):
            await self.load_byte(start_addr + i, byte)

    async def read_memory(self, addr):
        """Read a byte from RAM"""
        assert addr < 32, f"Address {addr} exceeds 32-byte RAM limit"
        self.dut.mem_read_addr.value = addr
        await RisingEdge(self.dut.clk)
        return int(self.dut.mem_read_data.value)

    async def wait_for_io_write(self, max_cycles=5000):
        """Wait for an I/O write operation and return the data"""
        cycles = 0
        while cycles < max_cycles:
            await RisingEdge(self.dut.clk)
            cycles += 1
            if self.dut.io_write.value == 1:
                return int(self.dut.io_out_data.value)
        raise TimeoutError(f"No I/O write within {max_cycles} cycles")

    async def run_cycles(self, n):
        """Run for n clock cycles"""
        await ClockCycles(self.dut.clk, n)


# ============================================================================
# Test Programs (must fit in 32 bytes)
# ============================================================================

def create_simple_add_program():
    """
    Simple addition: 5 + 3 = 8
    Total: 12 bytes (fits in 32 bytes)

    Memory map:
        0x00-0x0B: Program (12 bytes)
        0x1E: Variable A (address 30)
    """
    ADDR_A = 0x1E  # Address 30

    program = [
        Op.LDI, 0x05,       # 0x00: AC = 5
        Op.STA, ADDR_A,     # 0x02: mem[0x1E] = 5
        Op.LDI, 0x03,       # 0x04: AC = 3
        Op.ADD, ADDR_A,     # 0x06: AC = 3 + 5 = 8
        Op.OUT, 0x00,       # 0x08: Output AC
        Op.HLT, 0x00,       # 0x0A: Halt
    ]
    return program, 8  # Expected result


def create_countdown_program():
    """
    Count down from 3 to 0 and output 0.
    Tests loop with JZ instruction.
    Total: 20 bytes (fits in 32 bytes)

    Memory map:
        0x00-0x13: Program (20 bytes)
        0x1E: COUNT
        0x1F: NEG1 (-1)
    """
    ADDR_COUNT = 0x1E  # Address 30
    ADDR_NEG1 = 0x1F   # Address 31
    LOOP_ADDR = 0x08
    DONE_ADDR = 0x10

    program = [
        # Initialize
        Op.LDI, 0x03,           # 0x00: AC = 3
        Op.STA, ADDR_COUNT,     # 0x02: COUNT = 3
        Op.LDI, 0xFF,           # 0x04: AC = -1
        Op.STA, ADDR_NEG1,      # 0x06: NEG1 = 0xFF

        # LOOP (0x08):
        Op.LDA, ADDR_COUNT,     # 0x08: AC = COUNT
        Op.JZ, DONE_ADDR,       # 0x0A: if AC==0, jump to DONE
        Op.ADD, ADDR_NEG1,      # 0x0C: AC = AC - 1
        Op.STA, ADDR_COUNT,     # 0x0E: COUNT = AC

        # DONE (0x10):
        Op.JMP, LOOP_ADDR,      # 0x10: Jump back to LOOP (will exit via JZ)
        # Note: When JZ succeeds, we fall through here from the loop
    ]

    # Patch: DONE should output and halt
    # Recalculate addresses
    program = [
        # Initialize (8 bytes)
        Op.LDI, 0x03,           # 0x00: AC = 3
        Op.STA, ADDR_COUNT,     # 0x02: COUNT = 3
        Op.LDI, 0xFF,           # 0x04: AC = -1
        Op.STA, ADDR_NEG1,      # 0x06: NEG1 = 0xFF

        # LOOP (0x08):
        Op.LDA, ADDR_COUNT,     # 0x08: AC = COUNT
        Op.JZ, 0x12,            # 0x0A: if AC==0, jump to 0x12 (DONE)
        Op.ADD, ADDR_NEG1,      # 0x0C: AC = AC - 1
        Op.STA, ADDR_COUNT,     # 0x0E: COUNT = AC
        Op.JMP, 0x08,           # 0x10: Jump back to LOOP

        # DONE (0x12):
        Op.OUT, 0x00,           # 0x12: Output AC (should be 0)
        Op.HLT, 0x00,           # 0x14: Halt
    ]
    return program, 0  # Expected result: 0


def create_io_echo_program():
    """
    Read from input port and output it.
    Total: 6 bytes
    """
    program = [
        Op.IN, 0x00,        # 0x00: Read from input port 0
        Op.OUT, 0x00,       # 0x02: Output what we read
        Op.HLT, 0x00,       # 0x04: Halt
    ]
    return program


def create_logical_ops_program():
    """
    Test logical operations: (0xAA AND 0x0F) OR 0xF0 = 0xFA
    Total: 18 bytes

    Memory map:
        0x1C: Value 0x0F
        0x1D: Value 0xF0
        0x1E: Temp storage
    """
    ADDR_MASK_LO = 0x1C
    ADDR_MASK_HI = 0x1D

    program = [
        # Setup masks
        Op.LDI, 0x0F,           # 0x00: AC = 0x0F
        Op.STA, ADDR_MASK_LO,   # 0x02: mem[0x1C] = 0x0F
        Op.LDI, 0xF0,           # 0x04: AC = 0xF0
        Op.STA, ADDR_MASK_HI,   # 0x06: mem[0x1D] = 0xF0

        # Compute (0xAA AND 0x0F) OR 0xF0
        Op.LDI, 0xAA,           # 0x08: AC = 0xAA
        Op.AND, ADDR_MASK_LO,   # 0x0A: AC = 0xAA AND 0x0F = 0x0A
        Op.OR, ADDR_MASK_HI,    # 0x0C: AC = 0x0A OR 0xF0 = 0xFA

        Op.OUT, 0x00,           # 0x0E: Output AC
        Op.HLT, 0x00,           # 0x10: Halt
    ]
    return program, 0xFA  # Expected: 0xFA


def create_not_program():
    """
    Test NOT operation: NOT 0x55 = 0xAA
    Total: 6 bytes
    """
    program = [
        Op.LDI, 0x55,       # 0x00: AC = 0x55
        Op.NOT, 0x00,       # 0x02: AC = NOT 0x55 = 0xAA
        Op.OUT, 0x00,       # 0x04: Output AC
        Op.HLT, 0x00,       # 0x06: Halt
    ]
    return program, 0xAA  # Expected: 0xAA


def create_jn_test_program():
    """
    Test JN (jump if negative) instruction.
    Load a negative value and verify JN jumps.
    Total: 14 bytes
    """
    program = [
        Op.LDI, 0x80,       # 0x00: AC = 0x80 (negative, MSB=1)
        Op.JN, 0x08,        # 0x02: JN to 0x08 (should jump)
        Op.LDI, 0xFF,       # 0x04: Skipped
        Op.OUT, 0x00,       # 0x06: Skipped

        # Jump target (0x08):
        Op.LDI, 0x42,       # 0x08: AC = 0x42 (success marker)
        Op.OUT, 0x00,       # 0x0A: Output 0x42
        Op.HLT, 0x00,       # 0x0C: Halt
    ]
    return program, 0x42  # Expected: 0x42


# ============================================================================
# Tests
# ============================================================================

@cocotb.test()
async def test_tt_simple_add(dut):
    """Test simple addition on TinyTapeout top module: 5 + 3 = 8"""
    tb = TinyTapeoutTestbench(dut)
    await tb.setup()

    program, expected = create_simple_add_program()

    dut._log.info(f"Loading program ({len(program)} bytes)")
    dut._log.info(f"Testing: 5 + 3 = {expected}")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=3000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_tt_countdown(dut):
    """Test countdown loop on TinyTapeout: 3 -> 2 -> 1 -> 0"""
    tb = TinyTapeoutTestbench(dut)
    await tb.setup()

    program, expected = create_countdown_program()

    dut._log.info(f"Loading program ({len(program)} bytes)")
    dut._log.info(f"Testing countdown to {expected}")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=5000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_tt_io_echo(dut):
    """Test I/O: read input and echo to output"""
    tb = TinyTapeoutTestbench(dut)
    await tb.setup()

    program = create_io_echo_program()
    test_value = 0x5A

    dut._log.info(f"Loading program ({len(program)} bytes)")
    dut._log.info(f"Testing I/O echo with value 0x{test_value:02X}")

    await tb.load_program(program)

    # Set input value before running
    tb.dut.io_in.value = test_value

    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{test_value:02X})")
    assert result == test_value, f"Expected 0x{test_value:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_tt_logical_ops(dut):
    """Test logical operations: (0xAA AND 0x0F) OR 0xF0 = 0xFA"""
    tb = TinyTapeoutTestbench(dut)
    await tb.setup()

    program, expected = create_logical_ops_program()

    dut._log.info(f"Loading program ({len(program)} bytes)")
    dut._log.info(f"Testing: (0xAA AND 0x0F) OR 0xF0 = 0x{expected:02X}")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=3000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_tt_not_operation(dut):
    """Test NOT operation: NOT 0x55 = 0xAA"""
    tb = TinyTapeoutTestbench(dut)
    await tb.setup()

    program, expected = create_not_program()

    dut._log.info(f"Loading program ({len(program)} bytes)")
    dut._log.info(f"Testing: NOT 0x55 = 0x{expected:02X}")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_tt_jn_instruction(dut):
    """Test JN (jump if negative) instruction"""
    tb = TinyTapeoutTestbench(dut)
    await tb.setup()

    program, expected = create_jn_test_program()

    dut._log.info(f"Loading program ({len(program)} bytes)")
    dut._log.info("Testing JN instruction")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"JN test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_tt_memory_write_read(dut):
    """Test memory write (STA) and read (LDA) operations"""
    tb = TinyTapeoutTestbench(dut)
    await tb.setup()

    ADDR_VAR = 0x1E  # Address 30

    program = [
        Op.LDI, 0x37,       # 0x00: AC = 0x37
        Op.STA, ADDR_VAR,   # 0x02: Store at 0x1E
        Op.LDI, 0x00,       # 0x04: AC = 0 (clear)
        Op.LDA, ADDR_VAR,   # 0x06: Load from 0x1E
        Op.OUT, 0x00,       # 0x08: Output AC
        Op.HLT, 0x00,       # 0x0A: Halt
    ]
    expected = 0x37

    dut._log.info(f"Loading program ({len(program)} bytes)")
    dut._log.info("Testing STA/LDA memory operations")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=3000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"Expected 0x{expected:02X}, got 0x{result:02X}"

    # Verify memory content
    mem_value = await tb.read_memory(ADDR_VAR)
    dut._log.info(f"Memory at 0x{ADDR_VAR:02X}: 0x{mem_value:02X}")
    assert mem_value == expected, f"Memory mismatch at 0x{ADDR_VAR:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_tt_ram_signals(dut):
    """Test that RAM control signals are correctly output"""
    tb = TinyTapeoutTestbench(dut)
    await tb.setup()

    # Simple program that does memory operations
    ADDR_VAR = 0x1E

    program = [
        Op.LDI, 0xAB,       # 0x00: AC = 0xAB
        Op.STA, ADDR_VAR,   # 0x02: Store at 0x1E (should assert ram_we)
        Op.LDA, ADDR_VAR,   # 0x04: Load from 0x1E (should assert ram_oe)
        Op.OUT, 0x00,       # 0x06: Output
        Op.HLT, 0x00,       # 0x08: Halt
    ]

    dut._log.info("Testing RAM control signals")

    await tb.load_program(program)
    await tb.reset()

    # Run and monitor signals
    saw_write = False
    saw_read = False

    for _ in range(500):
        await RisingEdge(dut.clk)
        if dut.ram_we.value == 1:
            saw_write = True
            addr = int(dut.ram_addr.value)
            dut._log.info(f"RAM WRITE detected at address 0x{addr:02X}")
        if dut.ram_oe.value == 1:
            saw_read = True
        if dut.io_write.value == 1:
            break

    assert saw_write, "Never saw RAM write signal"
    assert saw_read, "Never saw RAM read signal"

    dut._log.info("Test PASSED! RAM signals verified.")


@cocotb.test()
async def test_tt_reset_behavior(dut):
    """Test that reset properly initializes the CPU"""
    tb = TinyTapeoutTestbench(dut)
    await tb.setup()

    program = [
        Op.LDI, 0x77,       # AC = 0x77
        Op.OUT, 0x00,
        Op.HLT, 0x00,
    ]

    await tb.load_program(program)

    # First run
    await tb.reset()
    result1 = await tb.wait_for_io_write(max_cycles=2000)

    # Reset and run again
    await tb.reset()
    result2 = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"First run: 0x{result1:02X}, Second run: 0x{result2:02X}")

    assert result1 == 0x77, f"First run failed. Expected 0x77, got 0x{result1:02X}"
    assert result2 == 0x77, f"Second run failed. Expected 0x77, got 0x{result2:02X}"

    dut._log.info("Test PASSED!")
