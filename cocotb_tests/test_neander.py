"""
Neander CPU Cocotb Tests
========================

Tests for the Neander 8-bit educational processor.
Includes a test program that calculates f(a,b) = a*5 + b using a loop,
and outputs the result to the I/O port.

Neander Instruction Set:
    NOP (0x0) - No operation
    STA (0x1) - Store AC to memory
    LDA (0x2) - Load AC from memory
    ADD (0x3) - Add memory to AC
    OR  (0x4) - OR memory with AC
    AND (0x5) - AND memory with AC
    NOT (0x6) - NOT AC
    JMP (0x8) - Unconditional jump
    JN  (0x9) - Jump if negative
    JZ  (0xA) - Jump if zero
    JNZ (0xB) - Jump if not zero
    IN  (0xC) - Input from port
    OUT (0xD) - Output to port
    LDI (0xE) - Load immediate
    HLT (0xF) - Halt
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer


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


class NeanderTestbench:
    """Helper class for Neander CPU testing"""

    def __init__(self, dut):
        self.dut = dut
        self.clock = None

    async def setup(self, clock_period_ns=10):
        """Initialize clock and reset"""
        self.clock = Clock(self.dut.clk, clock_period_ns, units="ns")
        cocotb.start_soon(self.clock.start())

        # Initialize signals
        self.dut.reset.value = 1
        self.dut.mem_load_en.value = 0
        self.dut.mem_load_addr.value = 0
        self.dut.mem_load_data.value = 0
        self.dut.io_in.value = 0
        self.dut.io_status.value = 0
        self.dut.mem_read_addr.value = 0

        await ClockCycles(self.dut.clk, 5)

    async def reset(self):
        """Assert and release reset"""
        self.dut.reset.value = 1
        await ClockCycles(self.dut.clk, 5)
        self.dut.reset.value = 0
        await ClockCycles(self.dut.clk, 2)

    async def load_byte(self, addr, data):
        """Load a single byte into memory"""
        self.dut.mem_load_en.value = 1
        self.dut.mem_load_addr.value = addr
        self.dut.mem_load_data.value = data
        await RisingEdge(self.dut.clk)
        self.dut.mem_load_en.value = 0
        await RisingEdge(self.dut.clk)

    async def load_program(self, program, start_addr=0):
        """Load a program (list of bytes) into memory starting at start_addr"""
        for i, byte in enumerate(program):
            await self.load_byte(start_addr + i, byte)

    async def read_memory(self, addr):
        """Read a byte from memory"""
        self.dut.mem_read_addr.value = addr
        await RisingEdge(self.dut.clk)
        return int(self.dut.mem_read_data.value)

    async def run_until_halt(self, max_cycles=10000):
        """Run CPU until HLT instruction (PC stops changing) or timeout"""
        last_pc = -1
        halt_counter = 0
        cycles = 0

        while cycles < max_cycles:
            await RisingEdge(self.dut.clk)
            cycles += 1

            current_pc = int(self.dut.dbg_pc.value)
            current_ri = int(self.dut.dbg_ri.value)

            # Check if we're in HLT state (PC doesn't change and RI is HLT)
            if current_pc == last_pc and (current_ri & 0xF0) == Op.HLT:
                halt_counter += 1
                if halt_counter > 10:  # Confirm we're really halted
                    return cycles
            else:
                halt_counter = 0

            last_pc = current_pc

        raise TimeoutError(f"CPU did not halt within {max_cycles} cycles")

    async def wait_for_io_write(self, max_cycles=10000):
        """Wait for an I/O write operation"""
        cycles = 0
        while cycles < max_cycles:
            await RisingEdge(self.dut.clk)
            cycles += 1
            if self.dut.io_write.value == 1:
                return int(self.dut.io_out.value)
        raise TimeoutError(f"No I/O write within {max_cycles} cycles")


def create_multiply_by_5_program(a_value, b_value):
    """
    Create a program that calculates f(a,b) = a*5 + b
    Uses a loop to multiply by repeated addition.

    Memory map:
        0x80: A (input value a)
        0x81: B (input value b)
        0x82: RESULT (accumulator for result)
        0x83: COUNT (loop counter)
        0x84: NEG1 (constant -1 = 0xFF)

    Returns: (program_bytes, expected_result)
    """
    # Data addresses
    ADDR_A = 0x80
    ADDR_B = 0x81
    ADDR_RESULT = 0x82
    ADDR_COUNT = 0x83
    ADDR_NEG1 = 0x84

    # Program addresses
    LOOP_ADDR = 0x14
    DONE_ADDR = 0x26

    program = [
        # Initialize A = a_value
        Op.LDI, a_value,        # 0x00: LDI a
        Op.STA, ADDR_A,         # 0x02: STA A

        # Initialize B = b_value
        Op.LDI, b_value,        # 0x04: LDI b
        Op.STA, ADDR_B,         # 0x06: STA B

        # Initialize COUNT = 5 (multiply by 5)
        Op.LDI, 5,              # 0x08: LDI 5
        Op.STA, ADDR_COUNT,     # 0x0A: STA COUNT

        # Initialize RESULT = 0
        Op.LDI, 0,              # 0x0C: LDI 0
        Op.STA, ADDR_RESULT,    # 0x0E: STA RESULT

        # Initialize NEG1 = 0xFF (-1)
        Op.LDI, 0xFF,           # 0x10: LDI 0xFF
        Op.STA, ADDR_NEG1,      # 0x12: STA NEG1

        # LOOP: (address 0x14)
        Op.LDA, ADDR_COUNT,     # 0x14: LDA COUNT
        Op.JZ, DONE_ADDR,       # 0x16: JZ DONE (if count==0, exit loop)

        Op.LDA, ADDR_RESULT,    # 0x18: LDA RESULT
        Op.ADD, ADDR_A,         # 0x1A: ADD A (result += a)
        Op.STA, ADDR_RESULT,    # 0x1C: STA RESULT

        Op.LDA, ADDR_COUNT,     # 0x1E: LDA COUNT
        Op.ADD, ADDR_NEG1,      # 0x20: ADD NEG1 (count--)
        Op.STA, ADDR_COUNT,     # 0x22: STA COUNT

        Op.JMP, LOOP_ADDR,      # 0x24: JMP LOOP

        # DONE: (address 0x26)
        Op.LDA, ADDR_RESULT,    # 0x26: LDA RESULT
        Op.ADD, ADDR_B,         # 0x28: ADD B (result += b)
        Op.STA, ADDR_RESULT,    # 0x2A: STA RESULT

        Op.OUT, 0x00,           # 0x2C: OUT 0 (output result to port 0)
        Op.HLT, 0x00,           # 0x2E: HLT
    ]

    expected = (a_value * 5 + b_value) & 0xFF  # 8-bit result
    return program, expected


@cocotb.test()
async def test_multiply_by_5_basic(dut):
    """Test f(a,b) = a*5 + b with a=3, b=7 => result=22"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Create program: f(3,7) = 3*5 + 7 = 22
    a, b = 3, 7
    program, expected = create_multiply_by_5_program(a, b)

    dut._log.info(f"Testing f({a},{b}) = {a}*5 + {b} = {expected}")

    # Load program
    await tb.load_program(program)

    # Release reset and run
    await tb.reset()

    # Wait for output
    result = await tb.wait_for_io_write(max_cycles=5000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")

    assert result == expected, f"Expected {expected}, got {result}"

    # Also verify memory
    mem_result = await tb.read_memory(0x82)
    assert mem_result == expected, f"Memory at 0x82: expected {expected}, got {mem_result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_multiply_by_5_larger(dut):
    """Test f(a,b) = a*5 + b with a=10, b=5 => result=55"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # f(10,5) = 10*5 + 5 = 55
    a, b = 10, 5
    program, expected = create_multiply_by_5_program(a, b)

    dut._log.info(f"Testing f({a},{b}) = {a}*5 + {b} = {expected}")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=5000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_multiply_by_5_overflow(dut):
    """Test f(a,b) with overflow: a=50, b=10 => result=260 & 0xFF = 4"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # f(50,10) = 50*5 + 10 = 260, but 260 & 0xFF = 4 (overflow)
    a, b = 50, 10
    program, expected = create_multiply_by_5_program(a, b)

    dut._log.info(f"Testing f({a},{b}) with overflow = {expected}")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=5000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_multiply_by_5_zero(dut):
    """Test f(a,b) = a*5 + b with a=0, b=42 => result=42"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # f(0,42) = 0*5 + 42 = 42
    a, b = 0, 42
    program, expected = create_multiply_by_5_program(a, b)

    dut._log.info(f"Testing f({a},{b}) = {a}*5 + {b} = {expected}")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=5000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


def create_simple_add_program(a_value, b_value):
    """
    Simple program: add two values and output result.
    f(a,b) = a + b
    """
    ADDR_A = 0x80
    ADDR_B = 0x81

    program = [
        Op.LDI, a_value,    # LDI a
        Op.STA, ADDR_A,     # STA A
        Op.LDI, b_value,    # LDI b
        Op.STA, ADDR_B,     # STA B
        Op.LDA, ADDR_A,     # LDA A
        Op.ADD, ADDR_B,     # ADD B
        Op.OUT, 0x00,       # OUT 0
        Op.HLT, 0x00,       # HLT
    ]

    expected = (a_value + b_value) & 0xFF
    return program, expected


@cocotb.test()
async def test_simple_add(dut):
    """Test simple addition: 10 + 20 = 30"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    a, b = 10, 20
    program, expected = create_simple_add_program(a, b)

    dut._log.info(f"Testing simple add: {a} + {b} = {expected}")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


def create_loop_countdown_program(start_value):
    """
    Count down from start_value to 0 and output final value (should be 0).
    Tests loop with JZ instruction.
    """
    ADDR_COUNT = 0x80
    ADDR_NEG1 = 0x81

    LOOP_ADDR = 0x08
    DONE_ADDR = 0x16

    program = [
        # Initialize
        Op.LDI, start_value,    # 0x00: LDI start
        Op.STA, ADDR_COUNT,     # 0x02: STA COUNT
        Op.LDI, 0xFF,           # 0x04: LDI -1
        Op.STA, ADDR_NEG1,      # 0x06: STA NEG1

        # LOOP:
        Op.LDA, ADDR_COUNT,     # 0x08: LDA COUNT
        Op.JZ, DONE_ADDR,       # 0x0A: JZ DONE
        Op.ADD, ADDR_NEG1,      # 0x0C: ADD NEG1 (decrement)
        Op.STA, ADDR_COUNT,     # 0x0E: STA COUNT
        Op.JMP, LOOP_ADDR,      # 0x10: JMP LOOP

        # Padding to reach DONE_ADDR
        Op.NOP, 0x00,           # 0x12: NOP
        Op.NOP, 0x00,           # 0x14: NOP

        # DONE:
        Op.LDA, ADDR_COUNT,     # 0x16: LDA COUNT (should be 0)
        Op.OUT, 0x00,           # 0x18: OUT 0
        Op.HLT, 0x00,           # 0x1A: HLT
    ]

    return program, 0  # Expected: 0 after countdown


@cocotb.test()
async def test_loop_countdown(dut):
    """Test loop countdown from 10 to 0"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    start = 10
    program, expected = create_loop_countdown_program(start)

    dut._log.info(f"Testing countdown from {start} to {expected}")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=5000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_logical_operations(dut):
    """Test AND, OR, NOT operations"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_A = 0x80
    ADDR_B = 0x81

    # Test: (0xF0 AND 0x0F) = 0x00, then OR with 0x0F = 0x0F, then NOT = 0xF0
    program = [
        Op.LDI, 0xF0,       # AC = 0xF0
        Op.STA, ADDR_A,     # A = 0xF0
        Op.LDI, 0x0F,       # AC = 0x0F
        Op.STA, ADDR_B,     # B = 0x0F
        Op.LDA, ADDR_A,     # AC = 0xF0
        Op.AND, ADDR_B,     # AC = 0xF0 AND 0x0F = 0x00
        Op.OR, ADDR_B,      # AC = 0x00 OR 0x0F = 0x0F
        Op.NOT, 0x00,       # AC = NOT 0x0F = 0xF0
        Op.OUT, 0x00,       # Output 0xF0
        Op.HLT, 0x00,
    ]
    expected = 0xF0

    dut._log.info(f"Testing logical operations, expected result: 0x{expected:02X}")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_conditional_jump_jn(dut):
    """Test JN (jump if negative) instruction"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_RESULT = 0x80
    JN_TARGET = 0x0E

    # Load negative value and test JN
    program = [
        Op.LDI, 0x80,           # 0x00: AC = 0x80 (negative, MSB=1)
        Op.STA, ADDR_RESULT,    # 0x02: Store it
        Op.LDA, ADDR_RESULT,    # 0x04: Load back (sets N flag)
        Op.JN, JN_TARGET,       # 0x06: JN to 0x0E (should jump)
        Op.LDI, 0xFF,           # 0x08: This should be skipped
        Op.OUT, 0x00,           # 0x0A: This should be skipped
        Op.HLT, 0x00,           # 0x0C: This should be skipped

        # JN_TARGET (0x0E):
        Op.LDI, 0x42,           # 0x0E: AC = 0x42 (success marker)
        Op.OUT, 0x00,           # 0x10: Output success
        Op.HLT, 0x00,           # 0x12: Halt
    ]
    expected = 0x42

    dut._log.info("Testing JN instruction")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"JN did not jump correctly. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


# ============================================================================
# ADDITIONAL TESTS FOR COMPLETE COVERAGE
# ============================================================================

@cocotb.test()
async def test_nop_instruction(dut):
    """Test NOP instruction - should do nothing and continue"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Program with NOPs between operations
    program = [
        Op.LDI, 0x10,       # AC = 0x10
        Op.NOP, 0x00,       # NOP (should do nothing)
        Op.NOP, 0x00,       # NOP (should do nothing)
        Op.NOP, 0x00,       # NOP (should do nothing)
        Op.OUT, 0x00,       # Output AC (should still be 0x10)
        Op.HLT, 0x00,
    ]
    expected = 0x10

    dut._log.info("Testing NOP instruction")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"NOP affected AC. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_jnz_instruction_jump(dut):
    """Test JNZ (jump if not zero) - should jump when AC != 0"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    JNZ_TARGET = 0x0C

    program = [
        Op.LDI, 0x05,           # 0x00: AC = 5 (not zero)
        Op.JNZ, JNZ_TARGET,     # 0x02: JNZ to 0x0C (should jump)
        Op.LDI, 0xFF,           # 0x04: This should be skipped
        Op.OUT, 0x00,           # 0x06: This should be skipped
        Op.HLT, 0x00,           # 0x08: This should be skipped
        Op.NOP, 0x00,           # 0x0A: Padding

        # JNZ_TARGET (0x0C):
        Op.LDI, 0xAA,           # 0x0C: AC = 0xAA (success marker)
        Op.OUT, 0x00,           # 0x0E: Output success
        Op.HLT, 0x00,           # 0x10: Halt
    ]
    expected = 0xAA

    dut._log.info("Testing JNZ instruction (should jump)")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"JNZ did not jump. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_jnz_instruction_no_jump(dut):
    """Test JNZ (jump if not zero) - should NOT jump when AC == 0"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    JNZ_TARGET = 0x0C

    program = [
        Op.LDI, 0x00,           # 0x00: AC = 0 (zero!)
        Op.JNZ, JNZ_TARGET,     # 0x02: JNZ to 0x0C (should NOT jump)
        Op.LDI, 0xBB,           # 0x04: AC = 0xBB (this should execute)
        Op.OUT, 0x00,           # 0x06: Output 0xBB
        Op.HLT, 0x00,           # 0x08: Halt
        Op.NOP, 0x00,           # 0x0A: Padding

        # JNZ_TARGET (0x0C):
        Op.LDI, 0xAA,           # 0x0C: This should be skipped
        Op.OUT, 0x00,           # 0x0E: This should be skipped
        Op.HLT, 0x00,           # 0x10: This should be skipped
    ]
    expected = 0xBB

    dut._log.info("Testing JNZ instruction (should NOT jump)")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"JNZ jumped when it shouldn't. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_jz_instruction_jump(dut):
    """Test JZ (jump if zero) - should jump when AC == 0"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    JZ_TARGET = 0x0C

    program = [
        Op.LDI, 0x00,           # 0x00: AC = 0 (zero!)
        Op.JZ, JZ_TARGET,       # 0x02: JZ to 0x0C (should jump)
        Op.LDI, 0xFF,           # 0x04: This should be skipped
        Op.OUT, 0x00,           # 0x06: This should be skipped
        Op.HLT, 0x00,           # 0x08: This should be skipped
        Op.NOP, 0x00,           # 0x0A: Padding

        # JZ_TARGET (0x0C):
        Op.LDI, 0xCC,           # 0x0C: AC = 0xCC (success marker)
        Op.OUT, 0x00,           # 0x0E: Output success
        Op.HLT, 0x00,           # 0x10: Halt
    ]
    expected = 0xCC

    dut._log.info("Testing JZ instruction (should jump)")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"JZ did not jump. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_jz_instruction_no_jump(dut):
    """Test JZ (jump if zero) - should NOT jump when AC != 0"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    JZ_TARGET = 0x0C

    program = [
        Op.LDI, 0x01,           # 0x00: AC = 1 (not zero)
        Op.JZ, JZ_TARGET,       # 0x02: JZ to 0x0C (should NOT jump)
        Op.LDI, 0xDD,           # 0x04: AC = 0xDD (this should execute)
        Op.OUT, 0x00,           # 0x06: Output 0xDD
        Op.HLT, 0x00,           # 0x08: Halt
        Op.NOP, 0x00,           # 0x0A: Padding

        # JZ_TARGET (0x0C):
        Op.LDI, 0xCC,           # 0x0C: This should be skipped
        Op.OUT, 0x00,           # 0x0E: This should be skipped
        Op.HLT, 0x00,           # 0x10: This should be skipped
    ]
    expected = 0xDD

    dut._log.info("Testing JZ instruction (should NOT jump)")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"JZ jumped when it shouldn't. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_jn_instruction_no_jump(dut):
    """Test JN (jump if negative) - should NOT jump when AC is positive"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    JN_TARGET = 0x0C

    program = [
        Op.LDI, 0x7F,           # 0x00: AC = 0x7F (positive, MSB=0)
        Op.JN, JN_TARGET,       # 0x02: JN to 0x0C (should NOT jump)
        Op.LDI, 0xEE,           # 0x04: AC = 0xEE (this should execute)
        Op.OUT, 0x00,           # 0x06: Output 0xEE
        Op.HLT, 0x00,           # 0x08: Halt
        Op.NOP, 0x00,           # 0x0A: Padding

        # JN_TARGET (0x0C):
        Op.LDI, 0x42,           # 0x0C: This should be skipped
        Op.OUT, 0x00,           # 0x0E: This should be skipped
        Op.HLT, 0x00,           # 0x10: This should be skipped
    ]
    expected = 0xEE

    dut._log.info("Testing JN instruction (should NOT jump)")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"JN jumped when it shouldn't. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_in_instruction(dut):
    """Test IN instruction - read from input port"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Set input value before running
    input_value = 0x5A
    tb.dut.io_in.value = input_value

    program = [
        Op.IN, 0x00,        # Read from port 0 (data port)
        Op.OUT, 0x00,       # Output what we read
        Op.HLT, 0x00,
    ]
    expected = input_value

    dut._log.info(f"Testing IN instruction with input value 0x{input_value:02X}")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"IN instruction failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_in_status_port(dut):
    """Test IN instruction - read from status port"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Set status value (bit 0 = data available)
    status_value = 0x01
    tb.dut.io_status.value = status_value

    program = [
        Op.IN, 0x01,        # Read from port 1 (status port)
        Op.OUT, 0x00,       # Output what we read
        Op.HLT, 0x00,
    ]
    expected = status_value

    dut._log.info(f"Testing IN instruction from status port")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"IN status failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_sta_lda_memory(dut):
    """Test STA and LDA - store and load from memory"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAR = 0x80

    program = [
        Op.LDI, 0x37,       # AC = 0x37
        Op.STA, ADDR_VAR,   # Store AC at 0x80
        Op.LDI, 0x00,       # AC = 0 (clear AC)
        Op.LDA, ADDR_VAR,   # Load from 0x80 (should be 0x37)
        Op.OUT, 0x00,       # Output AC
        Op.HLT, 0x00,
    ]
    expected = 0x37

    dut._log.info("Testing STA/LDA memory operations")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"STA/LDA failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    # Also verify memory directly
    mem_value = await tb.read_memory(ADDR_VAR)
    assert mem_value == expected, f"Memory verification failed. Expected 0x{expected:02X}, got 0x{mem_value:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_flags_zero(dut):
    """Test Z flag is set correctly when result is zero"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAR = 0x80
    JZ_TARGET = 0x10  # Corrected address

    # Test that Z flag is set after operation resulting in zero
    # Address map:
    # 0x00-0x01: LDI 0xFF
    # 0x02-0x03: STA 0x80
    # 0x04-0x05: LDI 0x01
    # 0x06-0x07: ADD 0x80
    # 0x08-0x09: JZ 0x10
    # 0x0A-0x0B: LDI 0x11 (skipped if Z=1)
    # 0x0C-0x0D: OUT 0x00 (skipped)
    # 0x0E-0x0F: HLT (skipped)
    # 0x10-0x11: LDI 0x22 (jump target)
    # 0x12-0x13: OUT 0x00
    # 0x14-0x15: HLT
    program = [
        Op.LDI, 0xFF,           # 0x00: AC = 0xFF
        Op.STA, ADDR_VAR,       # 0x02: Store 0xFF
        Op.LDI, 0x01,           # 0x04: AC = 0x01
        Op.ADD, ADDR_VAR,       # 0x06: AC = 0x01 + 0xFF = 0x00 (overflow, Z=1)
        Op.JZ, JZ_TARGET,       # 0x08: Should jump because Z=1
        Op.LDI, 0x11,           # 0x0A: Skipped
        Op.OUT, 0x00,           # 0x0C: Skipped
        Op.HLT, 0x00,           # 0x0E: Skipped

        # JZ_TARGET (0x10):
        Op.LDI, 0x22,           # 0x10: AC = 0x22 (success)
        Op.OUT, 0x00,           # 0x12
        Op.HLT, 0x00,           # 0x14
    ]
    expected = 0x22

    dut._log.info("Testing Z flag after overflow to zero")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"Z flag test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_flags_negative(dut):
    """Test N flag is set correctly when result is negative"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAR = 0x80
    JN_TARGET = 0x10  # Corrected address

    # Test that N flag is set after operation resulting in negative
    # Address map:
    # 0x00-0x01: LDI 0x7F
    # 0x02-0x03: STA 0x80
    # 0x04-0x05: LDI 0x02
    # 0x06-0x07: ADD 0x80
    # 0x08-0x09: JN 0x10
    # 0x0A-0x0B: LDI 0x11 (skipped if N=1)
    # 0x0C-0x0D: OUT 0x00 (skipped)
    # 0x0E-0x0F: HLT (skipped)
    # 0x10-0x11: LDI 0x33 (jump target)
    # 0x12-0x13: OUT 0x00
    # 0x14-0x15: HLT
    program = [
        Op.LDI, 0x7F,           # 0x00: AC = 0x7F (127, positive)
        Op.STA, ADDR_VAR,       # 0x02: Store
        Op.LDI, 0x02,           # 0x04: AC = 0x02
        Op.ADD, ADDR_VAR,       # 0x06: AC = 0x02 + 0x7F = 0x81 (negative, N=1)
        Op.JN, JN_TARGET,       # 0x08: Should jump because N=1
        Op.LDI, 0x11,           # 0x0A: Skipped
        Op.OUT, 0x00,           # 0x0C: Skipped
        Op.HLT, 0x00,           # 0x0E: Skipped

        # JN_TARGET (0x10):
        Op.LDI, 0x33,           # 0x10: AC = 0x33 (success)
        Op.OUT, 0x00,           # 0x12
        Op.HLT, 0x00,           # 0x14
    ]
    expected = 0x33

    dut._log.info("Testing N flag after overflow to negative")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"N flag test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_edge_case_max_value(dut):
    """Test edge case with maximum value 0xFF"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_ONE = 0x80

    program = [
        Op.LDI, 0x01,       # AC = 1
        Op.STA, ADDR_ONE,   # Store 1
        Op.LDI, 0xFF,       # AC = 0xFF (255, -1 in two's complement)
        Op.ADD, ADDR_ONE,   # AC = 0xFF + 0x01 = 0x00 (overflow)
        Op.OUT, 0x00,       # Output 0
        Op.HLT, 0x00,
    ]
    expected = 0x00

    dut._log.info("Testing edge case: 0xFF + 0x01 = 0x00 (overflow)")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"Overflow test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_edge_case_boundary_0x80(dut):
    """Test edge case at boundary 0x80 (most negative value)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # 0x80 = -128 in two's complement, also sets N flag
    program = [
        Op.LDI, 0x80,       # AC = 0x80 (-128)
        Op.NOT, 0x00,       # AC = NOT 0x80 = 0x7F (127)
        Op.OUT, 0x00,
        Op.HLT, 0x00,
    ]
    expected = 0x7F

    dut._log.info("Testing edge case: NOT 0x80 = 0x7F")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"Boundary test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_jmp_unconditional(dut):
    """Test JMP unconditional jump"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    JMP_TARGET = 0x0A

    program = [
        Op.LDI, 0x11,           # 0x00: AC = 0x11
        Op.JMP, JMP_TARGET,     # 0x02: JMP to 0x0A
        Op.LDI, 0xFF,           # 0x04: Skipped
        Op.OUT, 0x00,           # 0x06: Skipped
        Op.HLT, 0x00,           # 0x08: Skipped

        # JMP_TARGET (0x0A):
        Op.LDI, 0x44,           # 0x0A: AC = 0x44
        Op.OUT, 0x00,           # 0x0C: Output
        Op.HLT, 0x00,           # 0x0E: Halt
    ]
    expected = 0x44

    dut._log.info("Testing JMP unconditional jump")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"JMP test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_reset_behavior(dut):
    """Test that reset properly initializes the CPU"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    program = [
        Op.LDI, 0x55,       # AC = 0x55
        Op.OUT, 0x00,
        Op.HLT, 0x00,
    ]

    await tb.load_program(program)

    # Run first time
    await tb.reset()
    result1 = await tb.wait_for_io_write(max_cycles=2000)

    # Reset and run again - should produce same result
    await tb.reset()
    result2 = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"First run: 0x{result1:02X}, Second run: 0x{result2:02X}")

    assert result1 == 0x55, f"First run failed. Expected 0x55, got 0x{result1:02X}"
    assert result2 == 0x55, f"Second run after reset failed. Expected 0x55, got 0x{result2:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_all_alu_operations(dut):
    """Comprehensive test of all ALU operations"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_A = 0x80
    ADDR_B = 0x81
    ADDR_R = 0x82

    # Test ADD, AND, OR, NOT in sequence
    program = [
        # Setup
        Op.LDI, 0xAA,       # AC = 0xAA (10101010)
        Op.STA, ADDR_A,
        Op.LDI, 0x55,       # AC = 0x55 (01010101)
        Op.STA, ADDR_B,

        # ADD: 0xAA + 0x55 = 0xFF
        Op.LDA, ADDR_A,
        Op.ADD, ADDR_B,
        Op.STA, ADDR_R,     # R = 0xFF

        # AND: 0xAA AND 0x55 = 0x00
        Op.LDA, ADDR_A,
        Op.AND, ADDR_B,     # AC = 0x00

        # OR with R: 0x00 OR 0xFF = 0xFF
        Op.OR, ADDR_R,      # AC = 0xFF

        # NOT: NOT 0xFF = 0x00
        Op.NOT, 0x00,       # AC = 0x00

        Op.OUT, 0x00,
        Op.HLT, 0x00,
    ]
    expected = 0x00

    dut._log.info("Testing all ALU operations in sequence")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"ALU test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")
