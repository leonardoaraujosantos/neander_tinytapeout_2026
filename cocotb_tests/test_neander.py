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
    NOP  = 0x00
    # LCC Compiler Extension (opcode 0x0 family)
    NEG  = 0x01  # NEG: AC = -AC (two's complement)
    CMP  = 0x02  # CMP addr: compare AC with MEM[addr], set flags only
    # Y Register Extension (opcode 0x0 family)
    TAY  = 0x03  # TAY: Y = AC
    TYA  = 0x04  # TYA: AC = Y
    INY  = 0x05  # INY: Y = Y + 1
    LDYI = 0x06  # LDYI imm: Y = imm
    LDY  = 0x07  # LDY addr: Y = MEM[addr]
    STY  = 0x08  # STY addr: MEM[addr] = Y
    MUL  = 0x09  # MUL: AC * X -> Y:AC (16-bit result)
    # Frame Pointer Extension (opcode 0x0 family)
    TSF  = 0x0A  # TSF: FP = SP (transfer SP to FP)
    TFS  = 0x0B  # TFS: SP = FP (transfer FP to SP)
    PUSH_FP = 0x0C  # PUSH_FP: MEM[--SP] = FP
    POP_FP  = 0x0D  # POP_FP: FP = MEM[SP++]
    # Division Extension (opcode 0x0 family)
    DIV  = 0x0E  # DIV: AC / X -> AC (quotient), Y (remainder)
    MOD  = 0x0F  # MOD: AC % X -> AC (remainder), Y (quotient)
    STA  = 0x10
    LDA  = 0x20
    ADD  = 0x30
    OR   = 0x40
    AND  = 0x50
    NOT  = 0x60
    PUSH = 0x70  # Stack PUSH (sub-opcode 0)
    POP  = 0x71  # Stack POP  (sub-opcode 1)
    CALL = 0x72  # CALL subroutine (sub-opcode 2)
    RET  = 0x73  # RET from subroutine (sub-opcode 3)
    # LCC Extension instructions
    SUB  = 0x74  # SUB addr: AC = AC - MEM[addr]
    INC  = 0x75  # INC: AC = AC + 1
    DEC  = 0x76  # DEC: AC = AC - 1
    XOR  = 0x77  # XOR addr: AC = AC ^ MEM[addr]
    SHL  = 0x78  # SHL: AC = AC << 1
    SHR  = 0x79  # SHR: AC = AC >> 1
    # X Register Extension instructions
    LDX  = 0x7A  # LDX addr: X = MEM[addr]
    STX  = 0x7B  # STX addr: MEM[addr] = X
    LDXI = 0x7C  # LDXI imm: X = imm
    TAX  = 0x7D  # TAX: X = AC
    TXA  = 0x7E  # TXA: AC = X
    INX  = 0x7F  # INX: X = X + 1
    # Indexed addressing modes (X)
    LDA_X = 0x21  # LDA addr,X: AC = MEM[addr + X]
    STA_X = 0x11  # STA addr,X: MEM[addr + X] = AC
    # Indexed addressing modes (Y)
    LDA_Y = 0x22  # LDA addr,Y: AC = MEM[addr + Y]
    STA_Y = 0x12  # STA addr,Y: MEM[addr + Y] = AC
    # Indexed addressing modes (FP)
    LDA_FP = 0x24  # LDA addr,FP: AC = MEM[addr + FP]
    STA_FP = 0x14  # STA addr,FP: MEM[addr + FP] = AC
    JMP  = 0x80
    JC   = 0x81  # JC addr: jump if carry flag set
    JNC  = 0x82  # JNC addr: jump if carry flag clear
    # Signed comparison jumps (after CMP)
    JLE  = 0x83  # JLE addr: jump if less or equal (N=1 OR Z=1)
    JGT  = 0x84  # JGT addr: jump if greater than (N=0 AND Z=0)
    JGE  = 0x85  # JGE addr: jump if greater or equal (N=0)
    # Unsigned comparison jumps (after CMP)
    JBE  = 0x86  # JBE addr: jump if below or equal (C=1 OR Z=1)
    JA   = 0x87  # JA addr: jump if above (C=0 AND Z=0)
    JN   = 0x90
    JZ   = 0xA0
    JNZ  = 0xB0
    IN   = 0xC0
    OUT  = 0xD0
    LDI  = 0xE0
    HLT  = 0xF0


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


# ============================================================================
# STACK INSTRUCTION TESTS (PUSH/POP)
# ============================================================================

@cocotb.test()
async def test_push_pop_single(dut):
    """Test single PUSH and POP operation"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Program: Load value, push it, load different value, pop original back
    program = [
        Op.LDI, 0x42,       # AC = 0x42
        Op.PUSH, 0x00,      # Push 0x42 onto stack
        Op.LDI, 0x00,       # AC = 0x00 (clear AC)
        Op.POP, 0x00,       # Pop back to AC (should be 0x42)
        Op.OUT, 0x00,       # Output AC
        Op.HLT, 0x00,
    ]
    expected = 0x42

    dut._log.info("Testing single PUSH/POP: push 0x42, clear AC, pop back")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"PUSH/POP test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_push_pop_multiple(dut):
    """Test multiple PUSH and POP operations (LIFO order)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_R1 = 0x80
    ADDR_R2 = 0x81

    # Program: Push 3 values, pop them back in reverse order
    # Push 0x11, 0x22, 0x33, then pop and verify LIFO order
    program = [
        Op.LDI, 0x11,       # AC = 0x11
        Op.PUSH, 0x00,      # Push 0x11
        Op.LDI, 0x22,       # AC = 0x22
        Op.PUSH, 0x00,      # Push 0x22
        Op.LDI, 0x33,       # AC = 0x33
        Op.PUSH, 0x00,      # Push 0x33

        # Now pop - should get 0x33, 0x22, 0x11
        Op.POP, 0x00,       # Pop -> AC = 0x33
        Op.STA, ADDR_R1,    # Store first pop result
        Op.POP, 0x00,       # Pop -> AC = 0x22
        Op.STA, ADDR_R2,    # Store second pop result
        Op.POP, 0x00,       # Pop -> AC = 0x11

        # Output the last popped value (0x11)
        Op.OUT, 0x00,
        Op.HLT, 0x00,
    ]
    expected = 0x11  # Last value pushed, last to be popped

    dut._log.info("Testing multiple PUSH/POP with LIFO order")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=3000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"Multiple PUSH/POP test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    # Verify intermediate values stored in memory
    r1 = await tb.read_memory(ADDR_R1)
    r2 = await tb.read_memory(ADDR_R2)
    dut._log.info(f"First pop (stored at 0x80): 0x{r1:02X} (expected: 0x33)")
    dut._log.info(f"Second pop (stored at 0x81): 0x{r2:02X} (expected: 0x22)")

    assert r1 == 0x33, f"First pop should be 0x33, got 0x{r1:02X}"
    assert r2 == 0x22, f"Second pop should be 0x22, got 0x{r2:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_push_pop_flags(dut):
    """Test that POP sets N and Z flags correctly"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    JN_TARGET = 0x14

    # Push a negative value (0x80), pop it, and check N flag via JN
    program = [
        Op.LDI, 0x80,           # 0x00: AC = 0x80 (negative)
        Op.PUSH, 0x00,          # 0x02: Push 0x80
        Op.LDI, 0x00,           # 0x04: AC = 0x00 (clear, to reset flags)
        Op.POP, 0x00,           # 0x06: Pop -> AC = 0x80, sets N=1
        Op.JN, JN_TARGET,       # 0x08: Should jump because N=1
        Op.LDI, 0xFF,           # 0x0A: Skipped if JN works
        Op.OUT, 0x00,           # 0x0C: Skipped
        Op.HLT, 0x00,           # 0x0E: Skipped
        Op.NOP, 0x00,           # 0x10: Padding
        Op.NOP, 0x00,           # 0x12: Padding

        # JN_TARGET (0x14):
        Op.LDI, 0xAA,           # 0x14: AC = 0xAA (success marker)
        Op.OUT, 0x00,           # 0x16: Output success
        Op.HLT, 0x00,           # 0x18: Halt
    ]
    expected = 0xAA

    dut._log.info("Testing POP sets N flag correctly")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"POP N flag test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_push_pop_zero_flag(dut):
    """Test that POP sets Z flag correctly when popping zero"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    JZ_TARGET = 0x14

    # Push zero, change AC, pop zero back, check Z flag via JZ
    program = [
        Op.LDI, 0x00,           # 0x00: AC = 0x00
        Op.PUSH, 0x00,          # 0x02: Push 0x00
        Op.LDI, 0xFF,           # 0x04: AC = 0xFF (non-zero, Z=0)
        Op.POP, 0x00,           # 0x06: Pop -> AC = 0x00, sets Z=1
        Op.JZ, JZ_TARGET,       # 0x08: Should jump because Z=1
        Op.LDI, 0xFF,           # 0x0A: Skipped if JZ works
        Op.OUT, 0x00,           # 0x0C: Skipped
        Op.HLT, 0x00,           # 0x0E: Skipped
        Op.NOP, 0x00,           # 0x10: Padding
        Op.NOP, 0x00,           # 0x12: Padding

        # JZ_TARGET (0x14):
        Op.LDI, 0xBB,           # 0x14: AC = 0xBB (success marker)
        Op.OUT, 0x00,           # 0x16: Output success
        Op.HLT, 0x00,           # 0x18: Halt
    ]
    expected = 0xBB

    dut._log.info("Testing POP sets Z flag correctly")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"POP Z flag test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_stack_with_subroutine_simulation(dut):
    """Test using stack to simulate a simple subroutine pattern"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_TEMP = 0x80

    # Simulate saving/restoring AC across operations
    # 1. Load value, push to save
    # 2. Do some computation
    # 3. Pop to restore original value
    program = [
        Op.LDI, 0x55,       # AC = 0x55 (original value)
        Op.PUSH, 0x00,      # Save AC on stack

        # Do some "work" that modifies AC
        Op.LDI, 0x10,       # AC = 0x10
        Op.STA, ADDR_TEMP,  # Store temporary
        Op.LDI, 0x20,       # AC = 0x20
        Op.ADD, ADDR_TEMP,  # AC = 0x30

        # Restore original AC
        Op.POP, 0x00,       # AC = 0x55 (restored)

        Op.OUT, 0x00,       # Output restored value
        Op.HLT, 0x00,
    ]
    expected = 0x55

    dut._log.info("Testing stack for save/restore pattern (subroutine simulation)")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"Save/restore test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


# ============================================================================
# CALL/RET INSTRUCTION TESTS
# ============================================================================

@cocotb.test()
async def test_call_ret_simple(dut):
    """Test simple CALL and RET - call subroutine and return"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Memory layout:
    # 0x00-0x07: Main program
    # 0x20-0x27: Subroutine
    SUBROUTINE_ADDR = 0x20

    # Main program calls subroutine, subroutine sets AC=0x42 and returns
    program = [
        # Main program (starts at 0x00)
        Op.LDI, 0x00,               # 0x00: AC = 0x00
        Op.CALL, SUBROUTINE_ADDR,   # 0x02: Call subroutine at 0x20
        Op.OUT, 0x00,               # 0x04: Output AC (should be 0x42 from subroutine)
        Op.HLT, 0x00,               # 0x06: Halt
    ]

    # Pad to subroutine address
    while len(program) < SUBROUTINE_ADDR:
        program.append(0x00)

    # Subroutine at 0x20
    program.extend([
        Op.LDI, 0x42,               # 0x20: AC = 0x42
        Op.RET, 0x00,               # 0x22: Return to caller
    ])

    expected = 0x42

    dut._log.info("Testing simple CALL/RET")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=3000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"CALL/RET test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_call_ret_with_parameter(dut):
    """Test CALL/RET where main passes value, subroutine modifies it"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    SUBROUTINE_ADDR = 0x20
    ADDR_PARAM = 0x80

    # Main: set AC=10, call subroutine that doubles it, output result
    program = [
        # Main program
        Op.LDI, 0x0A,               # 0x00: AC = 10
        Op.STA, ADDR_PARAM,         # 0x02: Store parameter
        Op.CALL, SUBROUTINE_ADDR,   # 0x04: Call double subroutine
        Op.LDA, ADDR_PARAM,         # 0x06: Load result
        Op.OUT, 0x00,               # 0x08: Output (should be 20)
        Op.HLT, 0x00,               # 0x0A: Halt
    ]

    # Pad to subroutine address
    while len(program) < SUBROUTINE_ADDR:
        program.append(0x00)

    # Subroutine: double the value at ADDR_PARAM
    program.extend([
        Op.LDA, ADDR_PARAM,         # 0x20: Load parameter
        Op.ADD, ADDR_PARAM,         # 0x22: Double it (AC = AC + param)
        Op.STA, ADDR_PARAM,         # 0x24: Store result
        Op.RET, 0x00,               # 0x26: Return
    ])

    expected = 20  # 10 * 2

    dut._log.info("Testing CALL/RET with parameter passing")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=3000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"CALL/RET param test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_nested_calls(dut):
    """Test nested CALL/RET - main calls sub1, sub1 calls sub2"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    SUB1_ADDR = 0x20
    SUB2_ADDR = 0x30
    ADDR_VAR = 0x80

    # Main calls sub1, sub1 calls sub2
    # sub2 sets AC=5, returns
    # sub1 adds 10, returns
    # main outputs result (should be 15)
    program = [
        # Main program
        Op.CALL, SUB1_ADDR,         # 0x00: Call sub1
        Op.OUT, 0x00,               # 0x02: Output result
        Op.HLT, 0x00,               # 0x04: Halt
    ]

    # Pad to sub1 address
    while len(program) < SUB1_ADDR:
        program.append(0x00)

    # Sub1: call sub2, then add 10
    program.extend([
        Op.CALL, SUB2_ADDR,         # 0x20: Call sub2
        Op.STA, ADDR_VAR,           # 0x22: Store sub2 result
        Op.LDI, 0x0A,               # 0x24: AC = 10
        Op.ADD, ADDR_VAR,           # 0x26: AC = 10 + sub2_result
        Op.RET, 0x00,               # 0x28: Return
    ])

    # Pad to sub2 address
    while len(program) < SUB2_ADDR:
        program.append(0x00)

    # Sub2: set AC=5 and return
    program.extend([
        Op.LDI, 0x05,               # 0x30: AC = 5
        Op.RET, 0x00,               # 0x32: Return
    ])

    expected = 15  # 5 + 10

    dut._log.info("Testing nested CALL/RET")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=4000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"Nested CALL/RET test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_call_ret_preserves_stack(dut):
    """Test that CALL/RET properly manages stack with PUSH/POP"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    SUBROUTINE_ADDR = 0x20

    # Main: push 0xAA, call subroutine, pop (should get 0xAA back)
    # Subroutine: push 0xBB, pop (internal use), return
    program = [
        # Main program
        Op.LDI, 0xAA,               # 0x00: AC = 0xAA
        Op.PUSH, 0x00,              # 0x02: Push 0xAA
        Op.CALL, SUBROUTINE_ADDR,   # 0x04: Call subroutine
        Op.POP, 0x00,               # 0x06: Pop (should be 0xAA)
        Op.OUT, 0x00,               # 0x08: Output
        Op.HLT, 0x00,               # 0x0A: Halt
    ]

    # Pad to subroutine address
    while len(program) < SUBROUTINE_ADDR:
        program.append(0x00)

    # Subroutine: uses stack internally but cleans up
    program.extend([
        Op.LDI, 0xBB,               # 0x20: AC = 0xBB
        Op.PUSH, 0x00,              # 0x22: Push 0xBB
        Op.POP, 0x00,               # 0x24: Pop (clean up)
        Op.RET, 0x00,               # 0x26: Return
    ])

    expected = 0xAA  # The value we pushed before CALL

    dut._log.info("Testing CALL/RET with PUSH/POP (stack integrity)")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=3000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"Stack integrity test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_multiple_calls_same_subroutine(dut):
    """Test calling the same subroutine multiple times"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    SUBROUTINE_ADDR = 0x20
    ADDR_COUNTER = 0x80

    # Main: initialize counter to 0, call increment subroutine 3 times
    program = [
        # Main program
        Op.LDI, 0x00,               # 0x00: AC = 0
        Op.STA, ADDR_COUNTER,       # 0x02: counter = 0
        Op.CALL, SUBROUTINE_ADDR,   # 0x04: Call increment (counter = 1)
        Op.CALL, SUBROUTINE_ADDR,   # 0x06: Call increment (counter = 2)
        Op.CALL, SUBROUTINE_ADDR,   # 0x08: Call increment (counter = 3)
        Op.LDA, ADDR_COUNTER,       # 0x0A: Load counter
        Op.OUT, 0x00,               # 0x0C: Output (should be 3)
        Op.HLT, 0x00,               # 0x0E: Halt
    ]

    # Pad to subroutine address
    while len(program) < SUBROUTINE_ADDR:
        program.append(0x00)

    # Subroutine: increment counter
    one_addr = 0x40
    program.extend([
        Op.LDA, ADDR_COUNTER,       # 0x20: Load counter
        Op.ADD, one_addr,           # 0x22: Add 1
        Op.STA, ADDR_COUNTER,       # 0x24: Store counter
        Op.RET, 0x00,               # 0x26: Return
    ])

    # Pad to constant address
    while len(program) < one_addr:
        program.append(0x00)

    # Constant 1 at 0x40
    program.append(0x01)

    expected = 3  # Called 3 times

    dut._log.info("Testing multiple calls to same subroutine")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=5000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"Multiple calls test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_loop_with_function_call(dut):
    """
    Test a for-loop that calls a function f(x) = x + 5 multiple times.

    This is a comprehensive test combining:
    - Loop with counter and conditional jump
    - CALL/RET to a subroutine
    - Subroutine performs f(x) = x + 5

    Program structure:
        Main:
            result = 0
            counter = 4
        loop:
            result = add5(result)   ; CALL add5
            counter = counter - 1
            if counter != 0: goto loop
            OUT result
            HLT

        add5:                       ; Subroutine: f(x) = x + 5
            AC = AC + 5
            RET

    Expected: 0 + 5 + 5 + 5 + 5 = 20
    """
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Memory layout
    SUBROUTINE_ADDR = 0x30  # add5 subroutine location
    ADDR_RESULT = 0x80      # Running result
    ADDR_COUNTER = 0x81     # Loop counter
    ADDR_NEG_ONE = 0x82     # Constant -1 (0xFF) for decrement
    ADDR_FIVE = 0x83        # Constant 5 for add5 function

    program = [
        # Initialize
        Op.LDI, 0x00,               # 0x00: AC = 0
        Op.STA, ADDR_RESULT,        # 0x02: result = 0
        Op.LDI, 0x04,               # 0x04: AC = 4
        Op.STA, ADDR_COUNTER,       # 0x06: counter = 4

        # Loop starts at 0x08
        Op.LDA, ADDR_RESULT,        # 0x08: AC = result
        Op.CALL, SUBROUTINE_ADDR,   # 0x0A: Call add5, AC = result + 5
        Op.STA, ADDR_RESULT,        # 0x0C: result = AC
        Op.LDA, ADDR_COUNTER,       # 0x0E: AC = counter
        Op.ADD, ADDR_NEG_ONE,       # 0x10: AC = counter - 1
        Op.STA, ADDR_COUNTER,       # 0x12: counter = AC
        Op.JNZ, 0x08,               # 0x14: If counter != 0, loop

        # Loop done, output result
        Op.LDA, ADDR_RESULT,        # 0x16: AC = result
        Op.OUT, 0x00,               # 0x18: Output result
        Op.HLT, 0x00,               # 0x1A: Halt
    ]

    # Pad to subroutine address
    while len(program) < SUBROUTINE_ADDR:
        program.append(0x00)

    # Subroutine add5: f(x) = x + 5
    program.extend([
        Op.ADD, ADDR_FIVE,          # 0x30: AC = AC + 5
        Op.RET, 0x00,               # 0x32: Return
    ])

    # Pad to data section
    while len(program) < ADDR_RESULT:
        program.append(0x00)

    # Data section
    program.append(0x00)            # 0x80: result (initialized to 0)
    program.append(0x00)            # 0x81: counter (will be set to 4)
    program.append(0xFF)            # 0x82: -1 (two's complement)
    program.append(0x05)            # 0x83: constant 5

    expected = 20  # 0 + 5 + 5 + 5 + 5 = 20

    dut._log.info("Testing for-loop with CALL to f(x) = x + 5")
    dut._log.info("Program: loop 4 times calling add5, expected result = 20")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=8000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"Loop with function call test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


# ============================================================================
# LCC EXTENSION INSTRUCTION TESTS (SUB, INC, DEC, XOR, SHL, SHR)
# ============================================================================

@cocotb.test()
async def test_sub_instruction(dut):
    """Test SUB instruction: AC = AC - MEM[addr]"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_A = 0x80

    # Test: 10 - 3 = 7
    program = [
        Op.LDI, 0x03,       # AC = 3
        Op.STA, ADDR_A,     # Store 3 at 0x80
        Op.LDI, 0x0A,       # AC = 10
        Op.SUB, ADDR_A,     # AC = 10 - 3 = 7
        Op.OUT, 0x00,       # Output 7
        Op.HLT, 0x00,
    ]
    expected = 7

    dut._log.info("Testing SUB instruction: 10 - 3 = 7")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"SUB test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_sub_underflow(dut):
    """Test SUB instruction with underflow (wrap-around)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_A = 0x80

    # Test: 3 - 10 = -7 = 0xF9 (underflow/wrap)
    program = [
        Op.LDI, 0x0A,       # AC = 10
        Op.STA, ADDR_A,     # Store 10 at 0x80
        Op.LDI, 0x03,       # AC = 3
        Op.SUB, ADDR_A,     # AC = 3 - 10 = -7 = 0xF9
        Op.OUT, 0x00,       # Output 0xF9
        Op.HLT, 0x00,
    ]
    expected = 0xF9  # -7 in two's complement

    dut._log.info("Testing SUB instruction with underflow: 3 - 10 = 0xF9")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"SUB underflow test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_inc_instruction(dut):
    """Test INC instruction: AC = AC + 1"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: INC from 5 to 6
    program = [
        Op.LDI, 0x05,       # AC = 5
        Op.INC, 0x00,       # AC = 5 + 1 = 6
        Op.OUT, 0x00,       # Output 6
        Op.HLT, 0x00,
    ]
    expected = 6

    dut._log.info("Testing INC instruction: 5 + 1 = 6")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"INC test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_inc_overflow(dut):
    """Test INC instruction overflow: 0xFF + 1 = 0x00"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: INC from 0xFF to 0x00 (overflow)
    program = [
        Op.LDI, 0xFF,       # AC = 0xFF
        Op.INC, 0x00,       # AC = 0xFF + 1 = 0x00 (overflow)
        Op.OUT, 0x00,       # Output 0
        Op.HLT, 0x00,
    ]
    expected = 0x00

    dut._log.info("Testing INC instruction overflow: 0xFF + 1 = 0x00")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"INC overflow test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_dec_instruction(dut):
    """Test DEC instruction: AC = AC - 1"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: DEC from 10 to 9
    program = [
        Op.LDI, 0x0A,       # AC = 10
        Op.DEC, 0x00,       # AC = 10 - 1 = 9
        Op.OUT, 0x00,       # Output 9
        Op.HLT, 0x00,
    ]
    expected = 9

    dut._log.info("Testing DEC instruction: 10 - 1 = 9")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"DEC test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_dec_underflow(dut):
    """Test DEC instruction underflow: 0x00 - 1 = 0xFF"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: DEC from 0x00 to 0xFF (underflow)
    program = [
        Op.LDI, 0x00,       # AC = 0x00
        Op.DEC, 0x00,       # AC = 0x00 - 1 = 0xFF (underflow)
        Op.OUT, 0x00,       # Output 0xFF
        Op.HLT, 0x00,
    ]
    expected = 0xFF

    dut._log.info("Testing DEC instruction underflow: 0x00 - 1 = 0xFF")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"DEC underflow test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_xor_instruction(dut):
    """Test XOR instruction: AC = AC ^ MEM[addr]"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_A = 0x80

    # Test: 0xAA XOR 0x55 = 0xFF
    program = [
        Op.LDI, 0x55,       # AC = 0x55
        Op.STA, ADDR_A,     # Store 0x55 at 0x80
        Op.LDI, 0xAA,       # AC = 0xAA
        Op.XOR, ADDR_A,     # AC = 0xAA ^ 0x55 = 0xFF
        Op.OUT, 0x00,       # Output 0xFF
        Op.HLT, 0x00,
    ]
    expected = 0xFF

    dut._log.info("Testing XOR instruction: 0xAA ^ 0x55 = 0xFF")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"XOR test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_xor_self_zero(dut):
    """Test XOR with self produces zero: A ^ A = 0"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_A = 0x80

    # Test: 0x42 XOR 0x42 = 0x00
    program = [
        Op.LDI, 0x42,       # AC = 0x42
        Op.STA, ADDR_A,     # Store 0x42 at 0x80
        Op.XOR, ADDR_A,     # AC = 0x42 ^ 0x42 = 0x00
        Op.OUT, 0x00,       # Output 0x00
        Op.HLT, 0x00,
    ]
    expected = 0x00

    dut._log.info("Testing XOR self: 0x42 ^ 0x42 = 0x00")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"XOR self test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_shl_instruction(dut):
    """Test SHL instruction: AC = AC << 1"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: 0x15 << 1 = 0x2A
    program = [
        Op.LDI, 0x15,       # AC = 0x15 (00010101)
        Op.SHL, 0x00,       # AC = 0x15 << 1 = 0x2A (00101010)
        Op.OUT, 0x00,       # Output 0x2A
        Op.HLT, 0x00,
    ]
    expected = 0x2A

    dut._log.info("Testing SHL instruction: 0x15 << 1 = 0x2A")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"SHL test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_shl_msb_loss(dut):
    """Test SHL instruction with MSB loss: 0x80 << 1 = 0x00"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: 0x80 << 1 = 0x00 (MSB shifted out)
    program = [
        Op.LDI, 0x80,       # AC = 0x80 (10000000)
        Op.SHL, 0x00,       # AC = 0x80 << 1 = 0x00 (00000000)
        Op.OUT, 0x00,       # Output 0x00
        Op.HLT, 0x00,
    ]
    expected = 0x00

    dut._log.info("Testing SHL instruction MSB loss: 0x80 << 1 = 0x00")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"SHL MSB loss test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_shr_instruction(dut):
    """Test SHR instruction: AC = AC >> 1"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: 0x2A >> 1 = 0x15
    program = [
        Op.LDI, 0x2A,       # AC = 0x2A (00101010)
        Op.SHR, 0x00,       # AC = 0x2A >> 1 = 0x15 (00010101)
        Op.OUT, 0x00,       # Output 0x15
        Op.HLT, 0x00,
    ]
    expected = 0x15

    dut._log.info("Testing SHR instruction: 0x2A >> 1 = 0x15")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"SHR test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_shr_lsb_loss(dut):
    """Test SHR instruction with LSB loss: 0x01 >> 1 = 0x00"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: 0x01 >> 1 = 0x00 (LSB shifted out)
    program = [
        Op.LDI, 0x01,       # AC = 0x01 (00000001)
        Op.SHR, 0x00,       # AC = 0x01 >> 1 = 0x00 (00000000)
        Op.OUT, 0x00,       # Output 0x00
        Op.HLT, 0x00,
    ]
    expected = 0x00

    dut._log.info("Testing SHR instruction LSB loss: 0x01 >> 1 = 0x00")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"SHR LSB loss test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_inc_dec_chain(dut):
    """Test INC and DEC in sequence: INC 3 times, DEC 2 times"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: Start at 10, INC 3 times (=13), DEC 2 times (=11)
    program = [
        Op.LDI, 0x0A,       # AC = 10
        Op.INC, 0x00,       # AC = 11
        Op.INC, 0x00,       # AC = 12
        Op.INC, 0x00,       # AC = 13
        Op.DEC, 0x00,       # AC = 12
        Op.DEC, 0x00,       # AC = 11
        Op.OUT, 0x00,       # Output 11
        Op.HLT, 0x00,
    ]
    expected = 11

    dut._log.info("Testing INC/DEC chain: 10 + 3 - 2 = 11")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"INC/DEC chain test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_multiply_by_2_using_shl(dut):
    """Test multiplication by 2 using SHL: 25 * 2 = 50"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: 25 * 2 = 50 using SHL
    program = [
        Op.LDI, 25,         # AC = 25
        Op.SHL, 0x00,       # AC = 25 << 1 = 50
        Op.OUT, 0x00,       # Output 50
        Op.HLT, 0x00,
    ]
    expected = 50

    dut._log.info("Testing multiply by 2 using SHL: 25 * 2 = 50")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"Multiply by 2 test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_divide_by_2_using_shr(dut):
    """Test division by 2 using SHR: 50 / 2 = 25"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: 50 / 2 = 25 using SHR
    program = [
        Op.LDI, 50,         # AC = 50
        Op.SHR, 0x00,       # AC = 50 >> 1 = 25
        Op.OUT, 0x00,       # Output 25
        Op.HLT, 0x00,
    ]
    expected = 25

    dut._log.info("Testing divide by 2 using SHR: 50 / 2 = 25")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"Divide by 2 test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_countdown_with_dec(dut):
    """Test countdown loop using DEC instruction"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_COUNTER = 0x80

    # Count down from 5 to 0 using DEC and JNZ
    program = [
        Op.LDI, 0x05,           # 0x00: AC = 5
        # Loop at 0x02:
        Op.DEC, 0x00,           # 0x02: AC = AC - 1
        Op.JNZ, 0x02,           # 0x04: if AC != 0, goto 0x02
        Op.OUT, 0x00,           # 0x06: Output 0 (loop done)
        Op.HLT, 0x00,           # 0x08: Halt
    ]
    expected = 0

    dut._log.info("Testing countdown with DEC: 5 -> 0")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"Countdown with DEC failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_lcc_comprehensive(dut):
    """Comprehensive test using all LCC extension instructions"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_A = 0x80
    ADDR_B = 0x81

    # Program:
    # 1. LDI 20, store at A
    # 2. LDI 5, store at B
    # 3. LDA A, SUB B -> 15
    # 4. INC -> 16
    # 5. SHL -> 32
    # 6. SHR -> 16
    # 7. DEC -> 15
    # 8. Store at A, LDI 0x0F, XOR A -> 0x0F ^ 0x0F = 0
    # 9. Output 0
    program = [
        Op.LDI, 20,         # AC = 20
        Op.STA, ADDR_A,     # A = 20
        Op.LDI, 5,          # AC = 5
        Op.STA, ADDR_B,     # B = 5
        Op.LDA, ADDR_A,     # AC = 20
        Op.SUB, ADDR_B,     # AC = 20 - 5 = 15
        Op.INC, 0x00,       # AC = 16
        Op.SHL, 0x00,       # AC = 32
        Op.SHR, 0x00,       # AC = 16
        Op.DEC, 0x00,       # AC = 15
        Op.STA, ADDR_A,     # A = 15
        Op.LDI, 0x0F,       # AC = 15 (0x0F)
        Op.XOR, ADDR_A,     # AC = 0x0F ^ 0x0F = 0
        Op.OUT, 0x00,       # Output 0
        Op.HLT, 0x00,
    ]
    expected = 0

    dut._log.info("Testing comprehensive LCC extension program")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=3000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"Comprehensive LCC test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


# ============================================================================
# X REGISTER EXTENSION INSTRUCTION TESTS (LDX, STX, LDXI, TAX, TXA, INX)
# ============================================================================

@cocotb.test()
async def test_ldxi_instruction(dut):
    """Test LDXI instruction: X = immediate value"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: Load immediate 0x42 to X, then transfer to AC and output
    program = [
        Op.LDXI, 0x42,      # X = 0x42
        Op.TXA, 0x00,       # AC = X = 0x42
        Op.OUT, 0x00,       # Output 0x42
        Op.HLT, 0x00,
    ]
    expected = 0x42

    dut._log.info("Testing LDXI instruction: X = 0x42")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"LDXI test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_tax_instruction(dut):
    """Test TAX instruction: X = AC"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: Load AC with 0x55, transfer to X, then transfer back and output
    program = [
        Op.LDI, 0x55,       # AC = 0x55
        Op.TAX, 0x00,       # X = AC = 0x55
        Op.LDI, 0x00,       # AC = 0x00 (clear AC)
        Op.TXA, 0x00,       # AC = X = 0x55
        Op.OUT, 0x00,       # Output 0x55
        Op.HLT, 0x00,
    ]
    expected = 0x55

    dut._log.info("Testing TAX instruction: X = AC")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"TAX test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_txa_instruction(dut):
    """Test TXA instruction: AC = X"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: Load X with immediate, then transfer to AC and output
    program = [
        Op.LDXI, 0xAB,      # X = 0xAB
        Op.TXA, 0x00,       # AC = X = 0xAB
        Op.OUT, 0x00,       # Output 0xAB
        Op.HLT, 0x00,
    ]
    expected = 0xAB

    dut._log.info("Testing TXA instruction: AC = X")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"TXA test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_inx_instruction(dut):
    """Test INX instruction: X = X + 1"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: Load X with 0x09, increment to 0x0A, output
    program = [
        Op.LDXI, 0x09,      # X = 0x09
        Op.INX, 0x00,       # X = X + 1 = 0x0A
        Op.TXA, 0x00,       # AC = X = 0x0A
        Op.OUT, 0x00,       # Output 0x0A
        Op.HLT, 0x00,
    ]
    expected = 0x0A

    dut._log.info("Testing INX instruction: X = X + 1")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"INX test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_inx_overflow(dut):
    """Test INX instruction overflow: 0xFF + 1 = 0x00"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: Load X with 0xFF, increment to 0x00 (overflow)
    program = [
        Op.LDXI, 0xFF,      # X = 0xFF
        Op.INX, 0x00,       # X = X + 1 = 0x00 (overflow)
        Op.TXA, 0x00,       # AC = X = 0x00
        Op.OUT, 0x00,       # Output 0x00
        Op.HLT, 0x00,
    ]
    expected = 0x00

    dut._log.info("Testing INX instruction overflow: 0xFF + 1 = 0x00")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"INX overflow test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_ldx_stx_instructions(dut):
    """Test LDX and STX instructions: memory load/store for X register"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_A = 0x80
    ADDR_B = 0x81

    # Test: Store value to memory, load into X, store X elsewhere, verify
    program = [
        Op.LDI, 0x37,       # AC = 0x37
        Op.STA, ADDR_A,     # MEM[0x80] = 0x37
        Op.LDX, ADDR_A,     # X = MEM[0x80] = 0x37
        Op.STX, ADDR_B,     # MEM[0x81] = X = 0x37
        Op.LDI, 0x00,       # AC = 0x00 (clear)
        Op.LDA, ADDR_B,     # AC = MEM[0x81] = 0x37
        Op.OUT, 0x00,       # Output 0x37
        Op.HLT, 0x00,
    ]
    expected = 0x37

    dut._log.info("Testing LDX/STX instructions: X = MEM[addr], MEM[addr] = X")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=3000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"LDX/STX test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_lda_indexed(dut):
    """Test LDA with indexed addressing: AC = MEM[addr + X]"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Set up an array in memory at 0x80-0x83 with values [10, 20, 30, 40]
    ARRAY_BASE = 0x80

    program = [
        # Initialize array
        Op.LDI, 10,
        Op.STA, ARRAY_BASE,      # array[0] = 10
        Op.LDI, 20,
        Op.STA, ARRAY_BASE + 1,  # array[1] = 20
        Op.LDI, 30,
        Op.STA, ARRAY_BASE + 2,  # array[2] = 30
        Op.LDI, 40,
        Op.STA, ARRAY_BASE + 3,  # array[3] = 40

        # Load X with index 2
        Op.LDXI, 0x02,           # X = 2

        # Load array[2] using indexed addressing
        Op.LDA_X, ARRAY_BASE,    # AC = MEM[0x80 + 2] = MEM[0x82] = 30

        Op.OUT, 0x00,            # Output 30
        Op.HLT, 0x00,
    ]
    expected = 30

    dut._log.info("Testing LDA with indexed addressing: AC = MEM[0x80 + 2] = 30")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=4000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"LDA indexed test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_sta_indexed(dut):
    """Test STA with indexed addressing: MEM[addr + X] = AC"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ARRAY_BASE = 0x80

    program = [
        # Initialize X to 3 (index into array)
        Op.LDXI, 0x03,           # X = 3

        # Store value 99 at array[3]
        Op.LDI, 99,              # AC = 99
        Op.STA_X, ARRAY_BASE,    # MEM[0x80 + 3] = 99

        # Clear AC and reload from the indexed location to verify
        Op.LDI, 0x00,            # AC = 0
        Op.LDA_X, ARRAY_BASE,    # AC = MEM[0x80 + 3] = 99

        Op.OUT, 0x00,            # Output 99
        Op.HLT, 0x00,
    ]
    expected = 99

    dut._log.info("Testing STA with indexed addressing: MEM[0x80 + 3] = 99")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=3000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"STA indexed test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_array_sum_indexed(dut):
    """Test array sum using indexed addressing: sum of [5, 10, 15, 20] = 50"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ARRAY_BASE = 0x80
    ADDR_SUM = 0x90
    ADDR_COUNT = 0x91
    ADDR_NEG_ONE = 0x92

    # Program to sum array elements using indexed addressing
    program = [
        # Initialize array at 0x80-0x83
        Op.LDI, 5,
        Op.STA, ARRAY_BASE,      # array[0] = 5
        Op.LDI, 10,
        Op.STA, ARRAY_BASE + 1,  # array[1] = 10
        Op.LDI, 15,
        Op.STA, ARRAY_BASE + 2,  # array[2] = 15
        Op.LDI, 20,
        Op.STA, ARRAY_BASE + 3,  # array[3] = 20

        # Initialize sum = 0, count = 4, neg_one = -1
        Op.LDI, 0,
        Op.STA, ADDR_SUM,        # sum = 0
        Op.LDI, 4,
        Op.STA, ADDR_COUNT,      # count = 4
        Op.LDI, 0xFF,
        Op.STA, ADDR_NEG_ONE,    # neg_one = -1

        # Initialize X = 0 (array index)
        Op.LDXI, 0x00,           # 0x1C: X = 0

        # Loop: sum += array[X], X++, count--, if count != 0 goto loop
        # LOOP_ADDR = 0x1E (30 bytes from start)
        Op.LDA_X, ARRAY_BASE,    # 0x1E: AC = array[X]
        Op.ADD, ADDR_SUM,        # 0x20: AC = AC + sum
        Op.STA, ADDR_SUM,        # 0x22: sum = AC
        Op.INX, 0x00,            # 0x24: X++
        Op.LDA, ADDR_COUNT,      # 0x26: AC = count
        Op.ADD, ADDR_NEG_ONE,    # 0x28: AC = count - 1
        Op.STA, ADDR_COUNT,      # 0x2A: count = AC
        Op.JNZ, 0x1E,            # 0x2C: if count != 0, loop

        # Output sum
        Op.LDA, ADDR_SUM,        # 0x2E: AC = sum
        Op.OUT, 0x00,            # 0x30: Output sum
        Op.HLT, 0x00,            # 0x32: Halt
    ]
    expected = 50  # 5 + 10 + 15 + 20 = 50

    dut._log.info("Testing array sum using indexed addressing: sum of [5, 10, 15, 20] = 50")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=8000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"Array sum test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_x_register_loop(dut):
    """Test X register as loop counter: count from 0 to 5"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_FIVE = 0x80

    # Count how many times X was incremented (should be 5)
    program = [
        Op.LDI, 0x05,       # AC = 5
        Op.STA, ADDR_FIVE,  # Store 5

        Op.LDXI, 0x00,      # X = 0

        # LOOP_ADDR = 0x06
        Op.INX, 0x00,       # 0x06: X++
        Op.TXA, 0x00,       # 0x08: AC = X
        Op.SUB, ADDR_FIVE,  # 0x0A: AC = X - 5
        Op.JNZ, 0x06,       # 0x0C: if X != 5, loop

        Op.TXA, 0x00,       # AC = X = 5
        Op.OUT, 0x00,       # Output 5
        Op.HLT, 0x00,
    ]
    expected = 5

    dut._log.info("Testing X register as loop counter")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=5000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"X register loop test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_array_fill_indexed(dut):
    """Test filling array using indexed addressing and INX"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ARRAY_BASE = 0x80
    ADDR_VALUE = 0x90
    ADDR_COUNT = 0x91
    ADDR_NEG_ONE = 0x92

    # Fill array[0..3] with value 42, then read back array[2] to verify
    program = [
        # Initialize value = 42, count = 4, neg_one = -1
        Op.LDI, 42,
        Op.STA, ADDR_VALUE,      # value = 42
        Op.LDI, 4,
        Op.STA, ADDR_COUNT,      # count = 4
        Op.LDI, 0xFF,
        Op.STA, ADDR_NEG_ONE,    # neg_one = -1

        # Initialize X = 0
        Op.LDXI, 0x00,           # 0x0C: X = 0

        # LOOP: fill array[X] = 42
        # LOOP_ADDR = 0x0E (14 bytes from start)
        Op.LDA, ADDR_VALUE,      # 0x0E: AC = 42
        Op.STA_X, ARRAY_BASE,    # 0x10: array[X] = 42
        Op.INX, 0x00,            # 0x12: X++
        Op.LDA, ADDR_COUNT,      # 0x14: AC = count
        Op.ADD, ADDR_NEG_ONE,    # 0x16: AC = count - 1
        Op.STA, ADDR_COUNT,      # 0x18: count = AC
        Op.JNZ, 0x0E,            # 0x1A: if count != 0, loop

        # Read back array[2] to verify
        Op.LDXI, 0x02,           # X = 2
        Op.LDA_X, ARRAY_BASE,    # AC = array[2] = 42
        Op.OUT, 0x00,            # Output 42
        Op.HLT, 0x00,
    ]
    expected = 42

    dut._log.info("Testing array fill with indexed addressing")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=8000)

    dut._log.info(f"I/O output received: {result} (expected: {expected})")
    assert result == expected, f"Array fill test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_txa_flags(dut):
    """Test TXA instruction sets flags correctly"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: TXA with zero value should set Z flag
    # After TXA, JZ should jump if Z flag is set
    program = [
        Op.LDXI, 0x00,      # 0x00: X = 0
        Op.TXA, 0x00,       # 0x02: AC = X = 0 (should set Z flag)
        Op.JZ, 0x0C,        # 0x04: If Z flag set, jump to 0x0C (success)
        Op.LDI, 0xFF,       # 0x06: If we get here, test failed
        Op.OUT, 0x00,       # 0x08: Output failure
        Op.HLT, 0x00,       # 0x0A: Halt (failure path)

        # Jump target (0x0C):
        Op.LDI, 0x42,       # 0x0C: Success marker
        Op.OUT, 0x00,       # 0x0E: Output 0x42
        Op.HLT, 0x00,       # 0x10: Halt (success path)
    ]
    expected = 0x42

    dut._log.info("Testing TXA instruction sets Z flag correctly")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"TXA Z flag test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_txa_negative_flag(dut):
    """Test TXA instruction sets N flag correctly"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: TXA with negative value (MSB=1) should set N flag
    program = [
        Op.LDXI, 0x80,      # 0x00: X = 0x80 (negative, MSB=1)
        Op.TXA, 0x00,       # 0x02: AC = X = 0x80 (should set N flag)
        Op.JN, 0x0C,        # 0x04: If N flag set, jump to 0x0C (success)
        Op.LDI, 0xFF,       # 0x06: If we get here, test failed
        Op.OUT, 0x00,       # 0x08: Output failure
        Op.HLT, 0x00,       # 0x0A: Halt (failure path)

        # Jump target (0x0C):
        Op.LDI, 0x33,       # 0x0C: Success marker
        Op.OUT, 0x00,       # 0x0E: Output 0x33
        Op.HLT, 0x00,       # 0x10: Halt (success path)
    ]
    expected = 0x33

    dut._log.info("Testing TXA instruction sets N flag correctly")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"TXA N flag test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


# ============================================================================
# LCC COMPILER EXTENSION TESTS: NEG, CMP, JC, JNC
# ============================================================================

@cocotb.test()
async def test_neg_positive(dut):
    """Test NEG instruction with positive value: NEG(5) = -5 = 0xFB"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: AC = 5, NEG -> AC = -5 = 0xFB (two's complement)
    program = [
        Op.LDI, 0x05,       # 0x00: AC = 5
        Op.NEG, 0x00,       # 0x02: AC = -AC = -5 = 0xFB
        Op.OUT, 0x00,       # 0x04: Output AC
        Op.HLT, 0x00,       # 0x06: Halt
    ]
    expected = 0xFB  # -5 in two's complement

    dut._log.info("Testing NEG instruction: NEG(5) = 0xFB")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"NEG test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_neg_negative(dut):
    """Test NEG instruction with negative value: NEG(-1) = 1"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: AC = 0xFF (-1), NEG -> AC = 1
    program = [
        Op.LDI, 0xFF,       # 0x00: AC = 0xFF (-1)
        Op.NEG, 0x00,       # 0x02: AC = -AC = 1
        Op.OUT, 0x00,       # 0x04: Output AC
        Op.HLT, 0x00,       # 0x06: Halt
    ]
    expected = 0x01

    dut._log.info("Testing NEG instruction: NEG(0xFF) = 0x01")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"NEG test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_neg_zero(dut):
    """Test NEG instruction with zero: NEG(0) = 0"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: AC = 0, NEG -> AC = 0
    program = [
        Op.LDI, 0x00,       # 0x00: AC = 0
        Op.NEG, 0x00,       # 0x02: AC = -AC = 0
        Op.OUT, 0x00,       # 0x04: Output AC
        Op.HLT, 0x00,       # 0x06: Halt
    ]
    expected = 0x00

    dut._log.info("Testing NEG instruction: NEG(0) = 0")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"NEG test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_neg_sets_n_flag(dut):
    """Test NEG instruction sets N flag correctly"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: NEG(1) = 0xFF (negative), should set N flag
    program = [
        Op.LDI, 0x01,       # 0x00: AC = 1
        Op.NEG, 0x00,       # 0x02: AC = -1 = 0xFF (sets N flag)
        Op.JN, 0x0C,        # 0x04: If N flag set, jump to success
        Op.LDI, 0xFF,       # 0x06: Failure marker
        Op.OUT, 0x00,       # 0x08: Output failure
        Op.HLT, 0x00,       # 0x0A: Halt (failure path)

        # Jump target (0x0C):
        Op.LDI, 0x42,       # 0x0C: Success marker
        Op.OUT, 0x00,       # 0x0E: Output 0x42
        Op.HLT, 0x00,       # 0x10: Halt (success path)
    ]
    expected = 0x42

    dut._log.info("Testing NEG instruction sets N flag correctly")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"NEG N flag test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_neg_sets_z_flag(dut):
    """Test NEG instruction sets Z flag when result is zero"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: NEG(0) = 0, should set Z flag
    program = [
        Op.LDI, 0x00,       # 0x00: AC = 0
        Op.NEG, 0x00,       # 0x02: AC = 0 (sets Z flag)
        Op.JZ, 0x0C,        # 0x04: If Z flag set, jump to success
        Op.LDI, 0xFF,       # 0x06: Failure marker
        Op.OUT, 0x00,       # 0x08: Output failure
        Op.HLT, 0x00,       # 0x0A: Halt (failure path)

        # Jump target (0x0C):
        Op.LDI, 0x43,       # 0x0C: Success marker
        Op.OUT, 0x00,       # 0x0E: Output 0x43
        Op.HLT, 0x00,       # 0x10: Halt (success path)
    ]
    expected = 0x43

    dut._log.info("Testing NEG instruction sets Z flag correctly")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"NEG Z flag test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_cmp_equal(dut):
    """Test CMP instruction: compare equal values (sets Z flag)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # Test: AC = 10, CMP with MEM[0x80] = 10, should set Z flag
    program = [
        Op.LDI, 0x0A,       # 0x00: AC = 10
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 10
        Op.CMP, ADDR_VAL,   # 0x04: Compare AC(10) with MEM(10) -> Z flag set
        Op.JZ, 0x0E,        # 0x06: If Z flag set, jump to success
        Op.LDI, 0xFF,       # 0x08: Failure marker
        Op.OUT, 0x00,       # 0x0A: Output failure
        Op.HLT, 0x00,       # 0x0C: Halt (failure path)

        # Jump target (0x0E):
        Op.LDI, 0x44,       # 0x0E: Success marker
        Op.OUT, 0x00,       # 0x10: Output 0x44
        Op.HLT, 0x00,       # 0x12: Halt (success path)
    ]
    expected = 0x44

    dut._log.info("Testing CMP instruction: equal values set Z flag")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"CMP equal test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_cmp_not_equal(dut):
    """Test CMP instruction: compare different values (Z flag clear)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # Test: AC = 10, CMP with MEM[0x80] = 5, should NOT set Z flag
    program = [
        Op.LDI, 0x05,       # 0x00: AC = 5
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 5
        Op.LDI, 0x0A,       # 0x04: AC = 10
        Op.CMP, ADDR_VAL,   # 0x06: Compare AC(10) with MEM(5) -> Z flag clear
        Op.JNZ, 0x10,       # 0x08: If Z flag clear, jump to success
        Op.LDI, 0xFF,       # 0x0A: Failure marker
        Op.OUT, 0x00,       # 0x0C: Output failure
        Op.HLT, 0x00,       # 0x0E: Halt (failure path)

        # Jump target (0x10):
        Op.LDI, 0x45,       # 0x10: Success marker
        Op.OUT, 0x00,       # 0x12: Output 0x45
        Op.HLT, 0x00,       # 0x14: Halt (success path)
    ]
    expected = 0x45

    dut._log.info("Testing CMP instruction: different values clear Z flag")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"CMP not equal test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_cmp_preserves_ac(dut):
    """Test CMP instruction does not modify AC"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # Test: AC = 10, CMP with MEM[0x80] = 5, AC should still be 10
    program = [
        Op.LDI, 0x05,       # 0x00: AC = 5
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 5
        Op.LDI, 0x0A,       # 0x04: AC = 10
        Op.CMP, ADDR_VAL,   # 0x06: Compare AC(10) with MEM(5), AC should stay 10
        Op.OUT, 0x00,       # 0x08: Output AC (should be 10)
        Op.HLT, 0x00,       # 0x0A: Halt
    ]
    expected = 0x0A  # AC should still be 10

    dut._log.info("Testing CMP instruction preserves AC")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"CMP preserves AC test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_cmp_less_than_sets_n(dut):
    """Test CMP instruction: AC < MEM sets N flag (result is negative)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # Test: AC = 5, CMP with MEM[0x80] = 10 -> 5 - 10 = -5 (negative, sets N)
    program = [
        Op.LDI, 0x0A,       # 0x00: AC = 10
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 10
        Op.LDI, 0x05,       # 0x04: AC = 5
        Op.CMP, ADDR_VAL,   # 0x06: Compare AC(5) with MEM(10) -> N flag set
        Op.JN, 0x10,        # 0x08: If N flag set, jump to success
        Op.LDI, 0xFF,       # 0x0A: Failure marker
        Op.OUT, 0x00,       # 0x0C: Output failure
        Op.HLT, 0x00,       # 0x0E: Halt (failure path)

        # Jump target (0x10):
        Op.LDI, 0x46,       # 0x10: Success marker
        Op.OUT, 0x00,       # 0x12: Output 0x46
        Op.HLT, 0x00,       # 0x14: Halt (success path)
    ]
    expected = 0x46

    dut._log.info("Testing CMP instruction: AC < MEM sets N flag")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"CMP less than test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_jc_taken(dut):
    """Test JC instruction: jump when carry flag is set"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # Test: Create carry by adding 0xFF + 0x02 = 0x101 (carry set)
    # Then JC should take the jump
    program = [
        Op.LDI, 0x02,       # 0x00: AC = 2
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 2
        Op.LDI, 0xFF,       # 0x04: AC = 255
        Op.ADD, ADDR_VAL,   # 0x06: AC = 255 + 2 = 257 -> 0x01 with carry
        Op.JC, 0x12,        # 0x08: If carry set, jump to success
        Op.LDI, 0xFF,       # 0x0A: Failure marker
        Op.OUT, 0x00,       # 0x0C: Output failure
        Op.HLT, 0x00,       # 0x0E: Halt (failure path)
        Op.NOP, 0x00,       # 0x10: Padding

        # Jump target (0x12):
        Op.LDI, 0x47,       # 0x12: Success marker
        Op.OUT, 0x00,       # 0x14: Output 0x47
        Op.HLT, 0x00,       # 0x16: Halt (success path)
    ]
    expected = 0x47

    dut._log.info("Testing JC instruction: jump when carry is set")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"JC taken test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_jc_not_taken(dut):
    """Test JC instruction: don't jump when carry flag is clear"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # Test: Add without overflow: 5 + 3 = 8 (no carry)
    # Then JC should NOT take the jump
    program = [
        Op.LDI, 0x03,       # 0x00: AC = 3
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 3
        Op.LDI, 0x05,       # 0x04: AC = 5
        Op.ADD, ADDR_VAL,   # 0x06: AC = 5 + 3 = 8 (no carry)
        Op.JC, 0x12,        # 0x08: If carry set, jump (should NOT jump)
        Op.LDI, 0x48,       # 0x0A: Success marker (carry not set)
        Op.OUT, 0x00,       # 0x0C: Output 0x48
        Op.HLT, 0x00,       # 0x0E: Halt (success path)
        Op.NOP, 0x00,       # 0x10: Padding

        # Jump target (0x12): - should NOT reach here
        Op.LDI, 0xFF,       # 0x12: Failure marker
        Op.OUT, 0x00,       # 0x14: Output failure
        Op.HLT, 0x00,       # 0x16: Halt (failure path)
    ]
    expected = 0x48

    dut._log.info("Testing JC instruction: don't jump when carry is clear")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"JC not taken test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_jnc_taken(dut):
    """Test JNC instruction: jump when carry flag is clear"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # Test: Add without overflow: 5 + 3 = 8 (no carry)
    # Then JNC should take the jump
    program = [
        Op.LDI, 0x03,       # 0x00: AC = 3
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 3
        Op.LDI, 0x05,       # 0x04: AC = 5
        Op.ADD, ADDR_VAL,   # 0x06: AC = 5 + 3 = 8 (no carry)
        Op.JNC, 0x12,       # 0x08: If no carry, jump to success
        Op.LDI, 0xFF,       # 0x0A: Failure marker
        Op.OUT, 0x00,       # 0x0C: Output failure
        Op.HLT, 0x00,       # 0x0E: Halt (failure path)
        Op.NOP, 0x00,       # 0x10: Padding

        # Jump target (0x12):
        Op.LDI, 0x49,       # 0x12: Success marker
        Op.OUT, 0x00,       # 0x14: Output 0x49
        Op.HLT, 0x00,       # 0x16: Halt (success path)
    ]
    expected = 0x49

    dut._log.info("Testing JNC instruction: jump when carry is clear")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"JNC taken test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_jnc_not_taken(dut):
    """Test JNC instruction: don't jump when carry flag is set"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # Test: Create carry by adding 0xFF + 0x02 = 0x101 (carry set)
    # Then JNC should NOT take the jump
    program = [
        Op.LDI, 0x02,       # 0x00: AC = 2
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 2
        Op.LDI, 0xFF,       # 0x04: AC = 255
        Op.ADD, ADDR_VAL,   # 0x06: AC = 255 + 2 = 257 -> carry set
        Op.JNC, 0x12,       # 0x08: If no carry, jump (should NOT jump)
        Op.LDI, 0x4A,       # 0x0A: Success marker (carry is set)
        Op.OUT, 0x00,       # 0x0C: Output 0x4A
        Op.HLT, 0x00,       # 0x0E: Halt (success path)
        Op.NOP, 0x00,       # 0x10: Padding

        # Jump target (0x12): - should NOT reach here
        Op.LDI, 0xFF,       # 0x12: Failure marker
        Op.OUT, 0x00,       # 0x14: Output failure
        Op.HLT, 0x00,       # 0x16: Halt (failure path)
    ]
    expected = 0x4A

    dut._log.info("Testing JNC instruction: don't jump when carry is set")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"JNC not taken test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_sub_sets_carry(dut):
    """Test SUB instruction sets carry flag on borrow"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # Test: 5 - 10 = -5 (borrow/carry set)
    # Use JC to verify carry is set after SUB
    program = [
        Op.LDI, 0x0A,       # 0x00: AC = 10
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 10
        Op.LDI, 0x05,       # 0x04: AC = 5
        Op.SUB, ADDR_VAL,   # 0x06: AC = 5 - 10 = -5 (carry/borrow set)
        Op.JC, 0x12,        # 0x08: If carry set, jump to success
        Op.LDI, 0xFF,       # 0x0A: Failure marker
        Op.OUT, 0x00,       # 0x0C: Output failure
        Op.HLT, 0x00,       # 0x0E: Halt (failure path)
        Op.NOP, 0x00,       # 0x10: Padding

        # Jump target (0x12):
        Op.LDI, 0x4B,       # 0x12: Success marker
        Op.OUT, 0x00,       # 0x14: Output 0x4B
        Op.HLT, 0x00,       # 0x16: Halt (success path)
    ]
    expected = 0x4B

    dut._log.info("Testing SUB instruction sets carry on borrow")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"SUB carry test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_cmp_sets_carry_on_borrow(dut):
    """Test CMP instruction sets carry flag on borrow (AC < MEM)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # Test: CMP 5 with 10 -> 5 - 10 causes borrow (carry set)
    # Use JC to verify carry is set after CMP
    program = [
        Op.LDI, 0x0A,       # 0x00: AC = 10
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 10
        Op.LDI, 0x05,       # 0x04: AC = 5
        Op.CMP, ADDR_VAL,   # 0x06: Compare 5 with 10 (borrow/carry set)
        Op.JC, 0x12,        # 0x08: If carry set, jump to success
        Op.LDI, 0xFF,       # 0x0A: Failure marker
        Op.OUT, 0x00,       # 0x0C: Output failure
        Op.HLT, 0x00,       # 0x0E: Halt (failure path)
        Op.NOP, 0x00,       # 0x10: Padding

        # Jump target (0x12):
        Op.LDI, 0x4C,       # 0x12: Success marker
        Op.OUT, 0x00,       # 0x14: Output 0x4C
        Op.HLT, 0x00,       # 0x16: Halt (success path)
    ]
    expected = 0x4C

    dut._log.info("Testing CMP instruction sets carry on borrow")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"CMP carry test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


# ==============================================================================
# Y REGISTER EXTENSION TESTS
# ==============================================================================

@cocotb.test()
async def test_tay_basic(dut):
    """Test TAY instruction: transfer AC to Y register"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_DATA = 0x80

    # Test: Load value into AC, transfer to Y, transfer back using TYA, output
    program = [
        Op.LDI, 0x42,       # 0x00: AC = 0x42
        Op.TAY,             # 0x02: Y = AC (0x42)
        Op.LDI, 0x00,       # 0x03: AC = 0 (clear AC)
        Op.TYA,             # 0x05: AC = Y (should be 0x42)
        Op.OUT, 0x00,       # 0x06: Output AC
        Op.HLT, 0x00,       # 0x08: Halt
    ]
    expected = 0x42

    dut._log.info("Testing TAY instruction: transfer AC to Y")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"TAY test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_tya_sets_flags(dut):
    """Test TYA instruction: sets N and Z flags"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: Load negative value into Y via TAY, clear AC, TYA should set N flag
    # Then JN should take the jump
    program = [
        Op.LDI, 0x80,       # 0x00: AC = 0x80 (negative, bit 7 set)
        Op.TAY,             # 0x02: Y = 0x80
        Op.LDI, 0x00,       # 0x03: AC = 0
        Op.TYA,             # 0x05: AC = Y = 0x80 (should set N flag)
        Op.JN, 0x0D,        # 0x06: If N set, jump to success
        Op.LDI, 0xFF,       # 0x08: Failure marker
        Op.OUT, 0x00,       # 0x0A: Output failure
        Op.HLT, 0x00,       # 0x0C: Halt

        # Jump target (0x0D):
        Op.LDI, 0x50,       # 0x0D: Success marker
        Op.OUT, 0x00,       # 0x0F: Output 0x50
        Op.HLT, 0x00,       # 0x11: Halt
    ]
    expected = 0x50

    dut._log.info("Testing TYA instruction: sets N flag")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"TYA N flag test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_iny_basic(dut):
    """Test INY instruction: increment Y register"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: Load 0x10 into Y, increment it 3 times, transfer to AC, output
    program = [
        Op.LDI, 0x10,       # 0x00: AC = 0x10
        Op.TAY,             # 0x02: Y = 0x10
        Op.INY,             # 0x03: Y = 0x11
        Op.INY,             # 0x04: Y = 0x12
        Op.INY,             # 0x05: Y = 0x13
        Op.TYA,             # 0x06: AC = Y = 0x13
        Op.OUT, 0x00,       # 0x07: Output AC
        Op.HLT, 0x00,       # 0x09: Halt
    ]
    expected = 0x13

    dut._log.info("Testing INY instruction: increment Y")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"INY test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_ldyi_basic(dut):
    """Test LDYI instruction: load immediate into Y"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Test: Load immediate value into Y, transfer to AC, output
    program = [
        Op.LDYI, 0x55,      # 0x00: Y = 0x55
        Op.TYA,             # 0x02: AC = Y = 0x55
        Op.OUT, 0x00,       # 0x03: Output AC
        Op.HLT, 0x00,       # 0x05: Halt
    ]
    expected = 0x55

    dut._log.info("Testing LDYI instruction: load immediate into Y")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"LDYI test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_ldy_basic(dut):
    """Test LDY instruction: load Y from memory"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_DATA = 0x80

    # Test: Store value in memory, load into Y, transfer to AC, output
    program = [
        Op.LDI, 0x77,       # 0x00: AC = 0x77
        Op.STA, ADDR_DATA,  # 0x02: MEM[0x80] = 0x77
        Op.LDY, ADDR_DATA,  # 0x04: Y = MEM[0x80] = 0x77
        Op.LDI, 0x00,       # 0x06: AC = 0 (clear)
        Op.TYA,             # 0x08: AC = Y = 0x77
        Op.OUT, 0x00,       # 0x09: Output AC
        Op.HLT, 0x00,       # 0x0B: Halt
    ]
    expected = 0x77

    dut._log.info("Testing LDY instruction: load Y from memory")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"LDY test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_sty_basic(dut):
    """Test STY instruction: store Y to memory"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_DATA = 0x80

    # Test: Load immediate into Y, store Y to memory, load from memory to AC, output
    program = [
        Op.LDYI, 0x99,      # 0x00: Y = 0x99
        Op.STY, ADDR_DATA,  # 0x02: MEM[0x80] = Y = 0x99
        Op.LDA, ADDR_DATA,  # 0x04: AC = MEM[0x80] = 0x99
        Op.OUT, 0x00,       # 0x06: Output AC
        Op.HLT, 0x00,       # 0x08: Halt
    ]
    expected = 0x99

    dut._log.info("Testing STY instruction: store Y to memory")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"STY test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_lda_indexed_y(dut):
    """Test LDA addr,Y instruction: indexed load using Y register"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    BASE_ADDR = 0x80

    # Test: Store values in array, set Y as index, load using indexed addressing
    program = [
        # Store values: MEM[0x80]=0x10, MEM[0x81]=0x20, MEM[0x82]=0x30, MEM[0x83]=0x40
        Op.LDI, 0x10,
        Op.STA, 0x80,
        Op.LDI, 0x20,
        Op.STA, 0x81,
        Op.LDI, 0x30,
        Op.STA, 0x82,
        Op.LDI, 0x40,
        Op.STA, 0x83,

        # Set Y = 2, load MEM[0x80 + 2] = MEM[0x82] = 0x30
        Op.LDYI, 0x02,      # 0x10: Y = 2
        Op.LDA_Y, BASE_ADDR,# 0x12: AC = MEM[0x80 + Y] = MEM[0x82] = 0x30
        Op.OUT, 0x00,       # 0x14: Output AC
        Op.HLT, 0x00,       # 0x16: Halt
    ]
    expected = 0x30

    dut._log.info("Testing LDA addr,Y instruction: indexed load")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"LDA indexed Y test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_sta_indexed_y(dut):
    """Test STA addr,Y instruction: indexed store using Y register"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    BASE_ADDR = 0x80

    # Test: Set Y as index, store value using indexed addressing, read back
    program = [
        Op.LDYI, 0x03,      # 0x00: Y = 3
        Op.LDI, 0xAB,       # 0x02: AC = 0xAB
        Op.STA_Y, BASE_ADDR,# 0x04: MEM[0x80 + Y] = MEM[0x83] = 0xAB
        Op.LDA, 0x83,       # 0x06: AC = MEM[0x83] = 0xAB
        Op.OUT, 0x00,       # 0x08: Output AC
        Op.HLT, 0x00,       # 0x0A: Halt
    ]
    expected = 0xAB

    dut._log.info("Testing STA addr,Y instruction: indexed store")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=2000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"STA indexed Y test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_array_copy_xy(dut):
    """Test using both X and Y for array copy operation"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    SRC_ADDR = 0x80
    DST_ADDR = 0x90

    # Test: Copy array using X as source index, Y as destination index
    # Copy MEM[0x80..0x82] to MEM[0x90..0x92]
    program = [
        # Initialize source array
        Op.LDI, 0xAA,       # 0x00
        Op.STA, 0x80,       # 0x02
        Op.LDI, 0xBB,       # 0x04
        Op.STA, 0x81,       # 0x06
        Op.LDI, 0xCC,       # 0x08
        Op.STA, 0x82,       # 0x0A

        # Initialize indices: X=0 (src), Y=0 (dst)
        Op.LDXI, 0x00,      # 0x0C: X = 0
        Op.LDYI, 0x00,      # 0x0E: Y = 0

        # Copy loop: copy 3 elements
        # Element 0
        Op.LDA_X, SRC_ADDR, # 0x10: AC = MEM[0x80 + X]
        Op.STA_Y, DST_ADDR, # 0x12: MEM[0x90 + Y] = AC
        Op.INX,             # 0x14: X++
        Op.INY,             # 0x15: Y++

        # Element 1
        Op.LDA_X, SRC_ADDR, # 0x16: AC = MEM[0x80 + X]
        Op.STA_Y, DST_ADDR, # 0x18: MEM[0x90 + Y] = AC
        Op.INX,             # 0x1A: X++
        Op.INY,             # 0x1B: Y++

        # Element 2
        Op.LDA_X, SRC_ADDR, # 0x1C: AC = MEM[0x80 + X]
        Op.STA_Y, DST_ADDR, # 0x1E: MEM[0x90 + Y] = AC

        # Verify: read from destination (index 1 should be 0xBB)
        Op.LDA, 0x91,       # 0x20: AC = MEM[0x91] = 0xBB
        Op.OUT, 0x00,       # 0x22: Output AC
        Op.HLT, 0x00,       # 0x24: Halt
    ]
    expected = 0xBB

    dut._log.info("Testing array copy with X and Y registers")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=3000)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"Array copy XY test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


# ============================================================================
# MUL INSTRUCTION TESTS
# ============================================================================

@cocotb.test()
async def test_mul_basic(dut):
    """Test MUL instruction: 5 * 3 = 15 (fits in low byte)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Program: Load 5 into AC, 3 into X, multiply, output result
    program = [
        Op.LDI, 0x05,    # AC = 5
        Op.TAX,          # X = AC (X = 5)
        Op.LDI, 0x03,    # AC = 3
        Op.MUL,          # AC * X -> Y:AC (3 * 5 = 15, Y=0, AC=15)
        Op.OUT, 0x00,    # Output AC (low byte)
        Op.HLT
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 15  # 5 * 3 = 15
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"MUL basic test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_mul_zero(dut):
    """Test MUL instruction: anything * 0 = 0"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Program: Load 42 into AC, 0 into X, multiply
    program = [
        Op.LDI, 0x2A,    # AC = 42
        Op.LDXI, 0x00,   # X = 0
        Op.MUL,          # AC * X -> Y:AC (42 * 0 = 0)
        Op.OUT, 0x00,    # Output AC (should be 0)
        Op.HLT
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"MUL zero test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_mul_one(dut):
    """Test MUL instruction: anything * 1 = same value"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Program: Load 42 into AC, 1 into X, multiply
    program = [
        Op.LDI, 0x2A,    # AC = 42
        Op.LDXI, 0x01,   # X = 1
        Op.MUL,          # AC * X -> Y:AC (42 * 1 = 42)
        Op.OUT, 0x00,    # Output AC (should be 42)
        Op.HLT
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 42
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"I/O output received: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"MUL one test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_mul_16bit_result(dut):
    """Test MUL instruction with 16-bit result: 16 * 16 = 256 (0x0100)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Program: 16 * 16 = 256 = 0x0100 (Y=1, AC=0)
    program = [
        Op.LDI, 0x10,    # AC = 16
        Op.TAX,          # X = 16
        Op.MUL,          # 16 * 16 = 256 -> Y=1, AC=0
        Op.OUT, 0x00,    # Output AC (low byte = 0)
        Op.TYA,          # AC = Y (get high byte)
        Op.OUT, 0x00,    # Output Y value (high byte = 1)
        Op.HLT
    ]

    await tb.load_program(program)
    await tb.reset()

    # First output should be low byte (0)
    result_low = await tb.wait_for_io_write(max_cycles=500)
    dut._log.info(f"Low byte: 0x{result_low:02X} (expected: 0x00)")
    assert result_low == 0x00, f"MUL 16-bit low byte failed. Expected 0x00, got 0x{result_low:02X}"

    # Second output should be high byte (1)
    result_high = await tb.wait_for_io_write(max_cycles=500)
    dut._log.info(f"High byte: 0x{result_high:02X} (expected: 0x01)")
    assert result_high == 0x01, f"MUL 16-bit high byte failed. Expected 0x01, got 0x{result_high:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_mul_large_result(dut):
    """Test MUL instruction: 200 * 200 = 40000 (0x9C40)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # 200 * 200 = 40000 = 0x9C40 (Y=0x9C, AC=0x40)
    program = [
        Op.LDI, 0xC8,    # AC = 200 (0xC8)
        Op.TAX,          # X = 200
        Op.MUL,          # 200 * 200 = 40000 -> Y=0x9C, AC=0x40
        Op.OUT, 0x00,    # Output AC (low byte = 0x40)
        Op.TYA,          # AC = Y
        Op.OUT, 0x00,    # Output high byte (0x9C)
        Op.HLT
    ]

    await tb.load_program(program)
    await tb.reset()

    # Low byte = 0x40
    result_low = await tb.wait_for_io_write(max_cycles=500)
    dut._log.info(f"Low byte: 0x{result_low:02X} (expected: 0x40)")
    assert result_low == 0x40, f"MUL large low byte failed. Expected 0x40, got 0x{result_low:02X}"

    # High byte = 0x9C
    result_high = await tb.wait_for_io_write(max_cycles=500)
    dut._log.info(f"High byte: 0x{result_high:02X} (expected: 0x9C)")
    assert result_high == 0x9C, f"MUL large high byte failed. Expected 0x9C, got 0x{result_high:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_mul_max_values(dut):
    """Test MUL instruction: 255 * 255 = 65025 (0xFE01)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # 255 * 255 = 65025 = 0xFE01 (Y=0xFE, AC=0x01)
    program = [
        Op.LDI, 0xFF,    # AC = 255
        Op.TAX,          # X = 255
        Op.MUL,          # 255 * 255 = 65025 -> Y=0xFE, AC=0x01
        Op.OUT, 0x00,    # Output AC (low byte = 0x01)
        Op.TYA,          # AC = Y
        Op.OUT, 0x00,    # Output high byte (0xFE)
        Op.HLT
    ]

    await tb.load_program(program)
    await tb.reset()

    # Low byte = 0x01
    result_low = await tb.wait_for_io_write(max_cycles=500)
    dut._log.info(f"Low byte: 0x{result_low:02X} (expected: 0x01)")
    assert result_low == 0x01, f"MUL max low byte failed. Expected 0x01, got 0x{result_low:02X}"

    # High byte = 0xFE
    result_high = await tb.wait_for_io_write(max_cycles=500)
    dut._log.info(f"High byte: 0x{result_high:02X} (expected: 0xFE)")
    assert result_high == 0xFE, f"MUL max high byte failed. Expected 0xFE, got 0x{result_high:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_mul_sets_carry_on_overflow(dut):
    """Test MUL sets carry flag when result overflows 8 bits"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # 16 * 16 = 256 > 255, should set carry flag
    # We use JC to detect if carry is set
    # Address layout:
    # 0x00: LDI, 0x01: 0x10, 0x02: TAX, 0x03: MUL
    # 0x04: JC, 0x05: target, 0x06: LDI, 0x07: 0x00
    # 0x08: OUT, 0x09: 0x00, 0x0A: HLT
    # 0x0B: LDI (carry set path), 0x0C: 0x01, 0x0D: OUT, 0x0E: 0x00, 0x0F: HLT
    program = [
        Op.LDI, 0x10,    # 0x00: AC = 16
        Op.TAX,          # 0x02: X = 16
        Op.MUL,          # 0x03: 16 * 16 = 256 -> sets carry (overflow)
        Op.JC, 0x0B,     # 0x04: Jump to 0x0B if carry set
        Op.LDI, 0x00,    # 0x06: AC = 0 (carry not set - shouldn't reach here)
        Op.OUT, 0x00,    # 0x08
        Op.HLT,          # 0x0A
        Op.LDI, 0x01,    # 0x0B: AC = 1 (carry was set)
        Op.OUT, 0x00,    # 0x0D
        Op.HLT           # 0x0F
    ]

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=500)
    dut._log.info(f"Carry flag test: 0x{result:02X} (expected: 0x01 if carry set)")
    assert result == 0x01, f"MUL should set carry on overflow. Expected 0x01, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_mul_no_carry_small_result(dut):
    """Test MUL clears carry flag when result fits in 8 bits"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # 3 * 5 = 15 <= 255, should not set carry flag
    # Address layout:
    # 0x00: LDI, 0x01: 0x03, 0x02: TAX, 0x03: LDI, 0x04: 0x05
    # 0x05: MUL, 0x06: JNC, 0x07: target, 0x08: LDI, 0x09: 0x00
    # 0x0A: OUT, 0x0B: 0x00, 0x0C: HLT
    # 0x0D: LDI (no carry path), 0x0E: 0x01, 0x0F: OUT, 0x10: 0x00, 0x11: HLT
    program = [
        Op.LDI, 0x03,    # 0x00: AC = 3
        Op.TAX,          # 0x02: X = 3
        Op.LDI, 0x05,    # 0x03: AC = 5
        Op.MUL,          # 0x05: 5 * 3 = 15 -> no carry (fits in 8 bits)
        Op.JNC, 0x0D,    # 0x06: Jump to 0x0D if no carry
        Op.LDI, 0x00,    # 0x08: AC = 0 (carry set - shouldn't reach here)
        Op.OUT, 0x00,    # 0x0A
        Op.HLT,          # 0x0C
        Op.LDI, 0x01,    # 0x0D: AC = 1 (no carry)
        Op.OUT, 0x00,    # 0x0F
        Op.HLT           # 0x11
    ]

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=500)
    dut._log.info(f"No carry flag test: 0x{result:02X} (expected: 0x01 if no carry)")
    assert result == 0x01, f"MUL should not set carry for small result. Expected 0x01, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_mul_factorial_5(dut):
    """Test MUL by computing 5! = 120 using repeated multiplication"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Compute 5! = 5 * 4 * 3 * 2 * 1 = 120
    # We'll do: 1*2=2, 2*3=6, 6*4=24, 24*5=120
    program = [
        Op.LDI, 0x01,    # AC = 1
        Op.LDXI, 0x02,   # X = 2
        Op.MUL,          # 1 * 2 = 2
        Op.LDXI, 0x03,   # X = 3
        Op.MUL,          # 2 * 3 = 6
        Op.LDXI, 0x04,   # X = 4
        Op.MUL,          # 6 * 4 = 24
        Op.LDXI, 0x05,   # X = 5
        Op.MUL,          # 24 * 5 = 120
        Op.OUT, 0x00,    # Output result
        Op.HLT
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 120  # 5! = 120
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"5! = {result} (expected: {expected})")
    assert result == expected, f"Factorial test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_mul_power_of_2(dut):
    """Test MUL for computing powers of 2: 2^8 = 256"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Compute 2^8 = 256 by repeated multiplication by 2
    # 2*2=4, 4*2=8, 8*2=16, 16*2=32, 32*2=64, 64*2=128, 128*2=256
    program = [
        Op.LDI, 0x02,    # AC = 2
        Op.LDXI, 0x02,   # X = 2
        Op.MUL,          # 2*2=4
        Op.MUL,          # 4*2=8
        Op.MUL,          # 8*2=16
        Op.MUL,          # 16*2=32
        Op.MUL,          # 32*2=64
        Op.MUL,          # 64*2=128
        Op.MUL,          # 128*2=256 -> Y=1, AC=0
        Op.TYA,          # Get high byte
        Op.OUT, 0x00,    # Output high byte (should be 1)
        Op.HLT
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 1  # High byte of 256 (0x0100) is 0x01
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"2^8 high byte = 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"Power of 2 test failed. Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


# =============================================================================
# DIV AND MOD INSTRUCTION TESTS
# =============================================================================

@cocotb.test()
async def test_div_basic(dut):
    """Test DIV instruction: 15 / 3 = 5 (quotient), remainder = 0"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Program: Load 15 into AC, 3 into X, divide, output quotient
    program = [
        Op.LDI, 15,      # AC = 15
        Op.LDXI, 3,      # X = 3
        Op.DIV,          # AC / X -> AC (quotient=5), Y (remainder=0)
        Op.OUT, 0x00,    # Output AC (quotient)
        Op.HLT
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 5  # 15 / 3 = 5
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"DIV basic: 15 / 3 = {result} (expected: {expected})")
    assert result == expected, f"DIV basic test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_div_with_remainder(dut):
    """Test DIV instruction: 17 / 5 = 3 (quotient), remainder = 2"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Program: 17 / 5 = 3 remainder 2
    program = [
        Op.LDI, 17,      # AC = 17
        Op.LDXI, 5,      # X = 5
        Op.DIV,          # AC / X -> AC (quotient=3), Y (remainder=2)
        Op.OUT, 0x00,    # Output AC (quotient)
        Op.TYA,          # AC = Y (get remainder)
        Op.OUT, 0x00,    # Output remainder
        Op.HLT
    ]

    await tb.load_program(program)
    await tb.reset()

    # First output: quotient = 3
    quotient = await tb.wait_for_io_write(max_cycles=500)
    dut._log.info(f"DIV quotient: 17 / 5 = {quotient} (expected: 3)")
    assert quotient == 3, f"DIV quotient failed. Expected 3, got {quotient}"

    # Second output: remainder = 2
    remainder = await tb.wait_for_io_write(max_cycles=500)
    dut._log.info(f"DIV remainder: 17 % 5 = {remainder} (expected: 2)")
    assert remainder == 2, f"DIV remainder failed. Expected 2, got {remainder}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_div_by_one(dut):
    """Test DIV instruction: anything / 1 = same value"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    program = [
        Op.LDI, 42,      # AC = 42
        Op.LDXI, 1,      # X = 1
        Op.DIV,          # 42 / 1 = 42, remainder 0
        Op.OUT, 0x00,    # Output quotient (42)
        Op.HLT
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 42
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"DIV by 1: 42 / 1 = {result} (expected: {expected})")
    assert result == expected, f"DIV by 1 failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_div_zero_dividend(dut):
    """Test DIV instruction: 0 / anything = 0"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    program = [
        Op.LDI, 0,       # AC = 0
        Op.LDXI, 5,      # X = 5
        Op.DIV,          # 0 / 5 = 0, remainder 0
        Op.OUT, 0x00,    # Output quotient (0)
        Op.HLT
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"DIV zero dividend: 0 / 5 = {result} (expected: {expected})")
    assert result == expected, f"DIV zero dividend failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_div_by_zero_sets_carry(dut):
    """Test DIV instruction: division by zero sets carry flag"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Division by zero should set carry flag
    # Address layout:
    # 0x00-0x01: LDI, 42
    # 0x02: TAX
    # 0x03-0x04: LDI, 0
    # 0x05: TAX
    # 0x06-0x07: LDI, 42
    # 0x08: DIV
    # 0x09-0x0A: JC, 0x10 (jump to success)
    # 0x0B-0x0C: LDI, 0x00 (failure)
    # 0x0D-0x0E: OUT, 0x00
    # 0x0F: HLT
    # 0x10-0x11: LDI, 0x01 (success - carry was set)
    # 0x12-0x13: OUT, 0x00
    # 0x14: HLT
    program = [
        Op.LDI, 42,      # 0x00: AC = 42
        Op.TAX,          # 0x02: X = 42 (we'll clear it next)
        Op.LDI, 0,       # 0x03: AC = 0
        Op.TAX,          # 0x05: X = 0
        Op.LDI, 42,      # 0x06: AC = 42
        Op.DIV,          # 0x08: 42 / 0 -> sets carry
        Op.JC, 0x10,     # 0x09: Jump to 0x10 if carry set
        Op.LDI, 0x00,    # 0x0B: AC = 0 (carry not set - failure)
        Op.OUT, 0x00,    # 0x0D
        Op.HLT,          # 0x0F
        Op.LDI, 0x01,    # 0x10: AC = 1 (carry was set - success)
        Op.OUT, 0x00,    # 0x12
        Op.HLT           # 0x14
    ]

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=500)
    dut._log.info(f"DIV by zero carry test: 0x{result:02X} (expected: 0x01 if carry set)")
    assert result == 0x01, f"DIV by zero should set carry. Expected 0x01, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_div_large_values(dut):
    """Test DIV instruction: 200 / 25 = 8"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    program = [
        Op.LDI, 200,     # AC = 200 (0xC8)
        Op.LDXI, 25,     # X = 25
        Op.DIV,          # 200 / 25 = 8, remainder 0
        Op.OUT, 0x00,    # Output quotient
        Op.HLT
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 8  # 200 / 25 = 8
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"DIV large: 200 / 25 = {result} (expected: {expected})")
    assert result == expected, f"DIV large test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_mod_basic(dut):
    """Test MOD instruction: 17 % 5 = 2 (remainder)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Program: 17 % 5 = 2
    program = [
        Op.LDI, 17,      # AC = 17
        Op.LDXI, 5,      # X = 5
        Op.MOD,          # AC % X -> AC (remainder=2), Y (quotient=3)
        Op.OUT, 0x00,    # Output AC (remainder)
        Op.HLT
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 2  # 17 % 5 = 2
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"MOD basic: 17 % 5 = {result} (expected: {expected})")
    assert result == expected, f"MOD basic test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_mod_no_remainder(dut):
    """Test MOD instruction: 15 % 3 = 0 (exact division)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    program = [
        Op.LDI, 15,      # AC = 15
        Op.LDXI, 3,      # X = 3
        Op.MOD,          # 15 % 3 = 0
        Op.OUT, 0x00,    # Output remainder (0)
        Op.HLT
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0  # 15 % 3 = 0
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"MOD no remainder: 15 % 3 = {result} (expected: {expected})")
    assert result == expected, f"MOD no remainder test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_mod_quotient_in_y(dut):
    """Test MOD instruction: verify quotient is stored in Y"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # 17 % 5 = 2 (remainder), quotient = 3 in Y
    program = [
        Op.LDI, 17,      # AC = 17
        Op.LDXI, 5,      # X = 5
        Op.MOD,          # AC % X -> AC (remainder=2), Y (quotient=3)
        Op.TYA,          # AC = Y (get quotient)
        Op.OUT, 0x00,    # Output quotient
        Op.HLT
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 3  # 17 / 5 = 3
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"MOD quotient in Y: 17 / 5 = {result} (expected: {expected})")
    assert result == expected, f"MOD quotient in Y test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_mod_by_zero_sets_carry(dut):
    """Test MOD instruction: division by zero sets carry flag"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Division by zero should set carry flag
    # Address layout:
    # 0x00-0x01: LDI, 42
    # 0x02-0x03: LDXI, 0
    # 0x04: MOD
    # 0x05-0x06: JC, 0x0C (jump to success)
    # 0x07-0x08: LDI, 0x00 (failure)
    # 0x09-0x0A: OUT, 0x00
    # 0x0B: HLT
    # 0x0C-0x0D: LDI, 0x01 (success)
    # 0x0E-0x0F: OUT, 0x00
    # 0x10: HLT
    program = [
        Op.LDI, 42,      # 0x00: AC = 42
        Op.LDXI, 0,      # 0x02: X = 0 (divisor)
        Op.MOD,          # 0x04: 42 % 0 -> sets carry
        Op.JC, 0x0C,     # 0x05: Jump to 0x0C if carry set
        Op.LDI, 0x00,    # 0x07: Failure marker (carry not set)
        Op.OUT, 0x00,    # 0x09
        Op.HLT,          # 0x0B
        Op.LDI, 0x01,    # 0x0C: Success marker (carry was set)
        Op.OUT, 0x00,    # 0x0E
        Op.HLT           # 0x10
    ]

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write(max_cycles=500)
    dut._log.info(f"MOD by zero carry test: 0x{result:02X} (expected: 0x01 if carry set)")
    assert result == 0x01, f"MOD by zero should set carry. Expected 0x01, got 0x{result:02X}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_mod_larger_divisor(dut):
    """Test MOD instruction: when divisor > dividend, remainder = dividend"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # 5 % 10 = 5 (remainder = dividend when divisor is larger)
    program = [
        Op.LDI, 5,       # AC = 5
        Op.LDXI, 10,     # X = 10
        Op.MOD,          # 5 % 10 = 5
        Op.OUT, 0x00,    # Output remainder
        Op.HLT
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 5  # 5 % 10 = 5
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"MOD larger divisor: 5 % 10 = {result} (expected: {expected})")
    assert result == expected, f"MOD larger divisor test failed. Expected {expected}, got {result}"

    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_div_mod_combined(dut):
    """Test DIV and MOD combined: verify consistency"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # 23 = 7 * 3 + 2, so 23 / 7 = 3 and 23 % 7 = 2
    # Verify: quotient * divisor + remainder = dividend
    program = [
        Op.LDI, 23,      # AC = 23
        Op.LDXI, 7,      # X = 7
        Op.DIV,          # 23 / 7 -> AC (quotient=3), Y (remainder=2)
        Op.OUT, 0x00,    # Output quotient (3)
        Op.TYA,          # AC = Y (remainder)
        Op.OUT, 0x00,    # Output remainder (2)
        Op.HLT
    ]

    await tb.load_program(program)
    await tb.reset()

    quotient = await tb.wait_for_io_write(max_cycles=500)
    remainder = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"DIV/MOD combined: 23 / 7 = {quotient} remainder {remainder}")
    assert quotient == 3, f"Combined quotient failed. Expected 3, got {quotient}"
    assert remainder == 2, f"Combined remainder failed. Expected 2, got {remainder}"

    # Verify: 7 * 3 + 2 = 23
    reconstructed = 7 * quotient + remainder
    assert reconstructed == 23, f"Reconstruction failed: {reconstructed} != 23"

    dut._log.info("Test PASSED!")


# =============================================================================
# Comparison Jump Tests (JLE, JGT, JGE, JBE, JA)
# =============================================================================

# JLE Tests (Jump if Less or Equal - signed: N=1 OR Z=1)

@cocotb.test()
async def test_jle_taken_less(dut):
    """Test JLE: jump when AC < value (N=1 after CMP)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # CMP sets flags based on AC - MEM[addr]
    # If AC (5) < MEM (10), result is negative -> N=1 -> JLE taken
    program = [
        Op.LDI, 10,         # 0x00: AC = 10
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 10
        Op.LDI, 5,          # 0x04: AC = 5
        Op.CMP, ADDR_VAL,   # 0x06: Compare 5 - 10 = -5 (N=1)
        Op.JLE, 0x10,       # 0x08: Jump to success if N=1 or Z=1
        Op.LDI, 0xFF,       # 0x0A: Failure marker
        Op.OUT, 0x00,       # 0x0C: Output failure
        Op.HLT, 0x00,       # 0x0E: Halt (with padding)
        # Jump target (0x10):
        Op.LDI, 0x4C,       # 0x10: Success marker (76)
        Op.OUT, 0x00,       # 0x12: Output success
        Op.HLT, 0x00,       # 0x14: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0x4C
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"JLE (less): result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"JLE taken (less) failed"
    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_jle_taken_equal(dut):
    """Test JLE: jump when AC == value (Z=1 after CMP)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # If AC (10) == MEM (10), result is zero -> Z=1 -> JLE taken
    program = [
        Op.LDI, 10,         # 0x00: AC = 10
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 10
        Op.CMP, ADDR_VAL,   # 0x04: Compare 10 - 10 = 0 (Z=1)
        Op.JLE, 0x0E,       # 0x06: Jump to success if N=1 or Z=1
        Op.LDI, 0xFF,       # 0x08: Failure marker
        Op.OUT, 0x00,       # 0x0A: Output failure
        Op.HLT, 0x00,       # 0x0C: Halt (with padding)
        # Jump target (0x0E):
        Op.LDI, 0x4D,       # 0x0E: Success marker (77)
        Op.OUT, 0x00,       # 0x10: Output success
        Op.HLT, 0x00,       # 0x12: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0x4D
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"JLE (equal): result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"JLE taken (equal) failed"
    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_jle_not_taken(dut):
    """Test JLE: don't jump when AC > value (N=0 and Z=0 after CMP)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # If AC (10) > MEM (5), result is positive -> N=0, Z=0 -> JLE NOT taken
    program = [
        Op.LDI, 5,          # 0x00: AC = 5
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 5
        Op.LDI, 10,         # 0x04: AC = 10
        Op.CMP, ADDR_VAL,   # 0x06: Compare 10 - 5 = 5 (N=0, Z=0)
        Op.JLE, 0x10,       # 0x08: Should NOT jump
        Op.LDI, 0x4E,       # 0x0A: Success marker (78)
        Op.OUT, 0x00,       # 0x0C: Output success
        Op.HLT, 0x00,       # 0x0E: Halt (with padding)
        # Jump target (0x10) - should not reach:
        Op.LDI, 0xFF,       # 0x10: Failure marker
        Op.OUT, 0x00,       # 0x12: Output failure
        Op.HLT, 0x00,       # 0x14: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0x4E
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"JLE (not taken): result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"JLE not taken failed"
    dut._log.info("Test PASSED!")


# JGT Tests (Jump if Greater Than - signed: N=0 AND Z=0)

@cocotb.test()
async def test_jgt_taken(dut):
    """Test JGT: jump when AC > value (N=0 and Z=0 after CMP)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # If AC (10) > MEM (5), result is positive -> N=0, Z=0 -> JGT taken
    program = [
        Op.LDI, 5,          # 0x00: AC = 5
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 5
        Op.LDI, 10,         # 0x04: AC = 10
        Op.CMP, ADDR_VAL,   # 0x06: Compare 10 - 5 = 5 (N=0, Z=0)
        Op.JGT, 0x10,       # 0x08: Jump to success if N=0 and Z=0
        Op.LDI, 0xFF,       # 0x0A: Failure marker
        Op.OUT, 0x00,       # 0x0C: Output failure
        Op.HLT, 0x00,       # 0x0E: Halt (with padding)
        # Jump target (0x10):
        Op.LDI, 0x50,       # 0x10: Success marker (80)
        Op.OUT, 0x00,       # 0x12: Output success
        Op.HLT, 0x00,       # 0x14: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0x50
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"JGT (taken): result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"JGT taken failed"
    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_jgt_not_taken_less(dut):
    """Test JGT: don't jump when AC < value (N=1 after CMP)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # If AC (5) < MEM (10), result is negative -> N=1 -> JGT NOT taken
    program = [
        Op.LDI, 10,         # 0x00: AC = 10
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 10
        Op.LDI, 5,          # 0x04: AC = 5
        Op.CMP, ADDR_VAL,   # 0x06: Compare 5 - 10 = -5 (N=1)
        Op.JGT, 0x10,       # 0x08: Should NOT jump
        Op.LDI, 0x51,       # 0x0A: Success marker (81)
        Op.OUT, 0x00,       # 0x0C: Output success
        Op.HLT, 0x00,       # 0x0E: Halt (with padding)
        # Jump target (0x10) - should not reach:
        Op.LDI, 0xFF,       # 0x10: Failure marker
        Op.OUT, 0x00,       # 0x12: Output failure
        Op.HLT, 0x00,       # 0x14: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0x51
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"JGT (not taken, less): result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"JGT not taken (less) failed"
    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_jgt_not_taken_equal(dut):
    """Test JGT: don't jump when AC == value (Z=1 after CMP)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # If AC (10) == MEM (10), result is zero -> Z=1 -> JGT NOT taken
    program = [
        Op.LDI, 10,         # 0x00: AC = 10
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 10
        Op.CMP, ADDR_VAL,   # 0x04: Compare 10 - 10 = 0 (Z=1)
        Op.JGT, 0x0E,       # 0x06: Should NOT jump
        Op.LDI, 0x52,       # 0x08: Success marker (82)
        Op.OUT, 0x00,       # 0x0A: Output success
        Op.HLT, 0x00,       # 0x0C: Halt (with padding)
        # Jump target (0x0E) - should not reach:
        Op.LDI, 0xFF,       # 0x0E: Failure marker
        Op.OUT, 0x00,       # 0x10: Output failure
        Op.HLT, 0x00,       # 0x12: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0x52
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"JGT (not taken, equal): result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"JGT not taken (equal) failed"
    dut._log.info("Test PASSED!")


# JGE Tests (Jump if Greater or Equal - signed: N=0)

@cocotb.test()
async def test_jge_taken_greater(dut):
    """Test JGE: jump when AC > value (N=0 after CMP)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # If AC (10) > MEM (5), result is positive -> N=0 -> JGE taken
    program = [
        Op.LDI, 5,          # 0x00: AC = 5
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 5
        Op.LDI, 10,         # 0x04: AC = 10
        Op.CMP, ADDR_VAL,   # 0x06: Compare 10 - 5 = 5 (N=0)
        Op.JGE, 0x10,       # 0x08: Jump to success if N=0
        Op.LDI, 0xFF,       # 0x0A: Failure marker
        Op.OUT, 0x00,       # 0x0C: Output failure
        Op.HLT, 0x00,       # 0x0E: Halt (with padding)
        # Jump target (0x10):
        Op.LDI, 0x53,       # 0x10: Success marker (83)
        Op.OUT, 0x00,       # 0x12: Output success
        Op.HLT, 0x00,       # 0x14: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0x53
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"JGE (taken, greater): result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"JGE taken (greater) failed"
    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_jge_taken_equal(dut):
    """Test JGE: jump when AC == value (Z=1, N=0 after CMP)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # If AC (10) == MEM (10), result is zero -> Z=1, N=0 -> JGE taken
    program = [
        Op.LDI, 10,         # 0x00: AC = 10
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 10
        Op.CMP, ADDR_VAL,   # 0x04: Compare 10 - 10 = 0 (Z=1, N=0)
        Op.JGE, 0x0E,       # 0x06: Jump to success if N=0
        Op.LDI, 0xFF,       # 0x08: Failure marker
        Op.OUT, 0x00,       # 0x0A: Output failure
        Op.HLT, 0x00,       # 0x0C: Halt (with padding)
        # Jump target (0x0E):
        Op.LDI, 0x54,       # 0x0E: Success marker (84)
        Op.OUT, 0x00,       # 0x10: Output success
        Op.HLT, 0x00,       # 0x12: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0x54
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"JGE (taken, equal): result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"JGE taken (equal) failed"
    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_jge_not_taken(dut):
    """Test JGE: don't jump when AC < value (N=1 after CMP)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # If AC (5) < MEM (10), result is negative -> N=1 -> JGE NOT taken
    program = [
        Op.LDI, 10,         # 0x00: AC = 10
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 10
        Op.LDI, 5,          # 0x04: AC = 5
        Op.CMP, ADDR_VAL,   # 0x06: Compare 5 - 10 = -5 (N=1)
        Op.JGE, 0x10,       # 0x08: Should NOT jump
        Op.LDI, 0x55,       # 0x0A: Success marker (85)
        Op.OUT, 0x00,       # 0x0C: Output success
        Op.HLT, 0x00,       # 0x0E: Halt (with padding)
        # Jump target (0x10) - should not reach:
        Op.LDI, 0xFF,       # 0x10: Failure marker
        Op.OUT, 0x00,       # 0x12: Output failure
        Op.HLT, 0x00,       # 0x14: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0x55
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"JGE (not taken): result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"JGE not taken failed"
    dut._log.info("Test PASSED!")


# JBE Tests (Jump if Below or Equal - unsigned: C=1 OR Z=1)

@cocotb.test()
async def test_jbe_taken_below(dut):
    """Test JBE: jump when AC < value unsigned (C=1 after CMP)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # Unsigned comparison: 5 < 200 -> borrow -> C=1 -> JBE taken
    program = [
        Op.LDI, 200,        # 0x00: AC = 200
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 200
        Op.LDI, 5,          # 0x04: AC = 5
        Op.CMP, ADDR_VAL,   # 0x06: Compare 5 - 200 (borrow, C=1)
        Op.JBE, 0x10,       # 0x08: Jump to success if C=1 or Z=1
        Op.LDI, 0xFF,       # 0x0A: Failure marker
        Op.OUT, 0x00,       # 0x0C: Output failure
        Op.HLT, 0x00,       # 0x0E: Halt (with padding)
        # Jump target (0x10):
        Op.LDI, 0x56,       # 0x10: Success marker (86)
        Op.OUT, 0x00,       # 0x12: Output success
        Op.HLT, 0x00,       # 0x14: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0x56
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"JBE (taken, below): result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"JBE taken (below) failed"
    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_jbe_taken_equal(dut):
    """Test JBE: jump when AC == value (Z=1 after CMP)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # If AC (100) == MEM (100), result is zero -> Z=1 -> JBE taken
    program = [
        Op.LDI, 100,        # 0x00: AC = 100
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 100
        Op.CMP, ADDR_VAL,   # 0x04: Compare 100 - 100 = 0 (Z=1)
        Op.JBE, 0x0E,       # 0x06: Jump to success if C=1 or Z=1
        Op.LDI, 0xFF,       # 0x08: Failure marker
        Op.OUT, 0x00,       # 0x0A: Output failure
        Op.HLT, 0x00,       # 0x0C: Halt (with padding)
        # Jump target (0x0E):
        Op.LDI, 0x57,       # 0x0E: Success marker (87)
        Op.OUT, 0x00,       # 0x10: Output success
        Op.HLT, 0x00,       # 0x12: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0x57
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"JBE (taken, equal): result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"JBE taken (equal) failed"
    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_jbe_not_taken(dut):
    """Test JBE: don't jump when AC > value unsigned (C=0 and Z=0)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # Unsigned comparison: 200 > 5 -> no borrow -> C=0, Z=0 -> JBE NOT taken
    program = [
        Op.LDI, 5,          # 0x00: AC = 5
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 5
        Op.LDI, 200,        # 0x04: AC = 200
        Op.CMP, ADDR_VAL,   # 0x06: Compare 200 - 5 = 195 (no borrow, C=0)
        Op.JBE, 0x10,       # 0x08: Should NOT jump
        Op.LDI, 0x58,       # 0x0A: Success marker (88)
        Op.OUT, 0x00,       # 0x0C: Output success
        Op.HLT, 0x00,       # 0x0E: Halt (with padding)
        # Jump target (0x10) - should not reach:
        Op.LDI, 0xFF,       # 0x10: Failure marker
        Op.OUT, 0x00,       # 0x12: Output failure
        Op.HLT, 0x00,       # 0x14: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0x58
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"JBE (not taken): result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"JBE not taken failed"
    dut._log.info("Test PASSED!")


# JA Tests (Jump if Above - unsigned: C=0 AND Z=0)

@cocotb.test()
async def test_ja_taken(dut):
    """Test JA: jump when AC > value unsigned (C=0 and Z=0)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # Unsigned comparison: 200 > 5 -> no borrow -> C=0, Z=0 -> JA taken
    program = [
        Op.LDI, 5,          # 0x00: AC = 5
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 5
        Op.LDI, 200,        # 0x04: AC = 200
        Op.CMP, ADDR_VAL,   # 0x06: Compare 200 - 5 = 195 (no borrow, C=0)
        Op.JA, 0x10,        # 0x08: Jump to success if C=0 and Z=0
        Op.LDI, 0xFF,       # 0x0A: Failure marker
        Op.OUT, 0x00,       # 0x0C: Output failure
        Op.HLT, 0x00,       # 0x0E: Halt (with padding)
        # Jump target (0x10):
        Op.LDI, 0x59,       # 0x10: Success marker (89)
        Op.OUT, 0x00,       # 0x12: Output success
        Op.HLT, 0x00,       # 0x14: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0x59
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"JA (taken): result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"JA taken failed"
    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_ja_not_taken_below(dut):
    """Test JA: don't jump when AC < value unsigned (C=1)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # Unsigned comparison: 5 < 200 -> borrow -> C=1 -> JA NOT taken
    program = [
        Op.LDI, 200,        # 0x00: AC = 200
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 200
        Op.LDI, 5,          # 0x04: AC = 5
        Op.CMP, ADDR_VAL,   # 0x06: Compare 5 - 200 (borrow, C=1)
        Op.JA, 0x10,        # 0x08: Should NOT jump
        Op.LDI, 0x5A,       # 0x0A: Success marker (90)
        Op.OUT, 0x00,       # 0x0C: Output success
        Op.HLT, 0x00,       # 0x0E: Halt (with padding)
        # Jump target (0x10) - should not reach:
        Op.LDI, 0xFF,       # 0x10: Failure marker
        Op.OUT, 0x00,       # 0x12: Output failure
        Op.HLT, 0x00,       # 0x14: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0x5A
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"JA (not taken, below): result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"JA not taken (below) failed"
    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_ja_not_taken_equal(dut):
    """Test JA: don't jump when AC == value (Z=1)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    ADDR_VAL = 0x80

    # If AC (100) == MEM (100), result is zero -> Z=1 -> JA NOT taken
    program = [
        Op.LDI, 100,        # 0x00: AC = 100
        Op.STA, ADDR_VAL,   # 0x02: MEM[0x80] = 100
        Op.CMP, ADDR_VAL,   # 0x04: Compare 100 - 100 = 0 (Z=1)
        Op.JA, 0x0E,        # 0x06: Should NOT jump
        Op.LDI, 0x5B,       # 0x08: Success marker (91)
        Op.OUT, 0x00,       # 0x0A: Output success
        Op.HLT, 0x00,       # 0x0C: Halt (with padding)
        # Jump target (0x0E) - should not reach:
        Op.LDI, 0xFF,       # 0x0E: Failure marker
        Op.OUT, 0x00,       # 0x10: Output failure
        Op.HLT, 0x00,       # 0x12: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0x5B
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"JA (not taken, equal): result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"JA not taken (equal) failed"
    dut._log.info("Test PASSED!")


# ============================================================================
# Frame Pointer Extension Tests
# ============================================================================

@cocotb.test()
async def test_tsf_basic(dut):
    """Test TSF: FP = SP (transfer SP to FP)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # After reset, SP = 0xFF
    # TSF should copy SP to FP
    # Then PUSH to decrement SP, verify FP unchanged
    program = [
        Op.TSF,         # 0x00: FP = SP (FP should be 0xFF)
        Op.LDI, 0x42,   # 0x01: AC = 0x42
        Op.PUSH,        # 0x03: Push AC (SP becomes 0xFE)
        Op.TFS,         # 0x04: SP = FP (restore SP to 0xFF)
        Op.LDI, 0x00,   # 0x05: Clear AC
        Op.POP,         # 0x07: Pop should get garbage (not 0x42) since SP was restored
        Op.OUT, 0x00,   # 0x08: Output AC
        Op.HLT, 0x00,   # 0x0A: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    # Wait for halt then check FP
    await tb.run_until_halt(max_cycles=200)
    fp_val = int(dut.dbg_fp.value)
    dut._log.info(f"TSF test: FP=0x{fp_val:02X}, expected=0xFF")
    assert fp_val == 0xFF, f"TSF failed: FP should be 0xFF, got 0x{fp_val:02X}"
    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_tfs_basic(dut):
    """Test TFS: SP = FP (transfer FP to SP)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Set FP to known value via TSF, modify SP with PUSH, restore with TFS
    program = [
        Op.TSF,         # 0x00: FP = SP (0xFF)
        Op.LDI, 0x11,   # 0x01: AC = 0x11
        Op.PUSH,        # 0x03: SP = 0xFE
        Op.LDI, 0x22,   # 0x04: AC = 0x22
        Op.PUSH,        # 0x06: SP = 0xFD
        Op.TFS,         # 0x07: SP = FP (restore to 0xFF)
        Op.HLT, 0x00,   # 0x08: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    await tb.run_until_halt(max_cycles=200)
    sp_val = int(dut.dbg_sp.value)
    dut._log.info(f"TFS test: SP=0x{sp_val:02X}, expected=0xFF")
    assert sp_val == 0xFF, f"TFS failed: SP should be 0xFF, got 0x{sp_val:02X}"
    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_push_fp_basic(dut):
    """Test PUSH_FP: MEM[--SP] = FP"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Set FP to known value, push it, pop to AC, verify
    program = [
        Op.TSF,         # 0x00: FP = SP (0xFF)
        Op.LDI, 0x33,   # 0x01: AC = 0x33
        Op.PUSH,        # 0x03: Push 0x33, SP = 0xFE
        Op.LDI, 0x44,   # 0x04: AC = 0x44
        Op.PUSH,        # 0x06: Push 0x44, SP = 0xFD
        Op.TSF,         # 0x07: FP = SP (0xFD) - capture current SP as FP
        Op.PUSH_FP,     # 0x08: Push FP (0xFD), SP = 0xFC
        Op.POP,         # 0x09: Pop to AC (should be 0xFD)
        Op.OUT, 0x00,   # 0x0A: Output AC
        Op.HLT, 0x00,   # 0x0C: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0xFD  # FP value that was pushed
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"PUSH_FP test: result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"PUSH_FP failed"
    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_pop_fp_basic(dut):
    """Test POP_FP: FP = MEM[SP++]"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Push a value, pop it to FP, verify FP
    program = [
        Op.LDI, 0xAB,   # 0x00: AC = 0xAB
        Op.PUSH,        # 0x02: Push 0xAB, SP = 0xFE
        Op.POP_FP,      # 0x03: FP = 0xAB, SP = 0xFF
        Op.HLT, 0x00,   # 0x04: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    await tb.run_until_halt(max_cycles=200)
    fp_val = int(dut.dbg_fp.value)
    sp_val = int(dut.dbg_sp.value)
    dut._log.info(f"POP_FP test: FP=0x{fp_val:02X}, SP=0x{sp_val:02X}")
    assert fp_val == 0xAB, f"POP_FP failed: FP should be 0xAB, got 0x{fp_val:02X}"
    assert sp_val == 0xFF, f"POP_FP failed: SP should be 0xFF, got 0x{sp_val:02X}"
    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_lda_fp_indexed(dut):
    """Test LDA addr,FP: AC = MEM[addr + FP]"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Set up: FP = 0x10, store data at 0x80, read with base 0x70 + FP
    ADDR_DATA = 0x80
    ADDR_BASE = 0x70  # 0x70 + 0x10 = 0x80
    DATA_VALUE = 0x5C

    program = [
        Op.LDI, DATA_VALUE,     # 0x00: AC = 0x5C
        Op.STA, ADDR_DATA,      # 0x02: MEM[0x80] = 0x5C
        Op.LDI, 0x10,           # 0x04: AC = 0x10
        Op.PUSH,                # 0x06: Push 0x10
        Op.POP_FP,              # 0x07: FP = 0x10
        Op.LDI, 0x00,           # 0x08: Clear AC
        Op.LDA_FP, ADDR_BASE,   # 0x0A: AC = MEM[0x70 + 0x10] = MEM[0x80]
        Op.OUT, 0x00,           # 0x0C: Output AC
        Op.HLT, 0x00,           # 0x0E: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = DATA_VALUE
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"LDA addr,FP test: result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"LDA addr,FP failed"
    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_sta_fp_indexed(dut):
    """Test STA addr,FP: MEM[addr + FP] = AC"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Set up: FP = 0x10, store data to base 0x70 + FP = 0x80
    ADDR_BASE = 0x70
    ADDR_TARGET = 0x80  # 0x70 + 0x10
    DATA_VALUE = 0x7E

    program = [
        Op.LDI, 0x10,           # 0x00: AC = 0x10
        Op.PUSH,                # 0x02: Push 0x10
        Op.POP_FP,              # 0x03: FP = 0x10
        Op.LDI, DATA_VALUE,     # 0x04: AC = 0x7E
        Op.STA_FP, ADDR_BASE,   # 0x06: MEM[0x70 + 0x10] = 0x7E
        Op.LDI, 0x00,           # 0x08: Clear AC
        Op.LDA, ADDR_TARGET,    # 0x0A: AC = MEM[0x80]
        Op.OUT, 0x00,           # 0x0C: Output AC
        Op.HLT, 0x00,           # 0x0E: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = DATA_VALUE
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"STA addr,FP test: result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"STA addr,FP failed"
    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_function_prologue_epilogue(dut):
    """Test complete function prologue/epilogue using FP"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Simulate a function call with prologue and epilogue:
    # Prologue:
    #   PUSH_FP      ; save caller's FP
    #   TSF          ; FP = SP (new frame pointer)
    # Epilogue:
    #   TFS          ; SP = FP (deallocate locals)
    #   POP_FP       ; restore caller's FP

    program = [
        # Main code
        Op.LDI, 0xBB,       # 0x00: AC = 0xBB (initial FP marker)
        Op.PUSH,            # 0x02: Push 0xBB
        Op.POP_FP,          # 0x03: FP = 0xBB (simulate caller's FP)
        Op.CALL, 0x10,      # 0x04: Call function at 0x10
        # After return, check FP is restored
        Op.LDI, 0x00,       # 0x06: Clear AC
        Op.PUSH_FP,         # 0x08: Push current FP
        Op.POP,             # 0x09: Pop to AC
        Op.OUT, 0x00,       # 0x0A: Output FP value
        Op.HLT, 0x00,       # 0x0C: Halt

        # Padding
        Op.NOP,             # 0x0E
        Op.NOP,             # 0x0F

        # Function at 0x10
        # Prologue
        Op.PUSH_FP,         # 0x10: Save caller's FP
        Op.TSF,             # 0x11: FP = SP (set up new frame)
        # Function body (allocate local by pushing)
        Op.LDI, 0x77,       # 0x12: Local variable = 0x77
        Op.PUSH,            # 0x14: Allocate local
        # Epilogue
        Op.TFS,             # 0x15: SP = FP (deallocate locals)
        Op.POP_FP,          # 0x16: Restore caller's FP
        Op.RET,             # 0x17: Return
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0xBB  # FP should be restored to caller's value
    result = await tb.wait_for_io_write(max_cycles=1000)

    dut._log.info(f"Function prologue/epilogue test: result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"Function prologue/epilogue failed: FP not restored"
    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_fp_local_variable_access(dut):
    """Test accessing local variables via FP-indexed addressing"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Simulate local variable access:
    # FP points to frame base, locals are at positive offsets from FP
    # Stack grows downward: PUSH decrements SP first, then writes

    program = [
        # Set up frame: SP=0xFF, after pushes SP will decrement
        Op.LDI, 0xAA,       # 0x00: First local = 0xAA
        Op.PUSH,            # 0x02: SP dec to 0xFE, MEM[0xFE] = 0xAA
        Op.LDI, 0xBB,       # 0x03: Second local = 0xBB
        Op.PUSH,            # 0x05: SP dec to 0xFD, MEM[0xFD] = 0xBB
        Op.TSF,             # 0x06: FP = SP = 0xFD

        # Now frame looks like:
        # 0xFF: (unused)
        # 0xFE: 0xAA (first pushed, offset +1 from FP)
        # 0xFD: 0xBB (second pushed, FP points here, offset +0)

        # Access first local (0xAA) using FP + 1
        Op.LDI, 0x00,       # 0x07: Clear AC
        Op.LDA_FP, 0x01,    # 0x09: AC = MEM[0xFD + 0x01] = MEM[0xFE] = 0xAA
        Op.OUT, 0x00,       # 0x0B: Output first local
        Op.HLT, 0x00,       # 0x0D: Halt
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0xAA  # First local variable
    result = await tb.wait_for_io_write(max_cycles=500)

    dut._log.info(f"FP local variable access test: result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"FP local variable access failed"
    dut._log.info("Test PASSED!")


@cocotb.test()
async def test_fp_parameter_access(dut):
    """Test accessing function parameters via FP-indexed addressing"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Simulate parameter passing:
    # Push parameter, call function, access parameter via FP

    program = [
        # Main: push parameter and call
        Op.LDI, 0x55,       # 0x00: Parameter value = 0x55
        Op.PUSH,            # 0x02: Push parameter, SP = 0xFE
        Op.CALL, 0x10,      # 0x03: Call function at 0x10
        Op.HLT, 0x00,       # 0x05: Halt

        # Padding
        Op.NOP, Op.NOP, Op.NOP, Op.NOP, Op.NOP, Op.NOP, Op.NOP, Op.NOP,  # 0x07-0x0E
        Op.NOP,             # 0x0F

        # Function at 0x10
        # After CALL: SP = 0xFD (return address at 0xFD)
        # Parameter at 0xFE, accessible as FP + offset
        Op.PUSH_FP,         # 0x10: Save FP, SP = 0xFC
        Op.TSF,             # 0x11: FP = SP = 0xFC
        # Frame layout:
        # 0xFF: (unused)
        # 0xFE: Parameter (0x55)
        # 0xFD: Return address
        # 0xFC: Old FP (FP points here)
        # Access parameter at FP + 2 = 0xFE
        Op.LDA_FP, 0x02,    # 0x12: AC = MEM[0xFC + 0x02] = MEM[0xFE] = 0x55
        Op.OUT, 0x00,       # 0x14: Output parameter
        Op.TFS,             # 0x16: Restore SP
        Op.POP_FP,          # 0x17: Restore FP
        Op.RET,             # 0x18: Return
    ]

    await tb.load_program(program)
    await tb.reset()

    expected = 0x55  # Parameter value
    result = await tb.wait_for_io_write(max_cycles=1000)

    dut._log.info(f"FP parameter access test: result=0x{result:02X}, expected=0x{expected:02X}")
    assert result == expected, f"FP parameter access failed"
    dut._log.info("Test PASSED!")
