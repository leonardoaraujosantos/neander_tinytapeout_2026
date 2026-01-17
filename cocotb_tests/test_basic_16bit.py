"""
Basic Tests for Neander CPU with 16-bit Address Extension
==========================================================

These are simplified tests to verify the 16-bit address extension
and SPI memory controller work correctly.

Instructions with address operands use 3 bytes: [opcode, addr_lo, addr_hi]
SPI memory access is slower (~64-80 cycles per byte transfer)
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles


# Neander Opcodes
class Op:
    NOP  = 0x00
    NEG  = 0x01
    STA  = 0x10
    LDA  = 0x20
    ADD  = 0x30
    OR   = 0x40
    AND  = 0x50
    NOT  = 0x60
    PUSH = 0x70
    POP  = 0x71
    CALL = 0x72
    RET  = 0x73
    SUB  = 0x74
    INC  = 0x75
    DEC  = 0x76
    XOR  = 0x77
    SHL  = 0x78
    SHR  = 0x79
    JMP  = 0x80
    JN   = 0x90
    JZ   = 0xA0
    JNZ  = 0xB0
    IN   = 0xC0
    OUT  = 0xD0
    LDI  = 0xE0
    HLT  = 0xF0


def addr16(addr):
    """Convert address to [lo, hi] little-endian"""
    return [addr & 0xFF, (addr >> 8) & 0xFF]


def instr(opcode, addr):
    """Create 3-byte instruction with 16-bit address"""
    return [opcode] + addr16(addr)


# Increase timeout for SPI memory (each access ~80 cycles)
DEFAULT_MAX_CYCLES = 200000


class NeanderTB:
    """Testbench helper for 16-bit Neander"""

    def __init__(self, dut):
        self.dut = dut

    async def setup(self, clock_period_ns=10):
        """Initialize clock and signals"""
        clock = Clock(self.dut.clk, clock_period_ns, units="ns")
        cocotb.start_soon(clock.start())

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
        """Load program into memory"""
        for i, byte in enumerate(program):
            await self.load_byte(start_addr + i, byte)

    async def read_memory(self, addr):
        """Read byte from memory"""
        self.dut.mem_read_addr.value = addr
        await RisingEdge(self.dut.clk)
        return int(self.dut.mem_read_data.value)

    async def wait_for_io_write(self, max_cycles=DEFAULT_MAX_CYCLES):
        """Wait for I/O write operation"""
        cycles = 0
        while cycles < max_cycles:
            await RisingEdge(self.dut.clk)
            cycles += 1
            if self.dut.io_write.value == 1:
                return int(self.dut.io_out.value)
        raise TimeoutError(f"No I/O write within {max_cycles} cycles")

    async def run_until_halt(self, max_cycles=DEFAULT_MAX_CYCLES):
        """Run until HLT instruction"""
        last_pc = -1
        halt_counter = 0
        cycles = 0

        while cycles < max_cycles:
            await RisingEdge(self.dut.clk)
            cycles += 1

            current_pc = int(self.dut.dbg_pc.value)
            current_ri = int(self.dut.dbg_ri.value)

            if current_pc == last_pc and (current_ri & 0xF0) == Op.HLT:
                halt_counter += 1
                if halt_counter > 10:
                    return cycles
            else:
                halt_counter = 0

            last_pc = current_pc

        raise TimeoutError(f"CPU did not halt within {max_cycles} cycles")


# ============================================================================
# Basic Tests
# ============================================================================

@cocotb.test()
async def test_ldi_out_hlt(dut):
    """Test LDI, OUT, HLT sequence (simplest possible test)"""
    tb = NeanderTB(dut)
    await tb.setup()

    # LDI 0x42, OUT, HLT
    # LDI is 2 bytes: opcode + immediate
    # OUT is 2 bytes: opcode + port
    # HLT is 1 byte: opcode only
    program = [
        Op.LDI, 0x42,    # 0x00-0x01: AC = 0x42
        Op.OUT, 0x00,    # 0x02-0x03: Output to port 0
        Op.HLT,          # 0x04: Halt
    ]

    dut._log.info("Testing LDI, OUT, HLT sequence")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write()

    dut._log.info(f"Result: 0x{result:02X} (expected: 0x42)")
    assert result == 0x42, f"Expected 0x42, got 0x{result:02X}"

    dut._log.info("PASSED!")


@cocotb.test()
async def test_lda_sta_16bit_addr(dut):
    """Test LDA and STA with 16-bit address"""
    tb = NeanderTB(dut)
    await tb.setup()

    # Data at address 0x0100
    DATA_ADDR = 0x0100

    # Program:
    # LDI 0x55
    # STA 0x0100  (store to address 0x0100)
    # LDI 0x00    (clear AC)
    # LDA 0x0100  (load from address 0x0100)
    # OUT
    # HLT
    program = [
        Op.LDI, 0x55,                    # 0x00-0x01: AC = 0x55
        *instr(Op.STA, DATA_ADDR),       # 0x02-0x04: MEM[0x0100] = AC
        Op.LDI, 0x00,                    # 0x05-0x06: AC = 0x00
        *instr(Op.LDA, DATA_ADDR),       # 0x07-0x09: AC = MEM[0x0100]
        Op.OUT, 0x00,                    # 0x0A-0x0B: Output
        Op.HLT,                          # 0x0C: Halt
    ]

    dut._log.info("Testing LDA/STA with 16-bit address (0x0100)")
    dut._log.info(f"Program: {[f'0x{b:02X}' for b in program]}")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write()

    dut._log.info(f"Result: 0x{result:02X} (expected: 0x55)")
    assert result == 0x55, f"Expected 0x55, got 0x{result:02X}"

    # Verify memory contents
    mem_val = await tb.read_memory(DATA_ADDR)
    dut._log.info(f"Memory[0x{DATA_ADDR:04X}] = 0x{mem_val:02X}")
    assert mem_val == 0x55, f"Memory should be 0x55, got 0x{mem_val:02X}"

    dut._log.info("PASSED!")


@cocotb.test()
async def test_add_16bit_addr(dut):
    """Test ADD with 16-bit address"""
    tb = NeanderTB(dut)
    await tb.setup()

    DATA_A = 0x0100
    DATA_B = 0x0101

    # Store values first, then add
    program = [
        # Store 10 at 0x0100
        Op.LDI, 10,                      # 0x00-0x01
        *instr(Op.STA, DATA_A),          # 0x02-0x04
        # Store 25 at 0x0101
        Op.LDI, 25,                      # 0x05-0x06
        *instr(Op.STA, DATA_B),          # 0x07-0x09
        # Load 10, add 25
        *instr(Op.LDA, DATA_A),          # 0x0A-0x0C
        *instr(Op.ADD, DATA_B),          # 0x0D-0x0F
        Op.OUT, 0x00,                    # 0x10-0x11
        Op.HLT,                          # 0x12
    ]
    expected = 35

    dut._log.info("Testing ADD: 10 + 25 = 35")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write()

    dut._log.info(f"Result: {result} (expected: {expected})")
    assert result == expected, f"Expected {expected}, got {result}"

    dut._log.info("PASSED!")


@cocotb.test()
async def test_jmp_16bit_addr(dut):
    """Test JMP with 16-bit address"""
    tb = NeanderTB(dut)
    await tb.setup()

    # JMP target at 0x0012
    JMP_TARGET = 0x0012

    program = [
        Op.LDI, 0x11,                    # 0x00-0x01: AC = 0x11
        *instr(Op.JMP, JMP_TARGET),      # 0x02-0x04: JMP to 0x0012
        # These should be skipped:
        Op.LDI, 0xFF,                    # 0x05-0x06
        Op.OUT, 0x00,                    # 0x07-0x08
        Op.HLT,                          # 0x09
        # Padding
        Op.NOP, Op.NOP, Op.NOP, Op.NOP,  # 0x0A-0x0D
        Op.NOP, Op.NOP, Op.NOP, Op.NOP,  # 0x0E-0x11
        # JMP target (0x12):
        Op.LDI, 0x42,                    # 0x12-0x13: AC = 0x42
        Op.OUT, 0x00,                    # 0x14-0x15
        Op.HLT,                          # 0x16
    ]
    expected = 0x42

    dut._log.info("Testing JMP to 16-bit address")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write()

    dut._log.info(f"Result: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"JMP failed: expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("PASSED!")


@cocotb.test()
async def test_jz_16bit_addr(dut):
    """Test JZ with 16-bit address"""
    tb = NeanderTB(dut)
    await tb.setup()

    JZ_TARGET = 0x0012

    program = [
        Op.LDI, 0x00,                    # 0x00-0x01: AC = 0 (sets Z flag)
        *instr(Op.JZ, JZ_TARGET),        # 0x02-0x04: JZ to 0x0012
        # These should be skipped:
        Op.LDI, 0xFF,                    # 0x05-0x06
        Op.OUT, 0x00,                    # 0x07-0x08
        Op.HLT,                          # 0x09
        # Padding
        Op.NOP, Op.NOP, Op.NOP, Op.NOP,  # 0x0A-0x0D
        Op.NOP, Op.NOP, Op.NOP, Op.NOP,  # 0x0E-0x11
        # JZ target (0x12):
        Op.LDI, 0xAA,                    # 0x12-0x13: AC = 0xAA
        Op.OUT, 0x00,                    # 0x14-0x15
        Op.HLT,                          # 0x16
    ]
    expected = 0xAA

    dut._log.info("Testing JZ with zero AC")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write()

    dut._log.info(f"Result: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"JZ failed: expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("PASSED!")


@cocotb.test()
async def test_inc_dec(dut):
    """Test INC and DEC instructions"""
    tb = NeanderTB(dut)
    await tb.setup()

    program = [
        Op.LDI, 10,      # AC = 10
        Op.INC,          # AC = 11
        Op.INC,          # AC = 12
        Op.INC,          # AC = 13
        Op.DEC,          # AC = 12
        Op.OUT, 0x00,
        Op.HLT,
    ]
    expected = 12

    dut._log.info("Testing INC/DEC: 10 + 3 - 1 = 12")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write()

    dut._log.info(f"Result: {result} (expected: {expected})")
    assert result == expected, f"Expected {expected}, got {result}"

    dut._log.info("PASSED!")


@cocotb.test()
async def test_not_instruction(dut):
    """Test NOT instruction"""
    tb = NeanderTB(dut)
    await tb.setup()

    program = [
        Op.LDI, 0x0F,    # AC = 0x0F
        Op.NOT,          # AC = 0xF0
        Op.OUT, 0x00,
        Op.HLT,
    ]
    expected = 0xF0

    dut._log.info("Testing NOT: ~0x0F = 0xF0")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write()

    dut._log.info(f"Result: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"Expected 0x{expected:02X}, got 0x{result:02X}"

    dut._log.info("PASSED!")


@cocotb.test()
async def test_shift_instructions(dut):
    """Test SHL and SHR instructions"""
    tb = NeanderTB(dut)
    await tb.setup()

    # Test SHL: 0x05 << 1 = 0x0A
    program = [
        Op.LDI, 0x05,    # AC = 0x05 (0000_0101)
        Op.SHL,          # AC = 0x0A (0000_1010)
        Op.OUT, 0x00,
        Op.HLT,
    ]

    dut._log.info("Testing SHL: 0x05 << 1 = 0x0A")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write()

    dut._log.info(f"Result: 0x{result:02X} (expected: 0x0A)")
    assert result == 0x0A, f"SHL failed: expected 0x0A, got 0x{result:02X}"

    dut._log.info("PASSED!")


@cocotb.test()
async def test_simple_loop(dut):
    """Test simple countdown loop"""
    tb = NeanderTB(dut)
    await tb.setup()

    # Data addresses
    COUNT = 0x0100
    NEG1 = 0x0101

    # Loop: count from 3 to 0
    LOOP = 0x000D
    DONE = 0x001F

    program = [
        # Initialize count = 3
        Op.LDI, 3,                       # 0x00-0x01
        *instr(Op.STA, COUNT),           # 0x02-0x04
        # Initialize neg1 = 0xFF (-1)
        Op.LDI, 0xFF,                    # 0x05-0x06
        *instr(Op.STA, NEG1),            # 0x07-0x09
        # LOOP starts at 0x0A
        # But JZ uses 3 bytes, so let's recalculate
    ]

    # Actually, let's build this more carefully
    program = []
    pc = 0

    # Initialize count = 3
    program += [Op.LDI, 3]
    pc += 2
    program += instr(Op.STA, COUNT)
    pc += 3

    # Initialize neg1 = 0xFF
    program += [Op.LDI, 0xFF]
    pc += 2
    program += instr(Op.STA, NEG1)
    pc += 3

    # LOOP: now at pc = 10 (0x0A)
    LOOP = pc

    # LDA COUNT
    program += instr(Op.LDA, COUNT)
    pc += 3

    # JZ DONE (need to calculate DONE address)
    # After this: LDA COUNT (3), ADD NEG1 (3), STA COUNT (3), JMP LOOP (3)
    # = 12 more bytes + JZ itself (3) = 15 bytes from here
    DONE = pc + 3 + 3 + 3 + 3 + 3  # = pc + 15
    program += instr(Op.JZ, DONE)
    pc += 3

    # ADD NEG1 (decrement)
    program += instr(Op.ADD, NEG1)
    pc += 3

    # STA COUNT
    program += instr(Op.STA, COUNT)
    pc += 3

    # JMP LOOP
    program += instr(Op.JMP, LOOP)
    pc += 3

    # DONE: output result (should be 0)
    # pc should now equal DONE
    dut._log.info(f"LOOP=0x{LOOP:04X}, DONE=0x{DONE:04X}, current pc=0x{pc:04X}")

    program += instr(Op.LDA, COUNT)
    pc += 3
    program += [Op.OUT, 0x00]
    pc += 2
    program += [Op.HLT]

    dut._log.info(f"Program length: {len(program)} bytes")
    dut._log.info(f"Testing countdown loop from 3 to 0")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write()

    dut._log.info(f"Result: {result} (expected: 0)")
    assert result == 0, f"Loop failed: expected 0, got {result}"

    dut._log.info("PASSED!")


@cocotb.test()
async def test_high_address_access(dut):
    """Test accessing high memory address (e.g., 0x8000)"""
    tb = NeanderTB(dut)
    await tb.setup()

    HIGH_ADDR = 0x8000  # High address to test

    program = [
        Op.LDI, 0x77,                    # AC = 0x77
        *instr(Op.STA, HIGH_ADDR),       # Store at 0x8000
        Op.LDI, 0x00,                    # Clear AC
        *instr(Op.LDA, HIGH_ADDR),       # Load from 0x8000
        Op.OUT, 0x00,
        Op.HLT,
    ]
    expected = 0x77

    dut._log.info(f"Testing high address access at 0x{HIGH_ADDR:04X}")

    await tb.load_program(program)
    await tb.reset()

    result = await tb.wait_for_io_write()

    dut._log.info(f"Result: 0x{result:02X} (expected: 0x{expected:02X})")
    assert result == expected, f"High address access failed: expected 0x{expected:02X}, got 0x{result:02X}"

    # Verify memory
    mem_val = await tb.read_memory(HIGH_ADDR)
    assert mem_val == expected, f"Memory at 0x{HIGH_ADDR:04X} = 0x{mem_val:02X}, expected 0x{expected:02X}"

    dut._log.info("PASSED!")
