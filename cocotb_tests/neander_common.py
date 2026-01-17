"""
Neander CPU Common Test Infrastructure
======================================

Shared code for Neander CPU tests including:
- Opcodes definitions
- Program converter for 16-bit addressing
- NeanderTestbench class

This module contains NO tests, only shared infrastructure.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles


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
    # B Register Extension (opcode 0x1 family)
    TAB  = 0x1C  # TAB: B = AC
    TBA  = 0x1D  # TBA: AC = B
    LDB  = 0x1E  # LDB addr: B = MEM[addr]
    STB  = 0x1F  # STB addr: MEM[addr] = B
    LDA  = 0x20
    ADD  = 0x30
    ADC  = 0x31  # ADC addr: AC = AC + MEM[addr] + Carry (add with carry)
    ADDX = 0x32  # ADDX: AC = AC + X
    SUBX = 0x33  # SUBX: AC = AC - X
    ADDY = 0x34  # ADDY: AC = AC + Y
    SUBY = 0x35  # SUBY: AC = AC - Y
    INB  = 0x36  # INB: B = B + 1
    DEB  = 0x37  # DEB: B = B - 1
    SWPB = 0x38  # SWPB: Swap AC <-> B
    ADDB = 0x39  # ADDB: AC = AC + B
    SUBB = 0x3A  # SUBB: AC = AC - B
    OR   = 0x40
    AND  = 0x50
    SBC  = 0x51  # SBC addr: AC = AC - MEM[addr] - Carry (subtract with borrow)
    NOT  = 0x60
    ASR  = 0x61  # ASR: AC = AC >> 1 (arithmetic shift right, preserves sign)
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
    PUSH_ADDR = 0x89  # PUSH_ADDR addr: MEM[--SP] = MEM[addr]
    POP_ADDR  = 0x8A  # POP_ADDR addr: MEM[addr] = MEM[SP++]
    JN   = 0x90
    JZ   = 0xA0
    JNZ  = 0xB0
    IN   = 0xC0
    OUT  = 0xD0
    LDI  = 0xE0
    LDBI = 0xE4  # LDBI imm: B = imm (16-bit)
    HLT  = 0xF0


# ============================================================================
# Program Converter: 8-bit address format -> 16-bit address format
# ============================================================================

# Opcodes that take a memory address (need expansion from 2 bytes to 3 bytes)
MEMORY_ADDRESS_OPCODES = {
    0x10, 0x11, 0x12, 0x14,  # STA family
    0x1E, 0x1F,              # LDB, STB (B register)
    0x20, 0x21, 0x22, 0x24,  # LDA family
    0x30, 0x31,              # ADD, ADC
    0x40,                    # OR
    0x50, 0x51,              # AND, SBC
    0x74, 0x77,              # SUB, XOR
    0x02,                    # CMP
    0x7A, 0x7B,              # LDX, STX
    0x07, 0x08,              # LDY, STY
    0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,  # JMP, JC, JNC, JLE, JGT, JGE, JBE, JA
    0x89, 0x8A,              # PUSH_ADDR, POP_ADDR
    0x90, 0xA0, 0xB0,        # JN, JZ, JNZ
    0x72,                    # CALL
}

# Jump/branch opcodes (reference code addresses that need remapping)
JUMP_OPCODES = {
    0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    0x90, 0xA0, 0xB0, 0x72,
}

# Opcodes that take a 16-bit immediate value (expand from 2 to 3 bytes)
IMMEDIATE_16BIT_OPCODES = {
    0xE0,  # LDI  (16-bit immediate)
    0x7C,  # LDXI (16-bit immediate)
    0x06,  # LDYI (16-bit immediate)
    0xE4,  # LDBI (16-bit immediate for B register)
}

# Opcodes that take an 8-bit immediate/port (keep as 2 bytes)
IMMEDIATE_8BIT_OPCODES = {
    0xD0,  # OUT (port number)
    0xC0,  # IN  (port number)
}

# Data area threshold
DATA_AREA_START = 0x40


def convert_program_to_16bit(program, data_area_start=DATA_AREA_START):
    """
    Convert an old 8-bit address format program to 16-bit address format.

    - Memory address opcodes: expand from 2 bytes to 3 bytes (addr8 -> addr_lo, addr_hi)
    - 16-bit immediate opcodes (LDI, LDXI, LDYI): expand from 2 bytes to 3 bytes (imm8 -> imm_lo, imm_hi)
    - 8-bit immediate opcodes (IN, OUT): keep as 2 bytes
    - Single byte opcodes: keep as 1 byte
    """
    # First pass: build address mapping
    addr_map = {}
    old_pos = 0
    new_pos = 0

    while old_pos < len(program):
        addr_map[old_pos] = new_pos
        opcode = program[old_pos]

        if opcode in MEMORY_ADDRESS_OPCODES:
            old_pos += 2
            new_pos += 3
        elif opcode in IMMEDIATE_16BIT_OPCODES:
            old_pos += 2
            new_pos += 3  # Expand 8-bit immediate to 16-bit
        elif opcode in IMMEDIATE_8BIT_OPCODES:
            old_pos += 2
            new_pos += 2  # Keep as 2 bytes
        else:
            old_pos += 1
            new_pos += 1

    old_program_end = old_pos

    # Second pass: convert program
    result = []
    i = 0
    while i < len(program):
        opcode = program[i]

        if opcode in MEMORY_ADDRESS_OPCODES:
            if i + 1 < len(program):
                addr = program[i + 1]
                result.append(opcode)

                if addr in addr_map:
                    new_addr = addr_map[addr]
                elif addr < old_program_end:
                    closest_start = max(k for k in addr_map.keys() if k <= addr)
                    offset = addr - closest_start
                    new_addr = addr_map[closest_start] + offset
                else:
                    # Address is in data area: remap to space out for 16-bit values
                    # Each old data slot (1 byte) maps to 2 bytes in 16-bit mode
                    new_addr = data_area_start + (addr - data_area_start) * 2

                result.append(new_addr & 0xFF)
                result.append((new_addr >> 8) & 0xFF)
                i += 2
            else:
                result.append(opcode)
                i += 1
        elif opcode in IMMEDIATE_16BIT_OPCODES:
            # LDI, LDXI, LDYI: expand 8-bit immediate to 16-bit (sign-extend)
            result.append(opcode)
            if i + 1 < len(program):
                imm_val = program[i + 1]
                # Sign-extend: if bit 7 is set, the value is negative in 8-bit
                # and should become a 16-bit negative (0xFF in high byte)
                if imm_val & 0x80:
                    imm_val = imm_val | 0xFF00  # Sign-extend to 16-bit
                result.append(imm_val & 0xFF)       # imm_lo
                result.append((imm_val >> 8) & 0xFF)  # imm_hi (0xFF for negative, 0x00 for positive)
                i += 2
            else:
                i += 1
        elif opcode in IMMEDIATE_8BIT_OPCODES:
            # IN, OUT: keep as 2 bytes (8-bit port number)
            result.append(opcode)
            if i + 1 < len(program):
                result.append(program[i + 1])
                i += 2
            else:
                i += 1
        else:
            result.append(opcode)
            i += 1

    return result


class NeanderTestbench:
    """Helper class for Neander CPU testing"""

    def __init__(self, dut):
        self.dut = dut
        self.clock = None

    async def setup(self, clock_period_ns=10):
        """Initialize clock and reset"""
        self.clock = Clock(self.dut.clk, clock_period_ns, units="ns")
        cocotb.start_soon(self.clock.start())

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

    async def load_program(self, program, start_addr=0, convert_to_16bit=True):
        """Load a program into memory"""
        if convert_to_16bit:
            program = convert_program_to_16bit(program)
        for i, byte in enumerate(program):
            await self.load_byte(start_addr + i, byte)

    async def read_memory(self, addr):
        """Read a byte from memory"""
        self.dut.mem_read_addr.value = addr
        await RisingEdge(self.dut.clk)
        return int(self.dut.mem_read_data.value)

    async def run_until_halt(self, max_cycles=100000):
        """Run CPU until HLT instruction or timeout"""
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

    async def wait_for_io_write(self, max_cycles=100000):
        """Wait for an I/O write operation"""
        cycles = 0
        while cycles < max_cycles:
            await RisingEdge(self.dut.clk)
            cycles += 1
            if self.dut.io_write.value == 1:
                return int(self.dut.io_out.value)
        raise TimeoutError(f"No I/O write within {max_cycles} cycles")
