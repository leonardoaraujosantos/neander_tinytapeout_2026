"""
Large Program Tests for Neander CPU
====================================

Tests with larger, more complex programs to stress test the CPU.
These tests verify correct operation with:
- Bubble sort algorithm
- Memory block operations
- Checksum calculations
- Array processing
- Complex control flow
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

# Import from shared common module (no tests, avoids test discovery issues)
from neander_common import (
    NeanderTestbench, Op, convert_program_to_16bit,
    MEMORY_ADDRESS_OPCODES, JUMP_OPCODES, IMMEDIATE_OPCODES
)


# ============================================================================
# Bubble Sort Tests
# ============================================================================

@cocotb.test()
async def test_large_bubble_sort_4_elements(dut):
    """Bubble sort 4 elements"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Data at 0x0200 - unsorted array
    array_base = 0x0200
    array_data = [0x44, 0x11, 0x33, 0x22]  # Should become [0x11, 0x22, 0x33, 0x44]
    n = len(array_data)

    for i, val in enumerate(array_data):
        await tb.load_byte(array_base + i, val)

    # Variables at 0x0210
    var_i = 0x0210       # Outer loop counter
    var_j = 0x0211       # Inner loop counter
    var_n = 0x0212       # Array length
    var_temp = 0x0213    # Temp for swap
    var_a = 0x0214       # array[j]
    var_b = 0x0215       # array[j+1]

    await tb.load_byte(var_n, n)

    # Bubble sort program (manually constructed for 16-bit addressing)
    # Outer loop: for i = n-1 downto 1
    # Inner loop: for j = 0 to i-1
    #   if array[j] > array[j+1]: swap

    program = [
        # Initialize i = n-1
        Op.LDA, (var_n & 0xFF), (var_n >> 8),           # 0x00: LDA var_n
        Op.DEC,                                          # 0x03: DEC
        Op.STA, (var_i & 0xFF), (var_i >> 8),           # 0x04: STA var_i

        # outer_loop: check if i == 0
        # 0x07: outer_loop
        Op.LDA, (var_i & 0xFF), (var_i >> 8),           # 0x07: LDA var_i
        Op.JZ, 0x60, 0x00,                              # 0x0A: JZ done (0x0060)

        # Initialize j = 0
        Op.LDI, 0x00,                                    # 0x0D: LDI 0
        Op.STA, (var_j & 0xFF), (var_j >> 8),           # 0x0F: STA var_j

        # inner_loop: check if j >= i
        # 0x12: inner_loop
        Op.LDA, (var_j & 0xFF), (var_j >> 8),           # 0x12: LDA var_j
        Op.LDXI, 0x00,                                   # 0x15: LDXI 0
        Op.TAX,                                          # 0x17: TAX (X = j)
        Op.CMP, (var_i & 0xFF), (var_i >> 8),           # 0x18: CMP var_i
        Op.JGE, 0x54, 0x00,                             # 0x1B: JGE next_outer (0x0054)

        # Load array[j] into var_a
        Op.LDA_X, (array_base & 0xFF), (array_base >> 8), # 0x1E: LDA array_base,X
        Op.STA, (var_a & 0xFF), (var_a >> 8),           # 0x21: STA var_a

        # Load array[j+1] into var_b (increment X first)
        Op.INX,                                          # 0x24: INX
        Op.LDA_X, (array_base & 0xFF), (array_base >> 8), # 0x25: LDA array_base,X
        Op.STA, (var_b & 0xFF), (var_b >> 8),           # 0x28: STA var_b

        # Compare: if array[j] <= array[j+1], no swap needed
        Op.LDA, (var_a & 0xFF), (var_a >> 8),           # 0x2B: LDA var_a
        Op.CMP, (var_b & 0xFF), (var_b >> 8),           # 0x2E: CMP var_b
        Op.JLE, 0x4B, 0x00,                             # 0x31: JLE no_swap (0x004B)

        # Swap: array[j] = var_b, array[j+1] = var_a
        Op.LDA, (var_j & 0xFF), (var_j >> 8),           # 0x34: LDA var_j
        Op.TAX,                                          # 0x37: TAX
        Op.LDA, (var_b & 0xFF), (var_b >> 8),           # 0x38: LDA var_b
        Op.STA_X, (array_base & 0xFF), (array_base >> 8), # 0x3B: STA array_base,X
        Op.INX,                                          # 0x3E: INX
        Op.LDA, (var_a & 0xFF), (var_a >> 8),           # 0x3F: LDA var_a
        Op.STA_X, (array_base & 0xFF), (array_base >> 8), # 0x42: STA array_base,X
        Op.NOP,                                          # 0x45: padding
        Op.NOP,                                          # 0x46: padding
        Op.NOP,                                          # 0x47: padding
        Op.NOP,                                          # 0x48: padding
        Op.NOP,                                          # 0x49: padding
        Op.NOP,                                          # 0x4A: padding

        # no_swap: j++
        # 0x4B: no_swap
        Op.LDA, (var_j & 0xFF), (var_j >> 8),           # 0x4B: LDA var_j
        Op.INC,                                          # 0x4E: INC
        Op.STA, (var_j & 0xFF), (var_j >> 8),           # 0x4F: STA var_j
        Op.JMP, 0x12, 0x00,                             # 0x52: JMP inner_loop

        # next_outer: i--
        # 0x54: (pad to here with NOPs if needed)
        Op.NOP,                                          # 0x55: padding
        Op.LDA, (var_i & 0xFF), (var_i >> 8),           # 0x56: LDA var_i
        Op.DEC,                                          # 0x59: DEC
        Op.STA, (var_i & 0xFF), (var_i >> 8),           # 0x5A: STA var_i
        Op.JMP, 0x07, 0x00,                             # 0x5D: JMP outer_loop

        # done: output result and halt
        # 0x60: done
        Op.LDA, (array_base & 0xFF), (array_base >> 8), # 0x60: LDA array[0]
        Op.OUT, 0x00,                                    # 0x63: OUT 0
        Op.HLT,                                          # 0x65: HLT
    ]

    # Load program at address 0
    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    await tb.reset()
    await tb.run_until_halt(max_cycles=500000)

    # Verify sorted array
    sorted_result = []
    for i in range(n):
        val = await tb.read_memory(array_base + i)
        sorted_result.append(val)

    expected = sorted(array_data)
    assert sorted_result == expected, f"Sort failed: got {sorted_result}, expected {expected}"


@cocotb.test()
async def test_large_bubble_sort_8_elements(dut):
    """Bubble sort 8 elements"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Data at 0x0200
    array_base = 0x0200
    array_data = [0x88, 0x44, 0x22, 0x11, 0x77, 0x33, 0x66, 0x55]
    n = len(array_data)

    for i, val in enumerate(array_data):
        await tb.load_byte(array_base + i, val)

    # Variables
    var_i = 0x0210
    var_j = 0x0211
    var_n = 0x0212
    var_a = 0x0213
    var_b = 0x0214

    await tb.load_byte(var_n, n)

    # Simplified bubble sort using indexed addressing
    program = [
        # Initialize i = n-1
        Op.LDA, (var_n & 0xFF), (var_n >> 8),
        Op.DEC,
        Op.STA, (var_i & 0xFF), (var_i >> 8),

        # outer_loop: 0x07
        Op.LDA, (var_i & 0xFF), (var_i >> 8),
        Op.JZ, 0x58, 0x00,  # done

        Op.LDI, 0x00,
        Op.STA, (var_j & 0xFF), (var_j >> 8),

        # inner_loop: 0x12
        Op.LDA, (var_j & 0xFF), (var_j >> 8),
        Op.TAX,
        Op.CMP, (var_i & 0xFF), (var_i >> 8),
        Op.JGE, 0x4C, 0x00,  # next_outer

        # Load array[j]
        Op.LDA_X, (array_base & 0xFF), (array_base >> 8),
        Op.STA, (var_a & 0xFF), (var_a >> 8),

        # Load array[j+1]
        Op.INX,
        Op.LDA_X, (array_base & 0xFF), (array_base >> 8),
        Op.STA, (var_b & 0xFF), (var_b >> 8),

        # Compare
        Op.LDA, (var_a & 0xFF), (var_a >> 8),
        Op.CMP, (var_b & 0xFF), (var_b >> 8),
        Op.JLE, 0x43, 0x00,  # no_swap

        # Swap
        Op.LDA, (var_j & 0xFF), (var_j >> 8),
        Op.TAX,
        Op.LDA, (var_b & 0xFF), (var_b >> 8),
        Op.STA_X, (array_base & 0xFF), (array_base >> 8),
        Op.INX,
        Op.LDA, (var_a & 0xFF), (var_a >> 8),
        Op.STA_X, (array_base & 0xFF), (array_base >> 8),

        # no_swap: 0x43
        Op.LDA, (var_j & 0xFF), (var_j >> 8),
        Op.INC,
        Op.STA, (var_j & 0xFF), (var_j >> 8),
        Op.JMP, 0x12, 0x00,

        # next_outer: 0x4C
        Op.LDA, (var_i & 0xFF), (var_i >> 8),
        Op.DEC,
        Op.STA, (var_i & 0xFF), (var_i >> 8),
        Op.JMP, 0x07, 0x00,

        # done: 0x58
        Op.LDA, (array_base & 0xFF), (array_base >> 8),
        Op.OUT, 0x00,
        Op.HLT,
    ]

    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    await tb.reset()
    await tb.run_until_halt(max_cycles=1000000)

    # Verify sorted
    sorted_result = []
    for i in range(n):
        val = await tb.read_memory(array_base + i)
        sorted_result.append(val)

    expected = sorted(array_data)
    assert sorted_result == expected, f"Sort failed: got {sorted_result}, expected {expected}"


# ============================================================================
# Memory Block Operations Tests
# ============================================================================

@cocotb.test()
async def test_large_memcpy_16_bytes(dut):
    """Copy 16 bytes from one memory region to another"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    src_addr = 0x0200
    dst_addr = 0x0300
    count = 16

    # Fill source with test pattern
    src_data = [i * 0x11 for i in range(count)]  # 0x00, 0x11, 0x22, ...
    for i, val in enumerate(src_data):
        await tb.load_byte(src_addr + i, val)

    # Variables
    var_count = 0x0180
    await tb.load_byte(var_count, count)

    # memcpy program using X register
    program = [
        Op.LDI, 0x00,                                    # 0x00: X = 0
        Op.TAX,                                          # 0x02

        # loop: 0x03
        Op.TXA,                                          # 0x03: AC = X
        Op.CMP, (var_count & 0xFF), (var_count >> 8),   # 0x04: CMP count
        Op.JGE, 0x1A, 0x00,                             # 0x07: JGE done

        # Copy byte: dst[X] = src[X]
        Op.LDA_X, (src_addr & 0xFF), (src_addr >> 8),   # 0x0A: LDA src,X
        Op.STA_X, (dst_addr & 0xFF), (dst_addr >> 8),   # 0x0D: STA dst,X

        Op.INX,                                          # 0x10: X++
        Op.JMP, 0x03, 0x00,                             # 0x11: JMP loop

        # done: 0x1A (adjusted)
        Op.NOP, Op.NOP, Op.NOP, Op.NOP, Op.NOP, Op.NOP, # padding to 0x1A
        Op.LDA, (dst_addr & 0xFF), (dst_addr >> 8),     # 0x1A: LDA dst[0]
        Op.OUT, 0x00,                                    # 0x1D: OUT
        Op.HLT,                                          # 0x1F: HLT
    ]

    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    await tb.reset()
    await tb.run_until_halt(max_cycles=200000)

    # Verify copy
    for i in range(count):
        val = await tb.read_memory(dst_addr + i)
        assert val == src_data[i], f"memcpy failed at offset {i}: got {val}, expected {src_data[i]}"


@cocotb.test()
async def test_large_memcpy_32_bytes(dut):
    """Copy 32 bytes from one memory region to another"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    src_addr = 0x0400
    dst_addr = 0x0500
    count = 32

    # Fill source with test pattern
    src_data = [(i * 7 + 3) & 0xFF for i in range(count)]
    for i, val in enumerate(src_data):
        await tb.load_byte(src_addr + i, val)

    var_count = 0x0380
    await tb.load_byte(var_count, count)

    program = [
        Op.LDI, 0x00,
        Op.TAX,

        # loop: 0x03
        Op.TXA,
        Op.CMP, (var_count & 0xFF), (var_count >> 8),
        Op.JGE, 0x1A, 0x00,

        Op.LDA_X, (src_addr & 0xFF), (src_addr >> 8),
        Op.STA_X, (dst_addr & 0xFF), (dst_addr >> 8),

        Op.INX,
        Op.JMP, 0x03, 0x00,

        Op.NOP, Op.NOP, Op.NOP, Op.NOP, Op.NOP, Op.NOP,
        Op.LDA, (dst_addr & 0xFF), (dst_addr >> 8),
        Op.OUT, 0x00,
        Op.HLT,
    ]

    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    await tb.reset()
    await tb.run_until_halt(max_cycles=400000)

    # Verify
    for i in range(count):
        val = await tb.read_memory(dst_addr + i)
        assert val == src_data[i], f"memcpy failed at offset {i}"


@cocotb.test()
async def test_large_memset_32_bytes(dut):
    """Fill 32 bytes with a constant value"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    dst_addr = 0x0600
    count = 32
    fill_value = 0xAA

    var_count = 0x0580
    var_fill = 0x0581
    await tb.load_byte(var_count, count)
    await tb.load_byte(var_fill, fill_value)

    program = [
        Op.LDI, 0x00,
        Op.TAX,

        # loop: 0x03
        Op.TXA,
        Op.CMP, (var_count & 0xFF), (var_count >> 8),
        Op.JGE, 0x17, 0x00,

        Op.LDA, (var_fill & 0xFF), (var_fill >> 8),
        Op.STA_X, (dst_addr & 0xFF), (dst_addr >> 8),

        Op.INX,
        Op.JMP, 0x03, 0x00,

        Op.NOP, Op.NOP, Op.NOP,
        Op.LDA, (dst_addr & 0xFF), (dst_addr >> 8),
        Op.OUT, 0x00,
        Op.HLT,
    ]

    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    await tb.reset()
    await tb.run_until_halt(max_cycles=400000)

    # Verify
    for i in range(count):
        val = await tb.read_memory(dst_addr + i)
        assert val == fill_value, f"memset failed at offset {i}: got {val}, expected {fill_value}"


# ============================================================================
# Checksum Calculation Tests
# ============================================================================

@cocotb.test()
async def test_large_checksum_16_bytes(dut):
    """Calculate XOR checksum of 16 bytes"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    data_addr = 0x0200
    count = 16

    # Test data
    test_data = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
                 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]

    for i, val in enumerate(test_data):
        await tb.load_byte(data_addr + i, val)

    # Calculate expected checksum
    expected_checksum = 0
    for val in test_data:
        expected_checksum ^= val

    var_count = 0x0280
    var_sum = 0x0281
    await tb.load_byte(var_count, count)
    await tb.load_byte(var_sum, 0)

    # Address calculation:
    # 0x00-0x01: LDI (2), 0x02: TAX (1), 0x03-0x05: STA (3)
    # loop at 0x06
    # 0x06: TXA (1), 0x07-0x09: CMP (3), 0x0A-0x0C: JGE (3)
    # 0x0D-0x0F: LDA_X (3), 0x10-0x12: XOR (3), 0x13-0x15: STA (3)
    # 0x16: INX (1), 0x17-0x19: JMP (3)
    # done at 0x1A
    program = [
        Op.LDI, 0x00,                                    # 0x00
        Op.TAX,                                          # 0x02
        Op.STA, (var_sum & 0xFF), (var_sum >> 8),       # 0x03

        # loop: 0x06
        Op.TXA,                                          # 0x06
        Op.CMP, (var_count & 0xFF), (var_count >> 8),   # 0x07
        Op.JGE, 0x1A, 0x00,                             # 0x0A -> done at 0x1A

        Op.LDA_X, (data_addr & 0xFF), (data_addr >> 8), # 0x0D
        Op.XOR, (var_sum & 0xFF), (var_sum >> 8),       # 0x10
        Op.STA, (var_sum & 0xFF), (var_sum >> 8),       # 0x13

        Op.INX,                                          # 0x16
        Op.JMP, 0x06, 0x00,                             # 0x17

        # done: 0x1A
        Op.LDA, (var_sum & 0xFF), (var_sum >> 8),       # 0x1A
        Op.OUT, 0x00,                                    # 0x1D
        Op.HLT,                                          # 0x1F
    ]

    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    await tb.reset()
    result = await tb.wait_for_io_write(max_cycles=200000)

    assert result == expected_checksum, f"Checksum mismatch: got 0x{result:02X}, expected 0x{expected_checksum:02X}"


@cocotb.test()
async def test_large_checksum_additive_32_bytes(dut):
    """Calculate additive checksum (sum mod 256) of 32 bytes"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    data_addr = 0x0300
    count = 32

    # Test data
    test_data = [(i * 13 + 5) & 0xFF for i in range(count)]

    for i, val in enumerate(test_data):
        await tb.load_byte(data_addr + i, val)

    expected_sum = sum(test_data) & 0xFF

    var_count = 0x0380
    var_sum = 0x0381
    await tb.load_byte(var_count, count)
    await tb.load_byte(var_sum, 0)

    program = [
        Op.LDI, 0x00,
        Op.TAX,
        Op.STA, (var_sum & 0xFF), (var_sum >> 8),

        # loop: 0x06
        Op.TXA,
        Op.CMP, (var_count & 0xFF), (var_count >> 8),
        Op.JGE, 0x1E, 0x00,

        Op.LDA, (var_sum & 0xFF), (var_sum >> 8),
        Op.ADD, (data_addr & 0xFF), (data_addr >> 8),  # This won't work with indexed...
        Op.STA, (var_sum & 0xFF), (var_sum >> 8),

        Op.INX,
        Op.JMP, 0x06, 0x00,

        Op.NOP, Op.NOP,
        Op.LDA, (var_sum & 0xFF), (var_sum >> 8),
        Op.OUT, 0x00,
        Op.HLT,
    ]

    # This program is incorrect - ADD doesn't support indexed mode
    # Let's rewrite it properly

    var_temp = 0x0382
    program = [
        Op.LDI, 0x00,
        Op.TAX,
        Op.STA, (var_sum & 0xFF), (var_sum >> 8),

        # loop: 0x06
        Op.TXA,
        Op.CMP, (var_count & 0xFF), (var_count >> 8),
        Op.JGE, 0x21, 0x00,  # done

        # temp = data[X]
        Op.LDA_X, (data_addr & 0xFF), (data_addr >> 8),
        Op.STA, (var_temp & 0xFF), (var_temp >> 8),

        # sum = sum + temp
        Op.LDA, (var_sum & 0xFF), (var_sum >> 8),
        Op.ADD, (var_temp & 0xFF), (var_temp >> 8),
        Op.STA, (var_sum & 0xFF), (var_sum >> 8),

        Op.INX,
        Op.JMP, 0x06, 0x00,

        # done: 0x21
        Op.NOP, Op.NOP, Op.NOP,
        Op.LDA, (var_sum & 0xFF), (var_sum >> 8),
        Op.OUT, 0x00,
        Op.HLT,
    ]

    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    await tb.reset()
    result = await tb.wait_for_io_write(max_cycles=400000)

    assert result == expected_sum, f"Additive checksum mismatch: got 0x{result:02X}, expected 0x{expected_sum:02X}"


# ============================================================================
# Array Processing Tests
# ============================================================================

@cocotb.test()
async def test_large_array_find_max(dut):
    """Find maximum value in an array of 16 elements"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    array_addr = 0x0200
    count = 16

    # Test data with known maximum (all values < 0x80 for signed comparison)
    test_data = [0x23, 0x45, 0x12, 0x7E, 0x34, 0x67, 0x5E, 0x11,
                 0x19, 0x22, 0x44, 0x66, 0x58, 0x6A, 0x55, 0x77]
    expected_max = max(test_data)

    for i, val in enumerate(test_data):
        await tb.load_byte(array_addr + i, val)

    var_count = 0x0280
    var_max = 0x0281
    var_curr = 0x0282
    await tb.load_byte(var_count, count)

    # Address calc: loop at 0x09, no_update at 0x22, done at 0x26
    # Use signed comparison (JLE) - all test values are < 0x80
    program = [
        # Initialize max = array[0]
        Op.LDA, (array_addr & 0xFF), (array_addr >> 8),  # 0x00
        Op.STA, (var_max & 0xFF), (var_max >> 8),       # 0x03

        # X = 1
        Op.LDI, 0x01,                                    # 0x06
        Op.TAX,                                          # 0x08

        # loop: 0x09
        Op.TXA,                                          # 0x09
        Op.CMP, (var_count & 0xFF), (var_count >> 8),   # 0x0A
        Op.JGE, 0x26, 0x00,                             # 0x0D -> done at 0x26

        # curr = array[X]
        Op.LDA_X, (array_addr & 0xFF), (array_addr >> 8), # 0x10
        Op.STA, (var_curr & 0xFF), (var_curr >> 8),     # 0x13

        # if curr > max: max = curr (skip if curr <= max)
        Op.CMP, (var_max & 0xFF), (var_max >> 8),       # 0x16
        Op.JLE, 0x22, 0x00,                             # 0x19 -> no_update (JLE = signed <=)

        # Update max
        Op.LDA, (var_curr & 0xFF), (var_curr >> 8),     # 0x1C
        Op.STA, (var_max & 0xFF), (var_max >> 8),       # 0x1F

        # no_update: 0x22
        Op.INX,                                          # 0x22
        Op.JMP, 0x09, 0x00,                             # 0x23

        # done: 0x26
        Op.LDA, (var_max & 0xFF), (var_max >> 8),       # 0x26
        Op.OUT, 0x00,                                    # 0x29
        Op.HLT,                                          # 0x2B
    ]

    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    await tb.reset()
    result = await tb.wait_for_io_write(max_cycles=200000)

    assert result == expected_max, f"Find max failed: got 0x{result:02X}, expected 0x{expected_max:02X}"


@cocotb.test()
async def test_large_array_find_min(dut):
    """Find minimum value in an array of 16 elements"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    array_addr = 0x0200
    count = 16

    # Test data with known minimum (all values < 0x80 for signed comparison)
    test_data = [0x23, 0x45, 0x12, 0x39, 0x34, 0x67, 0x02, 0x11,
                 0x29, 0x22, 0x44, 0x66, 0x38, 0x4A, 0x55, 0x77]
    expected_min = min(test_data)

    for i, val in enumerate(test_data):
        await tb.load_byte(array_addr + i, val)

    var_count = 0x0280
    var_min = 0x0281
    var_curr = 0x0282
    await tb.load_byte(var_count, count)

    # Address calc: loop at 0x09, no_update at 0x25, done at 0x29
    # Use signed comparison (JLE) - all test values are < 0x80
    program = [
        # Initialize min = array[0]
        Op.LDA, (array_addr & 0xFF), (array_addr >> 8),  # 0x00
        Op.STA, (var_min & 0xFF), (var_min >> 8),       # 0x03

        Op.LDI, 0x01,                                    # 0x06
        Op.TAX,                                          # 0x08

        # loop: 0x09
        Op.TXA,                                          # 0x09
        Op.CMP, (var_count & 0xFF), (var_count >> 8),   # 0x0A
        Op.JGE, 0x29, 0x00,                             # 0x0D -> done at 0x29

        Op.LDA_X, (array_addr & 0xFF), (array_addr >> 8), # 0x10
        Op.STA, (var_curr & 0xFF), (var_curr >> 8),     # 0x13

        # if curr < min: min = curr (skip if min <= curr)
        # Load min, compare with curr: min - curr
        Op.LDA, (var_min & 0xFF), (var_min >> 8),       # 0x16
        Op.CMP, (var_curr & 0xFF), (var_curr >> 8),     # 0x19
        Op.JLE, 0x25, 0x00,                             # 0x1C -> no_update (JLE = signed <=)

        Op.LDA, (var_curr & 0xFF), (var_curr >> 8),     # 0x1F
        Op.STA, (var_min & 0xFF), (var_min >> 8),       # 0x22

        # no_update: 0x25
        Op.INX,                                          # 0x25
        Op.JMP, 0x09, 0x00,                             # 0x26

        # done: 0x29
        Op.LDA, (var_min & 0xFF), (var_min >> 8),       # 0x29
        Op.OUT, 0x00,                                    # 0x2C
        Op.HLT,                                          # 0x2E
    ]

    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    await tb.reset()
    result = await tb.wait_for_io_write(max_cycles=200000)

    assert result == expected_min, f"Find min failed: got 0x{result:02X}, expected 0x{expected_min:02X}"


@cocotb.test()
async def test_large_array_count_threshold(dut):
    """Count elements greater than a threshold"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    array_addr = 0x0200
    count = 16
    threshold = 0x50

    test_data = [0x23, 0x45, 0x12, 0x89, 0x34, 0x67, 0x02, 0x11,
                 0x99, 0x22, 0x44, 0x66, 0x88, 0xAA, 0x55, 0x77]
    expected_count = sum(1 for v in test_data if v > threshold)

    for i, val in enumerate(test_data):
        await tb.load_byte(array_addr + i, val)

    var_count = 0x0280
    var_threshold = 0x0281
    var_result = 0x0282
    var_curr = 0x0283
    await tb.load_byte(var_count, count)
    await tb.load_byte(var_threshold, threshold)
    await tb.load_byte(var_result, 0)

    # Address calc: loop at 0x06, no_count at 0x20, done at 0x24
    program = [
        Op.LDI, 0x00,                                    # 0x00
        Op.TAX,                                          # 0x02
        Op.STA, (var_result & 0xFF), (var_result >> 8), # 0x03

        # loop: 0x06
        Op.TXA,                                          # 0x06
        Op.CMP, (var_count & 0xFF), (var_count >> 8),   # 0x07
        Op.JGE, 0x24, 0x00,                             # 0x0A -> done at 0x24

        Op.LDA_X, (array_addr & 0xFF), (array_addr >> 8), # 0x0D
        Op.STA, (var_curr & 0xFF), (var_curr >> 8),     # 0x10

        # if curr > threshold: result++
        Op.CMP, (var_threshold & 0xFF), (var_threshold >> 8), # 0x13
        Op.JLE, 0x20, 0x00,                             # 0x16 -> no_count at 0x20

        # Increment result
        Op.LDA, (var_result & 0xFF), (var_result >> 8), # 0x19
        Op.INC,                                          # 0x1C
        Op.STA, (var_result & 0xFF), (var_result >> 8), # 0x1D

        # no_count: 0x20
        Op.INX,                                          # 0x20
        Op.JMP, 0x06, 0x00,                             # 0x21

        # done: 0x24
        Op.LDA, (var_result & 0xFF), (var_result >> 8), # 0x24
        Op.OUT, 0x00,                                    # 0x27
        Op.HLT,                                          # 0x29
    ]

    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    await tb.reset()
    result = await tb.wait_for_io_write(max_cycles=200000)

    assert result == expected_count, f"Count threshold failed: got {result}, expected {expected_count}"


# ============================================================================
# Complex Control Flow Tests
# ============================================================================

@cocotb.test()
async def test_large_state_machine(dut):
    """Simple state machine with 4 states"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # State machine: count transitions through 4 states
    # State 0 -> State 1 -> State 2 -> State 3 -> done
    # Each state increments a counter

    var_state = 0x0200
    var_counter = 0x0201

    await tb.load_byte(var_state, 0)
    await tb.load_byte(var_counter, 0)

    # Address calc: Each state block is ~15 bytes
    # main_loop: 0x00, not_state_0: 0x15, not_state_1: 0x2D, not_state_2: 0x45
    program = [
        # main_loop: 0x00
        Op.LDA, (var_state & 0xFF), (var_state >> 8),   # 0x00

        # Check state 0
        Op.JNZ, 0x15, 0x00,                             # 0x03 -> not_state_0
        # State 0: increment counter, go to state 1
        Op.LDA, (var_counter & 0xFF), (var_counter >> 8), # 0x06
        Op.INC,                                          # 0x09
        Op.STA, (var_counter & 0xFF), (var_counter >> 8), # 0x0A
        Op.LDI, 0x01,                                    # 0x0D
        Op.STA, (var_state & 0xFF), (var_state >> 8),   # 0x0F
        Op.JMP, 0x00, 0x00,                             # 0x12

        # not_state_0: 0x15
        Op.LDA, (var_state & 0xFF), (var_state >> 8),   # 0x15
        Op.CMP, (0x0202 & 0xFF), (0x0202 >> 8),        # 0x18
        Op.JNZ, 0x2D, 0x00,                             # 0x1B -> not_state_1
        # State 1: increment counter, go to state 2
        Op.LDA, (var_counter & 0xFF), (var_counter >> 8), # 0x1E
        Op.INC,                                          # 0x21
        Op.STA, (var_counter & 0xFF), (var_counter >> 8), # 0x22
        Op.LDI, 0x02,                                    # 0x25
        Op.STA, (var_state & 0xFF), (var_state >> 8),   # 0x27
        Op.JMP, 0x00, 0x00,                             # 0x2A

        # not_state_1: 0x2D
        Op.LDA, (var_state & 0xFF), (var_state >> 8),   # 0x2D
        Op.CMP, (0x0203 & 0xFF), (0x0203 >> 8),        # 0x30
        Op.JNZ, 0x45, 0x00,                             # 0x33 -> not_state_2
        # State 2: increment counter, go to state 3
        Op.LDA, (var_counter & 0xFF), (var_counter >> 8), # 0x36
        Op.INC,                                          # 0x39
        Op.STA, (var_counter & 0xFF), (var_counter >> 8), # 0x3A
        Op.LDI, 0x03,                                    # 0x3D
        Op.STA, (var_state & 0xFF), (var_state >> 8),   # 0x3F
        Op.JMP, 0x00, 0x00,                             # 0x42

        # not_state_2: 0x45 (state 3 - done)
        Op.LDA, (var_counter & 0xFF), (var_counter >> 8), # 0x45
        Op.INC,                                          # 0x48
        Op.OUT, 0x00,                                    # 0x49
        Op.HLT,                                          # 0x4B
    ]

    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    # Store constants for comparison
    await tb.load_byte(0x0202, 0x01)  # Constant 1
    await tb.load_byte(0x0203, 0x02)  # Constant 2

    await tb.reset()
    result = await tb.wait_for_io_write(max_cycles=100000)

    # Should have 4 increments (one per state)
    assert result == 4, f"State machine failed: got {result}, expected 4"


@cocotb.test()
async def test_large_nested_loops(dut):
    """Nested loops: outer * inner iterations"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    outer_count = 4
    inner_count = 5
    expected = outer_count * inner_count

    var_outer = 0x0200
    var_inner = 0x0201
    var_counter = 0x0202
    var_outer_max = 0x0203
    var_inner_max = 0x0204

    await tb.load_byte(var_outer_max, outer_count)
    await tb.load_byte(var_inner_max, inner_count)
    await tb.load_byte(var_counter, 0)

    # Address calc: outer_loop: 0x05, inner_loop: 0x13, next_outer: 0x2D, done: 0x37
    program = [
        # outer = 0
        Op.LDI, 0x00,                                    # 0x00
        Op.STA, (var_outer & 0xFF), (var_outer >> 8),   # 0x02

        # outer_loop: 0x05
        Op.LDA, (var_outer & 0xFF), (var_outer >> 8),   # 0x05
        Op.CMP, (var_outer_max & 0xFF), (var_outer_max >> 8), # 0x08
        Op.JGE, 0x37, 0x00,                             # 0x0B -> done

        # inner = 0
        Op.LDI, 0x00,                                    # 0x0E
        Op.STA, (var_inner & 0xFF), (var_inner >> 8),   # 0x10

        # inner_loop: 0x13
        Op.LDA, (var_inner & 0xFF), (var_inner >> 8),   # 0x13
        Op.CMP, (var_inner_max & 0xFF), (var_inner_max >> 8), # 0x16
        Op.JGE, 0x2D, 0x00,                             # 0x19 -> next_outer

        # counter++
        Op.LDA, (var_counter & 0xFF), (var_counter >> 8), # 0x1C
        Op.INC,                                          # 0x1F
        Op.STA, (var_counter & 0xFF), (var_counter >> 8), # 0x20

        # inner++
        Op.LDA, (var_inner & 0xFF), (var_inner >> 8),   # 0x23
        Op.INC,                                          # 0x26
        Op.STA, (var_inner & 0xFF), (var_inner >> 8),   # 0x27
        Op.JMP, 0x13, 0x00,                             # 0x2A

        # next_outer: 0x2D
        Op.LDA, (var_outer & 0xFF), (var_outer >> 8),   # 0x2D
        Op.INC,                                          # 0x30
        Op.STA, (var_outer & 0xFF), (var_outer >> 8),   # 0x31
        Op.JMP, 0x05, 0x00,                             # 0x34

        # done: 0x37
        Op.LDA, (var_counter & 0xFF), (var_counter >> 8), # 0x37
        Op.OUT, 0x00,                                    # 0x3A
        Op.HLT,                                          # 0x3C
    ]

    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    await tb.reset()
    result = await tb.wait_for_io_write(max_cycles=300000)

    assert result == expected, f"Nested loops failed: got {result}, expected {expected}"


@cocotb.test()
async def test_large_fibonacci_10_terms(dut):
    """Calculate 10th Fibonacci number using iteration"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Fibonacci: 1, 1, 2, 3, 5, 8, 13, 21, 34, 55
    # F(10) = 55
    expected = 55

    var_a = 0x0200      # Previous value
    var_b = 0x0201      # Current value
    var_temp = 0x0202   # Temp for swap
    var_count = 0x0203  # Loop counter
    var_n = 0x0204      # Target count

    await tb.load_byte(var_n, 10)

    # Address calc: loop at 0x0D, done at 0x32
    program = [
        # a = 0, b = 1, count = 1
        Op.LDI, 0x00,                                    # 0x00
        Op.STA, (var_a & 0xFF), (var_a >> 8),           # 0x02
        Op.LDI, 0x01,                                    # 0x05
        Op.STA, (var_b & 0xFF), (var_b >> 8),           # 0x07
        Op.STA, (var_count & 0xFF), (var_count >> 8),   # 0x0A

        # loop: 0x0D
        Op.LDA, (var_count & 0xFF), (var_count >> 8),   # 0x0D
        Op.CMP, (var_n & 0xFF), (var_n >> 8),           # 0x10
        Op.JGE, 0x32, 0x00,                             # 0x13 -> done

        # temp = b
        Op.LDA, (var_b & 0xFF), (var_b >> 8),           # 0x16
        Op.STA, (var_temp & 0xFF), (var_temp >> 8),     # 0x19

        # b = a + b
        Op.ADD, (var_a & 0xFF), (var_a >> 8),           # 0x1C
        Op.STA, (var_b & 0xFF), (var_b >> 8),           # 0x1F

        # a = temp
        Op.LDA, (var_temp & 0xFF), (var_temp >> 8),     # 0x22
        Op.STA, (var_a & 0xFF), (var_a >> 8),           # 0x25

        # count++
        Op.LDA, (var_count & 0xFF), (var_count >> 8),   # 0x28
        Op.INC,                                          # 0x2B
        Op.STA, (var_count & 0xFF), (var_count >> 8),   # 0x2C
        Op.JMP, 0x0D, 0x00,                             # 0x2F

        # done: 0x32
        Op.LDA, (var_b & 0xFF), (var_b >> 8),           # 0x32
        Op.OUT, 0x00,                                    # 0x35
        Op.HLT,                                          # 0x37
    ]

    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    await tb.reset()
    result = await tb.wait_for_io_write(max_cycles=200000)

    assert result == expected, f"Fibonacci failed: got {result}, expected {expected}"


@cocotb.test()
async def test_large_gcd_euclidean(dut):
    """Calculate GCD using Euclidean algorithm"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    a = 48
    b = 18
    expected_gcd = 6  # GCD(48, 18) = 6

    var_a = 0x0200
    var_b = 0x0201
    var_temp = 0x0202

    await tb.load_byte(var_a, a)
    await tb.load_byte(var_b, b)

    # Subtraction-based GCD: while a != b: if a > b: a -= b else: b -= a
    # Address calc: loop at 0x00, a_greater at 0x18, done at 0x24
    program = [
        # loop: 0x00
        Op.LDA, (var_a & 0xFF), (var_a >> 8),           # 0x00
        Op.CMP, (var_b & 0xFF), (var_b >> 8),           # 0x03
        Op.JZ, 0x24, 0x00,                              # 0x06 -> done
        Op.JGT, 0x18, 0x00,                             # 0x09 -> a_greater

        # b > a: b = b - a
        Op.LDA, (var_b & 0xFF), (var_b >> 8),           # 0x0C
        Op.SUB, (var_a & 0xFF), (var_a >> 8),           # 0x0F
        Op.STA, (var_b & 0xFF), (var_b >> 8),           # 0x12
        Op.JMP, 0x00, 0x00,                             # 0x15

        # a_greater: 0x18 - a = a - b
        Op.LDA, (var_a & 0xFF), (var_a >> 8),           # 0x18
        Op.SUB, (var_b & 0xFF), (var_b >> 8),           # 0x1B
        Op.STA, (var_a & 0xFF), (var_a >> 8),           # 0x1E
        Op.JMP, 0x00, 0x00,                             # 0x21

        # done: 0x24
        Op.LDA, (var_a & 0xFF), (var_a >> 8),           # 0x24
        Op.OUT, 0x00,                                    # 0x27
        Op.HLT,                                          # 0x29
    ]

    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    await tb.reset()
    result = await tb.wait_for_io_write(max_cycles=200000)

    assert result == expected_gcd, f"GCD failed: got {result}, expected {expected_gcd}"


# ============================================================================
# Large Data Operations Tests
# ============================================================================

@cocotb.test()
async def test_large_array_reverse_8_elements(dut):
    """Reverse an array of 8 elements in place"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    array_addr = 0x0200
    count = 8

    test_data = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]
    expected = list(reversed(test_data))

    for i, val in enumerate(test_data):
        await tb.load_byte(array_addr + i, val)

    var_left = 0x0280
    var_right = 0x0281
    var_temp = 0x0282
    var_left_val = 0x0283
    var_right_val = 0x0284

    await tb.load_byte(var_left, 0)
    await tb.load_byte(var_right, count - 1)

    program = [
        # loop: 0x00
        # Check if left >= right
        Op.LDA, (var_left & 0xFF), (var_left >> 8),
        Op.CMP, (var_right & 0xFF), (var_right >> 8),
        Op.JGE, 0x3A, 0x00,  # done

        # left_val = array[left]
        Op.LDA, (var_left & 0xFF), (var_left >> 8),
        Op.TAX,
        Op.LDA_X, (array_addr & 0xFF), (array_addr >> 8),
        Op.STA, (var_left_val & 0xFF), (var_left_val >> 8),

        # right_val = array[right]
        Op.LDA, (var_right & 0xFF), (var_right >> 8),
        Op.TAX,
        Op.LDA_X, (array_addr & 0xFF), (array_addr >> 8),
        Op.STA, (var_right_val & 0xFF), (var_right_val >> 8),

        # array[left] = right_val
        Op.LDA, (var_left & 0xFF), (var_left >> 8),
        Op.TAX,
        Op.LDA, (var_right_val & 0xFF), (var_right_val >> 8),
        Op.STA_X, (array_addr & 0xFF), (array_addr >> 8),

        # array[right] = left_val
        Op.LDA, (var_right & 0xFF), (var_right >> 8),
        Op.TAX,
        Op.LDA, (var_left_val & 0xFF), (var_left_val >> 8),
        Op.STA_X, (array_addr & 0xFF), (array_addr >> 8),

        # left++, right--
        Op.LDA, (var_left & 0xFF), (var_left >> 8),
        Op.INC,
        Op.STA, (var_left & 0xFF), (var_left >> 8),
        Op.LDA, (var_right & 0xFF), (var_right >> 8),
        Op.DEC,
        Op.STA, (var_right & 0xFF), (var_right >> 8),
        Op.JMP, 0x00, 0x00,

        # done: 0x3A
        Op.LDA, (array_addr & 0xFF), (array_addr >> 8),  # First element
        Op.OUT, 0x00,
        Op.HLT,
    ]

    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    await tb.reset()
    await tb.run_until_halt(max_cycles=300000)

    # Verify reversed array
    result = []
    for i in range(count):
        val = await tb.read_memory(array_addr + i)
        result.append(val)

    assert result == expected, f"Array reverse failed: got {result}, expected {expected}"


@cocotb.test()
async def test_large_multiply_using_shifts(dut):
    """Multiply two numbers using shift-and-add"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    # Calculate 7 * 13 = 91
    a = 7
    b = 13
    expected = (a * b) & 0xFF

    var_a = 0x0200
    var_b = 0x0201
    var_result = 0x0202
    var_bit = 0x0203

    await tb.load_byte(var_a, a)
    await tb.load_byte(var_b, b)
    await tb.load_byte(var_result, 0)
    await tb.load_byte(var_bit, 0x01)  # Bit mask starting at 1

    # Shift-and-add multiplication
    # Address calc: loop at 0x00, skip_add at 0x15, done at 0x26
    program = [
        # loop: 0x00
        Op.LDA, (var_bit & 0xFF), (var_bit >> 8),       # 0x00
        Op.JZ, 0x26, 0x00,                              # 0x03 -> done (bit wrapped to 0)

        # Check if b & bit != 0
        Op.AND, (var_b & 0xFF), (var_b >> 8),           # 0x06
        Op.JZ, 0x15, 0x00,                              # 0x09 -> skip_add

        # result += a
        Op.LDA, (var_result & 0xFF), (var_result >> 8), # 0x0C
        Op.ADD, (var_a & 0xFF), (var_a >> 8),           # 0x0F
        Op.STA, (var_result & 0xFF), (var_result >> 8), # 0x12

        # skip_add: 0x15
        # a <<= 1
        Op.LDA, (var_a & 0xFF), (var_a >> 8),           # 0x15
        Op.SHL,                                          # 0x18
        Op.STA, (var_a & 0xFF), (var_a >> 8),           # 0x19

        # bit <<= 1
        Op.LDA, (var_bit & 0xFF), (var_bit >> 8),       # 0x1C
        Op.SHL,                                          # 0x1F
        Op.STA, (var_bit & 0xFF), (var_bit >> 8),       # 0x20
        Op.JMP, 0x00, 0x00,                             # 0x23

        # done: 0x26
        Op.LDA, (var_result & 0xFF), (var_result >> 8), # 0x26
        Op.OUT, 0x00,                                    # 0x29
        Op.HLT,                                          # 0x2B
    ]

    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    await tb.reset()
    result = await tb.wait_for_io_write(max_cycles=200000)

    assert result == expected, f"Shift multiply failed: got {result}, expected {expected}"


@cocotb.test()
async def test_large_linear_search(dut):
    """Linear search for a value in an array"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    array_addr = 0x0200
    count = 16
    target = 0x67

    test_data = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
                 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x67, 0x88]  # target at index 14
    expected_index = test_data.index(target)

    for i, val in enumerate(test_data):
        await tb.load_byte(array_addr + i, val)

    var_count = 0x0280
    var_target = 0x0281
    var_found = 0x0282  # 0xFF if not found, index if found
    var_curr = 0x0283

    await tb.load_byte(var_count, count)
    await tb.load_byte(var_target, target)
    await tb.load_byte(var_found, 0xFF)  # Not found initially

    # Address calc: loop at 0x03, not_match at 0x1A, done at 0x1E
    program = [
        Op.LDI, 0x00,                                    # 0x00
        Op.TAX,                                          # 0x02

        # loop: 0x03
        Op.TXA,                                          # 0x03
        Op.CMP, (var_count & 0xFF), (var_count >> 8),   # 0x04
        Op.JGE, 0x1E, 0x00,                             # 0x07 -> done

        Op.LDA_X, (array_addr & 0xFF), (array_addr >> 8), # 0x0A
        Op.CMP, (var_target & 0xFF), (var_target >> 8), # 0x0D
        Op.JNZ, 0x1A, 0x00,                             # 0x10 -> not_match

        # Found! Store index
        Op.TXA,                                          # 0x13
        Op.STA, (var_found & 0xFF), (var_found >> 8),   # 0x14
        Op.JMP, 0x1E, 0x00,                             # 0x17 -> done

        # not_match: 0x1A
        Op.INX,                                          # 0x1A
        Op.JMP, 0x03, 0x00,                             # 0x1B

        # done: 0x1E
        Op.LDA, (var_found & 0xFF), (var_found >> 8),   # 0x1E
        Op.OUT, 0x00,                                    # 0x21
        Op.HLT,                                          # 0x23
    ]

    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    await tb.reset()
    result = await tb.wait_for_io_write(max_cycles=200000)

    assert result == expected_index, f"Linear search failed: got {result}, expected {expected_index}"


# ============================================================================
# High Memory Stress Tests
# ============================================================================

@cocotb.test()
async def test_large_high_memory_bubble_sort(dut):
    """Bubble sort with data at high memory addresses (0x8000+)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    array_base = 0x8000  # High memory
    array_data = [0x44, 0x22, 0x33, 0x11]
    n = len(array_data)

    for i, val in enumerate(array_data):
        await tb.load_byte(array_base + i, val)

    var_i = 0x8100
    var_j = 0x8101
    var_n = 0x8102
    var_a = 0x8103
    var_b = 0x8104

    await tb.load_byte(var_n, n)

    program = [
        Op.LDA, (var_n & 0xFF), (var_n >> 8),
        Op.DEC,
        Op.STA, (var_i & 0xFF), (var_i >> 8),

        # outer_loop: 0x07
        Op.LDA, (var_i & 0xFF), (var_i >> 8),
        Op.JZ, 0x58, 0x00,

        Op.LDI, 0x00,
        Op.STA, (var_j & 0xFF), (var_j >> 8),

        # inner_loop: 0x12
        Op.LDA, (var_j & 0xFF), (var_j >> 8),
        Op.TAX,
        Op.CMP, (var_i & 0xFF), (var_i >> 8),
        Op.JGE, 0x4C, 0x00,

        Op.LDA_X, (array_base & 0xFF), (array_base >> 8),
        Op.STA, (var_a & 0xFF), (var_a >> 8),

        Op.INX,
        Op.LDA_X, (array_base & 0xFF), (array_base >> 8),
        Op.STA, (var_b & 0xFF), (var_b >> 8),

        Op.LDA, (var_a & 0xFF), (var_a >> 8),
        Op.CMP, (var_b & 0xFF), (var_b >> 8),
        Op.JLE, 0x43, 0x00,

        Op.LDA, (var_j & 0xFF), (var_j >> 8),
        Op.TAX,
        Op.LDA, (var_b & 0xFF), (var_b >> 8),
        Op.STA_X, (array_base & 0xFF), (array_base >> 8),
        Op.INX,
        Op.LDA, (var_a & 0xFF), (var_a >> 8),
        Op.STA_X, (array_base & 0xFF), (array_base >> 8),

        # no_swap: 0x43
        Op.LDA, (var_j & 0xFF), (var_j >> 8),
        Op.INC,
        Op.STA, (var_j & 0xFF), (var_j >> 8),
        Op.JMP, 0x12, 0x00,

        # next_outer: 0x4C
        Op.LDA, (var_i & 0xFF), (var_i >> 8),
        Op.DEC,
        Op.STA, (var_i & 0xFF), (var_i >> 8),
        Op.JMP, 0x07, 0x00,

        # done: 0x58
        Op.LDA, (array_base & 0xFF), (array_base >> 8),
        Op.OUT, 0x00,
        Op.HLT,
    ]

    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    await tb.reset()
    await tb.run_until_halt(max_cycles=500000)

    sorted_result = []
    for i in range(n):
        val = await tb.read_memory(array_base + i)
        sorted_result.append(val)

    expected = sorted(array_data)
    assert sorted_result == expected, f"High memory sort failed: got {sorted_result}, expected {expected}"


@cocotb.test()
async def test_large_cross_page_memcpy(dut):
    """Copy data across page boundaries (from 0x00F8 to 0x0108)"""
    tb = NeanderTestbench(dut)
    await tb.setup()

    src_addr = 0x00F8  # Starts before page boundary
    dst_addr = 0x0200
    count = 16         # Crosses into 0x0100+

    src_data = [(i * 17 + 3) & 0xFF for i in range(count)]
    for i, val in enumerate(src_data):
        await tb.load_byte(src_addr + i, val)

    var_count = 0x0180
    await tb.load_byte(var_count, count)

    program = [
        Op.LDI, 0x00,
        Op.TAX,

        # loop: 0x03
        Op.TXA,
        Op.CMP, (var_count & 0xFF), (var_count >> 8),
        Op.JGE, 0x1A, 0x00,

        Op.LDA_X, (src_addr & 0xFF), (src_addr >> 8),
        Op.STA_X, (dst_addr & 0xFF), (dst_addr >> 8),

        Op.INX,
        Op.JMP, 0x03, 0x00,

        Op.NOP, Op.NOP, Op.NOP, Op.NOP, Op.NOP, Op.NOP,
        Op.LDA, (dst_addr & 0xFF), (dst_addr >> 8),
        Op.OUT, 0x00,
        Op.HLT,
    ]

    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)

    await tb.reset()
    await tb.run_until_halt(max_cycles=200000)

    for i in range(count):
        val = await tb.read_memory(dst_addr + i)
        assert val == src_data[i], f"Cross-page memcpy failed at offset {i}"
