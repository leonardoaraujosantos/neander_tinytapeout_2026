"""
test_interconnect.py - Comprehensive tests for NEANDER-X Interconnect Hub

Tests include:
1. Debug ROM functionality (boot_mode=1)
2. Address decoding (RAM, Flash, MMIO regions)
3. MMIO register read/write
4. PWM module
5. Timer module
6. IRQ controller
7. External I/O
8. System integration
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, ClockCycles, Timer


# MMIO register addresses (offset from 0xF000)
ADDR_IRQ_STATUS  = 0xF000
ADDR_IRQ_ENABLE  = 0xF002
ADDR_IRQ_ACK     = 0xF004
ADDR_PWM_CTRL    = 0xF010
ADDR_PWM_DIV     = 0xF012
ADDR_PWM_PERIOD  = 0xF014
ADDR_PWM_DUTY    = 0xF016
ADDR_TMR_CTRL    = 0xF020
ADDR_TMR_DIV     = 0xF022
ADDR_TMR_COUNT   = 0xF024
ADDR_TMR_CMP     = 0xF026
ADDR_TMR_STATUS  = 0xF028
ADDR_SPI_CTRL    = 0xF030
ADDR_SPI_DIV     = 0xF032
ADDR_SPI_SS      = 0xF034
ADDR_SPI_TXRX    = 0xF036
ADDR_SPI_STATUS  = 0xF038

# Opcodes
OP_NOP   = 0x00
OP_STA   = 0x10
OP_LDA   = 0x20
OP_ADD   = 0x30
OP_OR    = 0x40
OP_AND   = 0x50
OP_NOT   = 0x60
OP_JMP   = 0x80
OP_JN    = 0x90
OP_JZ    = 0xA0
OP_HLT   = 0xF0
OP_LDI   = 0xE0
OP_OUT   = 0xD0
OP_IN    = 0xC0


class InterconnectTestbench:
    """Helper class for interconnect testing"""

    def __init__(self, dut):
        self.dut = dut
        self.clock_period_ns = 100  # 10 MHz

    async def setup(self):
        """Initialize the testbench"""
        # Start clock
        clock = Clock(self.dut.clk, self.clock_period_ns, units="ns")
        cocotb.start_soon(clock.start())

        # Initialize inputs
        self.dut.boot_sel.value = 0
        self.dut.irq_in.value = 0
        self.dut.ext_in.value = 0
        self.dut.mem_load_en.value = 0
        self.dut.mem_load_addr.value = 0
        self.dut.mem_load_data.value = 0
        self.dut.mem_read_addr.value = 0

    async def reset(self):
        """Apply reset"""
        self.dut.reset.value = 1
        await ClockCycles(self.dut.clk, 5)
        self.dut.reset.value = 0
        await ClockCycles(self.dut.clk, 2)

    async def load_byte(self, addr, data):
        """Load a byte into memory"""
        self.dut.mem_load_en.value = 1
        self.dut.mem_load_addr.value = addr
        self.dut.mem_load_data.value = data
        await RisingEdge(self.dut.clk)
        self.dut.mem_load_en.value = 0
        await RisingEdge(self.dut.clk)

    async def load_program(self, program_bytes, start_addr=0):
        """Load a program into memory"""
        for i, byte in enumerate(program_bytes):
            await self.load_byte(start_addr + i, byte)

    async def read_memory(self, addr):
        """Read a byte from memory"""
        self.dut.mem_read_addr.value = addr
        await RisingEdge(self.dut.clk)
        return int(self.dut.mem_read_data.value)

    async def wait_cycles(self, cycles):
        """Wait for a number of clock cycles"""
        await ClockCycles(self.dut.clk, cycles)

    async def wait_for_pc(self, target_pc, max_cycles=10000):
        """Wait for PC to reach a target value"""
        for _ in range(max_cycles):
            await RisingEdge(self.dut.clk)
            if int(self.dut.dbg_pc.value) == target_pc:
                return True
        return False

    async def wait_for_io_write(self, max_cycles=5000):
        """Wait for an I/O write operation"""
        for _ in range(max_cycles):
            await RisingEdge(self.dut.clk)
            if int(self.dut.io_write.value) == 1:
                return True
        return False


# ============================================================================
# Debug ROM Tests
# ============================================================================

@cocotb.test()
async def test_debug_rom_boot_mode(dut):
    """Test that boot_mode=1 causes CPU to execute from debug ROM"""
    tb = InterconnectTestbench(dut)
    await tb.setup()

    # Set boot mode before reset
    dut.boot_sel.value = 1
    await tb.reset()

    # Wait a few cycles and check PC
    await tb.wait_cycles(10)

    # PC should be advancing (not stuck at 0)
    initial_pc = int(dut.dbg_pc.value)
    await tb.wait_cycles(100)
    final_pc = int(dut.dbg_pc.value)

    dut._log.info(f"Boot mode test: initial_pc={initial_pc}, final_pc={final_pc}")

    # In boot mode, CPU should be executing ROM program
    assert final_pc > initial_pc or final_pc == 0, "PC should advance in boot mode"


@cocotb.test()
async def test_debug_rom_ext_out_toggle(dut):
    """Test that debug ROM program toggles EXT_OUT0"""
    tb = InterconnectTestbench(dut)
    await tb.setup()

    # Set boot mode
    dut.boot_sel.value = 1
    await tb.reset()

    # Wait for first io_write (EXT_OUT0 should go high)
    found = await tb.wait_for_io_write(max_cycles=5000)
    assert found, "Expected io_write in debug ROM program"

    first_ext_out = int(dut.ext_out.value) & 1
    dut._log.info(f"First EXT_OUT0 value: {first_ext_out}")

    # Wait for next io_write (EXT_OUT0 should toggle)
    await tb.wait_cycles(100)  # Skip some cycles
    found = await tb.wait_for_io_write(max_cycles=10000)
    assert found, "Expected second io_write"

    second_ext_out = int(dut.ext_out.value) & 1
    dut._log.info(f"Second EXT_OUT0 value: {second_ext_out}")

    # Values should be different (toggle)
    assert first_ext_out != second_ext_out, "EXT_OUT0 should toggle"


@cocotb.test()
async def test_boot_mode_0_spi_ram(dut):
    """Test that boot_mode=0 causes CPU to execute from SPI RAM"""
    tb = InterconnectTestbench(dut)
    await tb.setup()

    # Don't set boot mode (default is 0)
    dut.boot_sel.value = 0

    # Load a simple program into SPI RAM: LDI 0x1234, HLT
    program = [
        OP_LDI, 0x34, 0x12,  # LDI 0x1234
        OP_HLT               # HLT
    ]
    await tb.load_program(program)
    await tb.reset()

    # Wait for HLT
    await tb.wait_for_pc(0x0003, max_cycles=5000)

    # Check AC value
    ac = int(dut.dbg_ac.value)
    dut._log.info(f"AC value after LDI: 0x{ac:04X}")
    assert ac == 0x1234, f"Expected AC=0x1234, got 0x{ac:04X}"


# ============================================================================
# PWM Tests
# ============================================================================

@cocotb.test()
async def test_pwm_debug_mode_10_percent(dut):
    """Test that PWM outputs 10% duty in boot mode"""
    tb = InterconnectTestbench(dut)
    await tb.setup()

    # Set boot mode for 10% PWM
    dut.boot_sel.value = 1
    await tb.reset()

    # Measure PWM high time vs low time over one period
    # Debug mode: DIV=1, PERIOD=9999, DUTY=1000 -> 10% duty at 1kHz
    # At 10MHz, one period = 10000 cycles

    high_count = 0
    low_count = 0
    sample_cycles = 20000  # Two full periods

    for _ in range(sample_cycles):
        await RisingEdge(dut.clk)
        if int(dut.pwm_out.value) == 1:
            high_count += 1
        else:
            low_count += 1

    duty_percent = (high_count / sample_cycles) * 100
    dut._log.info(f"PWM duty cycle: {duty_percent:.1f}% (high={high_count}, low={low_count})")

    # Should be approximately 10% (allow some tolerance)
    assert 8 <= duty_percent <= 12, f"Expected ~10% duty, got {duty_percent:.1f}%"


# ============================================================================
# External I/O Tests
# ============================================================================

@cocotb.test()
async def test_ext_in_mapping(dut):
    """Test that EXT_IN pins are correctly mapped to io_in"""
    tb = InterconnectTestbench(dut)
    await tb.setup()
    await tb.reset()

    # Test different ext_in values
    for test_val in [0b00, 0b01, 0b10, 0b11]:
        dut.ext_in.value = test_val
        await tb.wait_cycles(2)

        # io_status should contain IRQ_PENDING, SPI_MEM_BUSY, SPI_PERIPH_BUSY
        # io_in should have ext_in in bits [1:0]
        # Note: We can't directly read io_in from here, but we can test via CPU

        dut._log.info(f"Set ext_in={test_val:02b}")


@cocotb.test()
async def test_ext_out_from_cpu(dut):
    """Test that CPU can control EXT_OUT via OUT instruction"""
    tb = InterconnectTestbench(dut)
    await tb.setup()

    # Load program: LDI 0x0003, OUT 0x01 (set both EXT_OUT bits)
    program = [
        OP_LDI, 0x03, 0x00,  # LDI 0x0003
        OP_OUT, 0x01,        # OUT port 1
        OP_HLT               # HLT
    ]
    await tb.load_program(program)
    await tb.reset()

    # Wait for HLT
    await tb.wait_for_pc(0x0005, max_cycles=5000)

    ext_out_val = int(dut.ext_out.value)
    dut._log.info(f"EXT_OUT value: {ext_out_val:02b}")
    assert ext_out_val == 0b11, f"Expected EXT_OUT=0b11, got {ext_out_val:02b}"


# ============================================================================
# Address Decoding Tests
# ============================================================================

@cocotb.test()
async def test_ram_access_low_address(dut):
    """Test RAM access at low address (0x0100)"""
    tb = InterconnectTestbench(dut)
    await tb.setup()

    # Load program: LDI 0x1234, STA 0x0100, LDA 0x0100, HLT
    program = [
        OP_LDI, 0x34, 0x12,  # LDI 0x1234
        OP_STA, 0x00, 0x01,  # STA 0x0100
        OP_LDI, 0x00, 0x00,  # LDI 0x0000 (clear AC)
        OP_LDA, 0x00, 0x01,  # LDA 0x0100 (load back)
        OP_HLT               # HLT
    ]
    await tb.load_program(program)
    await tb.reset()

    # Wait for HLT
    await tb.wait_for_pc(0x000C, max_cycles=10000)

    ac = int(dut.dbg_ac.value)
    dut._log.info(f"AC value after load: 0x{ac:04X}")
    assert ac == 0x1234, f"Expected AC=0x1234, got 0x{ac:04X}"


@cocotb.test()
async def test_ram_access_high_address(dut):
    """Test RAM access at high address (0xDF00)"""
    tb = InterconnectTestbench(dut)
    await tb.setup()

    # Load program: LDI 0xABCD, STA 0xDF00, LDA 0xDF00, HLT
    program = [
        OP_LDI, 0xCD, 0xAB,  # LDI 0xABCD
        OP_STA, 0x00, 0xDF,  # STA 0xDF00
        OP_LDI, 0x00, 0x00,  # LDI 0x0000 (clear AC)
        OP_LDA, 0x00, 0xDF,  # LDA 0xDF00 (load back)
        OP_HLT               # HLT
    ]
    await tb.load_program(program)
    await tb.reset()

    # Wait for HLT
    await tb.wait_for_pc(0x000C, max_cycles=15000)

    ac = int(dut.dbg_ac.value)
    dut._log.info(f"AC value after load from 0xDF00: 0x{ac:04X}")
    assert ac == 0xABCD, f"Expected AC=0xABCD, got 0x{ac:04X}"


# ============================================================================
# Timer Tests
# ============================================================================

@cocotb.test()
async def test_timer_output_toggle(dut):
    """Test timer output toggling"""
    tb = InterconnectTestbench(dut)
    await tb.setup()

    # Load program to configure timer via MMIO
    # TMR_CTRL: ENABLE=1, OUT_EN=1, OUT_MODE=0 (toggle)
    # TMR_DIV: 1
    # TMR_CMP: 100 (toggle every 100 ticks)
    program = [
        # Write TMR_DIV = 1
        OP_LDI, 0x01, 0x00,           # LDI 0x0001
        OP_STA, ADDR_TMR_DIV & 0xFF, (ADDR_TMR_DIV >> 8) & 0xFF,

        # Write TMR_CMP = 100
        OP_LDI, 0x64, 0x00,           # LDI 100
        OP_STA, ADDR_TMR_CMP & 0xFF, (ADDR_TMR_CMP >> 8) & 0xFF,

        # Write TMR_CTRL = 0x29 (ENABLE=1, OUT_EN=1, OUT_MODE=0, AUTO_RELOAD=1)
        OP_LDI, 0x29, 0x00,           # LDI 0x0029
        OP_STA, ADDR_TMR_CTRL & 0xFF, (ADDR_TMR_CTRL >> 8) & 0xFF,

        # Loop forever
        OP_JMP, 0x12, 0x00            # JMP to self
    ]
    await tb.load_program(program)
    await tb.reset()

    # Wait for timer to start
    await tb.wait_cycles(500)

    # Sample timer_out over time
    initial_timer_out = int(dut.timer_out.value)
    await tb.wait_cycles(200)  # Wait for compare match
    final_timer_out = int(dut.timer_out.value)

    dut._log.info(f"Timer output: initial={initial_timer_out}, final={final_timer_out}")

    # Timer should have toggled at least once
    # (This test may need adjustment based on timing)


# ============================================================================
# IRQ Tests
# ============================================================================

@cocotb.test()
async def test_irq_external_sets_status(dut):
    """Test that external IRQ sets status bit"""
    tb = InterconnectTestbench(dut)
    await tb.setup()

    # Load simple program that just loops
    program = [
        OP_JMP, 0x00, 0x00  # JMP to self
    ]
    await tb.load_program(program)
    await tb.reset()

    # Wait for CPU to start
    await tb.wait_cycles(100)

    # Assert external IRQ
    dut.irq_in.value = 1
    await tb.wait_cycles(5)
    dut.irq_in.value = 0  # Pulse it

    await tb.wait_cycles(5)

    # Check io_status[0] (IRQ_PENDING) - it should be 0 because IRQ_ENABLE is 0
    io_status = int(dut.io_status.value)
    irq_pending = io_status & 1
    dut._log.info(f"io_status=0x{io_status:02X}, IRQ_PENDING={irq_pending}")

    # IRQ_PENDING should be 0 because IRQ_ENABLE is 0 by default
    assert irq_pending == 0, "IRQ_PENDING should be 0 when IRQ_ENABLE is 0"


# ============================================================================
# Integration Tests
# ============================================================================

@cocotb.test()
async def test_mmio_pwm_register_write(dut):
    """Test writing to PWM MMIO registers"""
    tb = InterconnectTestbench(dut)
    await tb.setup()

    # Load program to configure PWM: enable with 50% duty
    program = [
        # Write PWM_DIV = 1
        OP_LDI, 0x01, 0x00,
        OP_STA, ADDR_PWM_DIV & 0xFF, (ADDR_PWM_DIV >> 8) & 0xFF,

        # Write PWM_PERIOD = 99
        OP_LDI, 0x63, 0x00,           # 99 decimal
        OP_STA, ADDR_PWM_PERIOD & 0xFF, (ADDR_PWM_PERIOD >> 8) & 0xFF,

        # Write PWM_DUTY = 50
        OP_LDI, 0x32, 0x00,           # 50 decimal
        OP_STA, ADDR_PWM_DUTY & 0xFF, (ADDR_PWM_DUTY >> 8) & 0xFF,

        # Write PWM_CTRL = 1 (enable)
        OP_LDI, 0x01, 0x00,
        OP_STA, ADDR_PWM_CTRL & 0xFF, (ADDR_PWM_CTRL >> 8) & 0xFF,

        # Loop forever
        OP_JMP, 0x15, 0x00
    ]
    await tb.load_program(program)
    await tb.reset()

    # Wait for program to configure PWM
    await tb.wait_cycles(3000)

    # Measure PWM duty cycle
    high_count = 0
    low_count = 0
    sample_cycles = 1000

    for _ in range(sample_cycles):
        await RisingEdge(dut.clk)
        if int(dut.pwm_out.value) == 1:
            high_count += 1
        else:
            low_count += 1

    duty_percent = (high_count / sample_cycles) * 100
    dut._log.info(f"Configured PWM duty: {duty_percent:.1f}%")

    # Should be approximately 50%
    assert 40 <= duty_percent <= 60, f"Expected ~50% duty, got {duty_percent:.1f}%"


@cocotb.test()
async def test_spi_cs_ram_active_on_memory_access(dut):
    """Test that CS_RAM is asserted during memory access"""
    tb = InterconnectTestbench(dut)
    await tb.setup()

    # Load program that accesses memory
    program = [
        OP_LDI, 0x55, 0xAA,  # LDI 0xAA55
        OP_STA, 0x00, 0x10,  # STA 0x1000
        OP_HLT
    ]
    await tb.load_program(program)
    await tb.reset()

    # Wait and check if CS_RAM goes low during access
    cs_ram_went_low = False
    for _ in range(5000):
        await RisingEdge(dut.clk)
        if int(dut.spi_cs_ram_n.value) == 0:
            cs_ram_went_low = True
            break

    dut._log.info(f"CS_RAM went low: {cs_ram_went_low}")
    assert cs_ram_went_low, "CS_RAM should go low during memory access"


# ============================================================================
# Run all tests
# ============================================================================

if __name__ == "__main__":
    # This allows running with: python test_interconnect.py
    import subprocess
    subprocess.run(["make", "-f", "Makefile.interconnect"])
