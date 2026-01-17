"""
Debug test to trace CPU execution
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles


class Op:
    LDI  = 0xE0
    OUT  = 0xD0
    HLT  = 0xF0


class DebugTB:
    def __init__(self, dut):
        self.dut = dut

    async def setup(self):
        clock = Clock(self.dut.clk, 10, units="ns")
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
        self.dut.reset.value = 1
        await ClockCycles(self.dut.clk, 5)
        self.dut.reset.value = 0
        await ClockCycles(self.dut.clk, 2)

    async def load_byte(self, addr, data):
        self.dut.mem_load_en.value = 1
        self.dut.mem_load_addr.value = addr
        self.dut.mem_load_data.value = data
        await RisingEdge(self.dut.clk)
        self.dut.mem_load_en.value = 0
        await RisingEdge(self.dut.clk)


@cocotb.test()
async def test_debug_trace(dut):
    """Debug test - trace PC and signals"""
    tb = DebugTB(dut)
    await tb.setup()

    # Simplest possible program: LDI 0x42, OUT, HLT
    program = [
        Op.LDI, 0x42,    # 0x00-0x01
        Op.OUT, 0x00,    # 0x02-0x03
        Op.HLT,          # 0x04
    ]

    dut._log.info("Loading program: LDI 0x42, OUT 0, HLT")
    for i, byte in enumerate(program):
        await tb.load_byte(i, byte)
        dut._log.info(f"  MEM[0x{i:02X}] = 0x{byte:02X}")

    # Verify memory loaded correctly
    for i, byte in enumerate(program):
        tb.dut.mem_read_addr.value = i
        await RisingEdge(tb.dut.clk)
        val = int(tb.dut.mem_read_data.value)
        dut._log.info(f"  Verify MEM[0x{i:02X}] = 0x{val:02X} (expected 0x{byte:02X})")

    await tb.reset()

    dut._log.info("Starting execution trace...")

    last_pc = -1
    last_mem_ready = -1
    for cycle in range(500):  # Extended for full program execution
        await RisingEdge(dut.clk)

        pc = int(dut.dbg_pc.value) if dut.dbg_pc.value.is_resolvable else -1
        ac = int(dut.dbg_ac.value) if dut.dbg_ac.value.is_resolvable else -1
        ri = int(dut.dbg_ri.value) if dut.dbg_ri.value.is_resolvable else -1
        io_wr = int(dut.io_write.value) if dut.io_write.value.is_resolvable else -1
        io_out = int(dut.io_out.value) if dut.io_out.value.is_resolvable else -1

        # Access internal signals through hierarchy
        try:
            cpu_mem_addr = int(dut.cpu_mem_addr.value)
            cpu_mem_req = int(dut.cpu_mem_req.value)
            cpu_mem_ready = int(dut.cpu_mem_ready.value)
            cpu_mem_data_in = int(dut.cpu_mem_data_in.value)
            spi_addr_latch = int(dut.spi_addr_latch.value)
            spi_state = int(dut.spi_state.value)
        except:
            cpu_mem_addr = -1
            cpu_mem_req = -1
            cpu_mem_ready = -1
            cpu_mem_data_in = -1
            spi_addr_latch = -1
            spi_state = -1

        # Print on mem_ready edge or every 10 cycles for first 100, every 50 after
        print_it = False
        if cycle < 100:
            if cpu_mem_ready == 1 and last_mem_ready != 1:
                print_it = True
            if cycle % 10 == 0:
                print_it = True
        else:
            if pc != last_pc or cycle % 50 == 0:
                print_it = True

        if print_it:
            dut._log.info(f"Cycle {cycle:4d}: PC=0x{pc:02X} RI=0x{ri:02X} | cpu_addr=0x{cpu_mem_addr:02X} req={cpu_mem_req} ready={cpu_mem_ready} data_in=0x{cpu_mem_data_in:02X} | spi_st={spi_state} spi_latch=0x{spi_addr_latch:02X}")

        last_pc = pc
        last_mem_ready = cpu_mem_ready

        if io_wr == 1:
            dut._log.info(f"I/O WRITE detected! Value = 0x{io_out:02X}")
            assert io_out == 0x42, f"Expected 0x42, got 0x{io_out:02X}"
            dut._log.info("PASSED!")
            return

    dut._log.warning("Test ended without I/O write (500 cycles traced)")
