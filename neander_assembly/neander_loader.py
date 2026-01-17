#!/usr/bin/env python3
"""
Neander Program Loader for Cocotb
=================================

Helper module to load Neander assembly programs into the cocotb testbench.

Usage in cocotb tests:
    from neander_assembly.neander_loader import NeanderLoader

    loader = NeanderLoader()
    program = loader.assemble_file("path/to/program.s")
    await tb.load_program(program)

    # Or load pre-assembled binary
    program = loader.load_binary("path/to/program.bin")
    await tb.load_program(program)
"""

import os
import sys
from typing import Dict, List, Optional, Tuple

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from neander_assembly.neanderasm import NeanderAssembler


class NeanderLoader:
    """Helper class for loading Neander programs in cocotb tests."""

    # JMP opcode for startup code
    JMP_OPCODE = 0x80

    def __init__(self, add_startup=True):
        """Initialize loader.

        Args:
            add_startup: If True, inject startup code that jumps to _main
        """
        self.assembler: Optional[NeanderAssembler] = None
        self.symbols: Dict[str, int] = {}
        self.errors: List[str] = []
        self.add_startup = add_startup

    def _inject_startup_code(self, source: str) -> str:
        """Inject startup code that calls _main and halts.

        Modifies the assembly source to:
        1. Add CALL _main + HLT at address 0x0000 (4 bytes)
        2. Move the original .org 0x0000 data to 0x0004

        Using CALL instead of JMP allows _main to RET properly.

        Args:
            source: Original assembly source

        Returns:
            Modified source with startup code
        """
        import re

        # Startup code: CALL _main (3 bytes) + HLT (1 byte) = 4 bytes
        startup_size = 4

        # Check if source has .org 0x0000 or .org 0
        has_org_zero = re.search(r'\.org\s+(0x0+|0)\s*$', source, re.MULTILINE)

        if not has_org_zero:
            # No data section at 0, just prepend startup
            startup = f"""; Startup code - call main and halt
    .org 0x0000
    CALL _main
    HLT

"""
            return startup + source

        # Source has .org 0x0000 - we need to:
        # 1. Replace first .org 0x0000 with .org 0x0004 (after CALL+HLT)
        # 2. Prepend startup code at 0x0000

        # Find the first .org 0x0000 or .org 0 and replace with .org 0x0004
        modified = re.sub(
            r'(\.org\s+)(0x0+|0)(\s*$)',
            f'\\g<1>0x{startup_size:04X}\\3',
            source,
            count=1,
            flags=re.MULTILINE
        )

        startup = f"""; Startup code - call main and halt
    .org 0x0000
    CALL _main
    HLT

"""
        return startup + modified

    def assemble(self, source: str, inject_startup: bool = None) -> List[int]:
        """Assemble source code and return byte list for cocotb.

        Args:
            source: Assembly source code as string
            inject_startup: Override default startup injection setting

        Returns:
            List of bytes suitable for load_program()

        Raises:
            AssemblyError: If assembly fails
        """
        should_inject = inject_startup if inject_startup is not None else self.add_startup

        if should_inject:
            source = self._inject_startup_code(source)

        self.assembler = NeanderAssembler()
        if not self.assembler.assemble(source):
            self.errors = self.assembler.errors
            raise AssemblyError(f"Assembly failed:\n" + "\n".join(self.errors))

        self.symbols = self.assembler.symbols
        return self.assembler.get_cocotb_list()

    def assemble_file(self, filepath: str, inject_startup: bool = None) -> List[int]:
        """Assemble a file and return byte list for cocotb.

        Args:
            filepath: Path to assembly file
            inject_startup: Override default startup injection setting

        Returns:
            List of bytes suitable for load_program()

        Raises:
            FileNotFoundError: If file doesn't exist
            AssemblyError: If assembly fails
        """
        with open(filepath, 'r') as f:
            source = f.read()
        return self.assemble(source, inject_startup)

    def load_binary(self, filepath: str) -> List[int]:
        """Load a pre-assembled binary file.

        Args:
            filepath: Path to binary file

        Returns:
            List of bytes suitable for load_program()
        """
        with open(filepath, 'rb') as f:
            data = f.read()
        return list(data)

    def get_symbol(self, name: str) -> int:
        """Get the address of a symbol from the last assembly.

        Args:
            name: Symbol name

        Returns:
            Address of the symbol

        Raises:
            KeyError: If symbol not found
        """
        if name not in self.symbols:
            raise KeyError(f"Symbol not found: {name}")
        return self.symbols[name]

    def get_symbols(self) -> Dict[str, int]:
        """Get all symbols from the last assembly.

        Returns:
            Dictionary mapping symbol names to addresses
        """
        return self.symbols.copy()

    def create_simple_program(self, instructions: List[Tuple]) -> List[int]:
        """Create a simple program from instruction tuples.

        This is a convenience method for creating small test programs
        without writing full assembly.

        Args:
            instructions: List of tuples (mnemonic, operand) or (mnemonic,)

        Returns:
            List of bytes

        Example:
            program = loader.create_simple_program([
                ('LDI', 0x42),
                ('OUT', 0x00),
                ('HLT',),
            ])
        """
        from neander_assembly.neanderasm import (
            SINGLE_BYTE_OPCODES, IMM8_OPCODES, ADDR16_OPCODES
        )

        result = []
        for item in instructions:
            mnemonic = item[0].upper()
            operand = item[1] if len(item) > 1 else None

            if mnemonic in SINGLE_BYTE_OPCODES:
                result.append(SINGLE_BYTE_OPCODES[mnemonic])
            elif mnemonic in IMM8_OPCODES:
                result.append(IMM8_OPCODES[mnemonic])
                result.append(operand & 0xFF if operand else 0)
            elif mnemonic in ADDR16_OPCODES:
                result.append(ADDR16_OPCODES[mnemonic])
                if operand is not None:
                    result.append(operand & 0xFF)
                    result.append((operand >> 8) & 0xFF)
                else:
                    result.extend([0, 0])
            else:
                raise ValueError(f"Unknown mnemonic: {mnemonic}")

        return result


class AssemblyError(Exception):
    """Exception raised when assembly fails."""
    pass


# Convenience functions for quick use
def assemble(source: str) -> List[int]:
    """Quick assemble source code to byte list."""
    loader = NeanderLoader()
    return loader.assemble(source)


def assemble_file(filepath: str) -> List[int]:
    """Quick assemble file to byte list."""
    loader = NeanderLoader()
    return loader.assemble_file(filepath)


if __name__ == '__main__':
    # Simple test
    import argparse

    parser = argparse.ArgumentParser(description='Test Neander Loader')
    parser.add_argument('input', help='Input assembly file')
    args = parser.parse_args()

    loader = NeanderLoader()
    try:
        program = loader.assemble_file(args.input)
        print(f"Assembled {len(program)} bytes")
        print(f"Symbols: {loader.get_symbols()}")
        print(f"First 32 bytes: {[f'0x{b:02X}' for b in program[:32]]}")
    except (FileNotFoundError, AssemblyError) as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
