"""
Neander Assembly Wrapper for cocotb tests
==========================================

Provides NeanderLoader class for assembling and loading programs.
"""

import os
import sys

# Add the neander_assembly directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'neander_assembly'))

from neanderasm import NeanderAssembler


class AssemblyError(Exception):
    """Exception raised for assembly errors."""
    pass


class NeanderLoader:
    """Loader for Neander-X assembly programs."""

    def __init__(self, add_startup=False):
        """
        Initialize the loader.

        Args:
            add_startup: If True, add a JMP to _main at address 0
        """
        self.add_startup = add_startup
        self.assembler = None
        self.symbols = {}

    def assemble_file(self, filepath):
        """
        Assemble an assembly file and return the binary.

        Args:
            filepath: Path to the .s file

        Returns:
            bytes: Assembled binary
        """
        with open(filepath, 'r') as f:
            source = f.read()

        return self.assemble(source)

    def assemble(self, source):
        """
        Assemble source code and return the binary.

        Args:
            source: Assembly source code string

        Returns:
            bytes: Assembled binary
        """
        # If add_startup is True, we need to modify the source to add
        # a proper startup sequence that calls _main and then halts
        if self.add_startup:
            # Find and replace .org 0x0000 with a startup that reserves space
            # The startup will be: CALL _main, HLT (4 bytes total)
            # We'll shift the original code by 4 bytes
            source = self._add_startup_wrapper(source)

        self.assembler = NeanderAssembler()
        success = self.assembler.assemble(source)

        if not success:
            errors = '\n'.join(self.assembler.errors)
            raise AssemblyError(f"Assembly failed:\n{errors}")

        # Store symbols for later lookup
        self.symbols = self.assembler.symbols.copy()

        # Get the binary output
        binary = list(self.assembler.get_binary())

        return bytes(binary)

    def _add_startup_wrapper(self, source):
        """
        Add startup wrapper code that calls _main and halts.

        The startup code:
            .org 0x0000
            CALL _main
            HLT
            ; original code follows

        Args:
            source: Original assembly source

        Returns:
            str: Modified source with startup wrapper
        """
        import re

        # Check if source has .org 0x0000 or .org 0x0
        org_match = re.search(r'\.org\s+0x0+\s*\n', source, re.IGNORECASE)

        if org_match:
            # Replace the .org with our startup code
            startup = """; Startup wrapper
    .org 0x0000
_start:
    CALL _main
    HLT

; Original code continues at 0x0004
"""
            # Insert startup and adjust the rest
            source = source[:org_match.start()] + startup + source[org_match.end():]
        else:
            # No .org found, prepend startup
            startup = """; Startup wrapper
    .org 0x0000
_start:
    CALL _main
    HLT

"""
            source = startup + source

        return source

    def get_symbol(self, name):
        """
        Get the address of a symbol.

        Args:
            name: Symbol name (e.g., '_main')

        Returns:
            int: Symbol address
        """
        if name in self.symbols:
            return self.symbols[name]
        raise KeyError(f"Symbol not found: {name}")

    def get_binary_list(self):
        """Get the assembled binary as a list of integers."""
        if self.assembler is None:
            return []
        return self.assembler.get_cocotb_list()
