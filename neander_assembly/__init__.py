"""
Neander-X 16-bit Assembler Package
==================================

This package provides an assembler for the Neander-X 16-bit CPU and
helper utilities for loading programs in cocotb simulations.

Quick start:
    from neander_assembly import assemble_file, NeanderLoader

    # Simple usage
    program = assemble_file("path/to/program.s")

    # With symbol access
    loader = NeanderLoader()
    program = loader.assemble_file("path/to/program.s")
    main_addr = loader.get_symbol("_main")
"""

from neander_assembly.neanderasm import NeanderAssembler
from neander_assembly.neander_loader import (
    NeanderLoader,
    AssemblyError,
    assemble,
    assemble_file,
)

__all__ = [
    'NeanderAssembler',
    'NeanderLoader',
    'AssemblyError',
    'assemble',
    'assemble_file',
]

__version__ = '1.0.0'
